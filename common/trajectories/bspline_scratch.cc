#include <random>

#include <Eigen/Dense>
#include <gflags/gflags.h>

#include "drake/common/eigen_types.h"
#include "drake/common/proto/call_python.h"
#include "drake/common/text_logging.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/manipulation/planner/bspline_basis.h"

using drake::common::CallPython;
using drake::common::ToPythonKwargs;
using drake::manipulation::planner::BsplineBasis;

DEFINE_int32(order, 4, "Order of the B-splines");
DEFINE_int32(num_control_points, 11, "Number of unique knot points");
DEFINE_int32(control_point_cols, 2, "Number of columns in the control points");
DEFINE_int32(num_plotting_points, 1000,
             "Number of points to use when plotting.");
DEFINE_int32(derivatives_to_plot, 0, "Order of derivatives to plot.");
namespace drake {
namespace {
int DoMain() {
  const int order = FLAGS_order;
  const int num_control_points = FLAGS_num_control_points;
  const int num_plotting_points = FLAGS_num_plotting_points;
  const int derivatives_to_plot = FLAGS_derivatives_to_plot;

  BsplineBasis basis{order, num_control_points};
  const VectorX<double> t{
      VectorX<double>::LinSpaced(num_plotting_points, 0, 1)};

  // Plot a B-spline curve for random control points in R².
  std::default_random_engine rand_generator{1234};
  std::uniform_real_distribution<double> rand_distribution{};
  const int control_point_rows = 2;
  const int control_point_cols = FLAGS_control_point_cols;
  std::vector<MatrixX<double>> control_points;
  for (int control_point_index = 0; control_point_index < num_control_points;
       ++control_point_index) {
    control_points.push_back(
        MatrixX<double>(control_point_rows, control_point_cols));
    for (int i = 0; i < control_point_rows; ++i) {
      for (int j = 0; j < control_point_cols; ++j) {
        control_points.back()(i, j) = rand_distribution(rand_generator) + j;
      }
    }
  }
  PiecewisePolynomial<double> curve =
      basis.ConstructBsplineCurve(control_points);

  CallPython("figure", 2);
  CallPython("clf");
  for (int j = 0; j < control_point_cols; ++j) {
    MatrixX<double> curve_values(2, t.size());
    for (int plotting_point_index = 0;
         plotting_point_index < num_plotting_points; ++plotting_point_index) {
      curve_values(0, plotting_point_index) =
          curve.value(t(plotting_point_index))(0, j);
      curve_values(1, plotting_point_index) =
          curve.value(t(plotting_point_index))(1, j);
    }

    VectorX<double> control_points_x(num_control_points);
    VectorX<double> control_points_y(num_control_points);
    for (int control_point_index = 0; control_point_index < num_control_points;
         ++control_point_index) {
      control_points_x(control_point_index) =
          control_points[control_point_index](0, j);
      control_points_y(control_point_index) =
          control_points[control_point_index](1, j);
    }
    drake::log()->debug("control_points = \n{}\n{}",
                        control_points_x.transpose(),
                        control_points_y.transpose());
    CallPython("plot", control_points_x, control_points_y,
               ToPythonKwargs("marker", "x"));
    CallPython("plot", curve_values.row(0).transpose(),
               curve_values.row(1).transpose());

    // Try ploting a sparser set of points on the curve by constructing and
    // evaluating an expression.
    std::vector<MatrixX<symbolic::Variable>> control_points_symbolic(
        num_control_points);
    symbolic::Environment control_points_environment;
    for (int control_point_index = 0; control_point_index < num_control_points;
         ++control_point_index) {
      control_points_symbolic[control_point_index].resize(control_point_rows,
                                                          control_point_cols);
      for (int i = 0; i < control_point_rows; ++i) {
        for (int j = 0; j < control_point_cols; ++j) {
          control_points_symbolic[control_point_index](i, j) =
              symbolic::Variable("y[" + std::to_string(control_point_index) +
                                 "_" + std::to_string(i) + std::to_string(j));
          control_points_environment
              [control_points_symbolic[control_point_index](i, j)] =
                  control_points[control_point_index](i, j);
        }
      }
    }
    const int num_sparse_plotting_points = basis.knots().size();
    MatrixX<double> sparse_curve_values(control_point_rows,
                                        num_sparse_plotting_points);
    for (int sparse_plotting_point_index = 0;
         sparse_plotting_point_index < num_sparse_plotting_points;
         ++sparse_plotting_point_index) {
      MatrixX<symbolic::Expression> expression =
          basis.ConstructExpressionForCurveValue(
              control_points_symbolic,
              basis.knots()[sparse_plotting_point_index]);
      for (int i = 0; i < control_point_rows; ++i) {
        sparse_curve_values(i, sparse_plotting_point_index) =
            expression(i, j)
                .Evaluate(control_points_environment);
      }
    }
    CallPython("scatter", sparse_curve_values.row(0).transpose(),
               sparse_curve_values.row(1).transpose(),
               ToPythonKwargs("marker", "o"));
    drake::log()->debug("sparse_curve_values = \n{}\n{}",
                        sparse_curve_values.row(0), sparse_curve_values.row(1));
  }
  // Plot basis.
  CallPython("figure", 1);
  CallPython("clf");
  auto fig_and_axes = CallPython("plt.subplots", derivatives_to_plot + 1, 1,
                                 ToPythonKwargs("squeeze", false, "num", 1));
  // auto fig = fig_and_axes[0];
  auto axes = fig_and_axes[1];
  const double knot_interval{
      1.0 / static_cast<double>(num_control_points - (order - 1))};
  for (int i = 0; i < basis.num_control_points(); ++i) {
    VectorX<double> bspline_values(t.size());
    for (int j = 0; j < num_plotting_points; ++j) {
      bspline_values(j) = basis.polynomials()[i].value(t(j))(0);
    }
    for (int k = 0; k <= derivatives_to_plot; ++k) {
      VectorX<double> y(t.size());
      for (int j = 0; j < num_plotting_points; ++j) {
        y(j) = basis.polynomials()[i].derivative(k).value(t(j))(0) *
               std::pow(knot_interval, k);
      }
      axes[k][0].attr("plot")(
          t, y,
          ToPythonKwargs("label",
                         "$B_{" + std::to_string(i) + std::to_string(order) +
                             "}" + std::string(k, '\'') + "(t)$"));
    }
  }
  for (int k = 0; k <= derivatives_to_plot; ++k) {
    axes[k][0].attr("legend")();
  }

  return 0;
}
}  // namespace
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();
  return drake::DoMain();
}
