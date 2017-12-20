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
  const VectorX<double> x{
      VectorX<double>::LinSpaced(num_plotting_points, 0, 1)};

  // Plot a B-spline curve for random control points in R².
  std::default_random_engine rand_generator{1234};
  std::uniform_real_distribution<double> rand_distribution{};
  auto control_points = MatrixX<double>(2, num_control_points);
  for (int i = 0; i < control_points.rows(); ++i) {
    for (int j = 0; j < control_points.cols(); ++j) {
      control_points(i, j) = rand_distribution(rand_generator);
    }
  }
  drake::log()->debug("control_points = \n{}", control_points);
  PiecewisePolynomial<double> curve =
      basis.ConstructBsplineCurve(control_points);
  MatrixX<double> curve_values(2, x.size());
  for (int j = 0; j < num_plotting_points; ++j) {
    curve_values(0,j) = curve.value(x(j))(0);
    curve_values(1,j) = curve.value(x(j))(1);
  }
  CallPython("figure", 2);
  CallPython("clf");
  CallPython("plot", control_points.row(0).transpose(),
             control_points.row(1).transpose());
  drake::log()->debug("Start point: ({}, {})", control_points(0, 0),
             control_points(1,0));
  drake::log()->debug("End point:   ({}, {})", control_points(0, num_control_points-1),
             control_points(1,num_control_points-1));
  CallPython("plot", control_points(0, 0),
             control_points(1,0), ToPythonKwargs("marker", "v"));
  CallPython("plot", control_points(0, num_control_points-1),
             control_points(1,num_control_points-1), ToPythonKwargs("marker", "^"));
  CallPython("plot", curve_values.row(0).transpose(), curve_values.row(1).transpose());

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
    VectorX<double> bspline_values(x.size());
    for (int j = 0; j < num_plotting_points; ++j) {
      bspline_values(j) = basis.polynomials()[i].value(x(j))(0);
    }
    for (int k = 0; k <= derivatives_to_plot; ++k) {
      VectorX<double> y(x.size());
      for (int j = 0; j < num_plotting_points; ++j) {
        y(j) = basis.polynomials()[i].derivative(k).value(x(j))(0) *
               std::pow(knot_interval, k);
      }
      axes[k][0].attr("plot")(
          x, y,
          ToPythonKwargs("label",
                         "$B_{" + std::to_string(i) + std::to_string(order) +
                             "}" + std::string(k, '\'') + "(x)$"));
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
