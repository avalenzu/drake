#include <Eigen/Dense>
#include <gflags/gflags.h>

#include "drake/common/eigen_types.h"
#include "drake/common/proto/call_python.h"
#include "drake/common/text_logging.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/common/trajectories/piecewise_polynomial.h"

using drake::common::CallPython;
using drake::common::ToPythonKwargs;

DEFINE_int32(order, 4, "Order of the B-splines");
DEFINE_int32(num_control_points, 11, "Number of unique knot points");
DEFINE_int32(num_plotting_points, 1000, "Number of points to use when plotting.");
DEFINE_int32(derivatives_to_plot, 0, "Order of derivatives to plot.");
namespace drake {
namespace {
int DoMain() {
  const int order = FLAGS_order;
  const int num_control_points = FLAGS_num_control_points;
  const int num_plotting_points = FLAGS_num_plotting_points;
  const int num_knots{num_control_points + order};
  const double knot_interval{1.0 / static_cast<double>(num_control_points - (order - 1))};
  const int derivatives_to_plot = FLAGS_derivatives_to_plot;
  std::vector<double> knots(num_knots, 0.0);
  for (int i = order; i < num_knots; ++i) {
    knots[i] = std::min(1.0, knots[i - 1] + knot_interval);
  }
  CallPython("figure", 1);
  CallPython("clf");
  auto fig_and_axes =
      CallPython("plt.subplots", derivatives_to_plot + 1, 1,
                 ToPythonKwargs("squeeze", false, "num", 1));
  //auto fig = fig_and_axes[0];
  auto axes = fig_and_axes[1];
  for (int i = 0; i < num_control_points; ++i) {
    PiecewisePolynomial<double> bspline =
        PiecewisePolynomial<double>::BSpline(i, order, knots);
    PiecewisePolynomial<double> omega_j =
        PiecewisePolynomial<double>::BSplineOmega(i, order, knots,
                                                  bspline.getSegmentTimes());
    PiecewisePolynomial<double> omega_j_plus_1 =
        PiecewisePolynomial<double>::BSplineOmega(i + 1, order, knots,
                                                  bspline.getSegmentTimes());
    const VectorX<double> x{
        VectorX<double>::LinSpaced(num_plotting_points, 0, 1)};
    VectorX<double> bspline_values(x.size());
    VectorX<double> omega_j_values(x.size());
    VectorX<double> omega_j_plus_1_values(x.size());
    for (int j = 0; j < num_plotting_points; ++j) {
      bspline_values(j) = bspline.value(x(j))(0);
      omega_j_values(j) = omega_j.value(x(j))(0);
      omega_j_plus_1_values(j) = omega_j_plus_1.value(x(j))(0);
    }
    drake::log()->debug(
        "Knots: {}",
        Eigen::Map<VectorX<double>>(knots.data(), knots.size()).transpose());
    drake::log()->debug("Values: \n{}", bspline_values);
    for (int k = 0; k <= derivatives_to_plot; ++k) {
      VectorX<double> y(x.size());
      for (int j = 0; j < num_plotting_points; ++j) {
        y(j) =
            bspline.derivative(k).value(x(j))(0) * std::pow(knot_interval, k);
      }
      axes[k][0].attr("plot")(x, y,
                 ToPythonKwargs("label", "$B_{" + std::to_string(k) +
                                             std::to_string(order) + "}" +
                                             std::string(k, '\'') + "(x)$"));
    }
  }
  for (int k = 0; k <= derivatives_to_plot; ++k) {
    axes[k][0].attr("legend")();
  }

  return 0;
}
} // namespace
} // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();
  return drake::DoMain();
}
