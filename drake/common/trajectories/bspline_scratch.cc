#include <Eigen/Dense>
#include <gflags/gflags.h>

#include "drake/common/eigen_types.h"
#include "drake/common/proto/call_matlab.h"
#include "drake/common/text_logging.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/common/trajectories/piecewise_polynomial.h"

using drake::common::CallMatlab;
using drake::common::CallMatlabSingleOutput;

DEFINE_int32(order, 4, "Order of the B-splines");
DEFINE_int32(num_control_points, 11, "Number of unique knot points");
DEFINE_int32(num_plotting_points, 1000, "Number of points to use when plotting.");
DEFINE_int32(derivatives_to_plot, 0, "Order of derivatives to plot.");
namespace drake {
namespace {
int DoMain() {
  const int kOrder = FLAGS_order;
  const int kNumControlPoints = FLAGS_num_control_points;
  const int kNumPlottingPoints = FLAGS_num_plotting_points;
  const int kNumKnots{kNumControlPoints + kOrder};
  const double kKnotInterval{1.0 / static_cast<double>(kNumControlPoints - (kOrder - 1))};
  const int kDerivativesToPlot = FLAGS_derivatives_to_plot;
  std::vector<double> knots(kNumKnots, 0.0);
  for (int i = kOrder; i < kNumKnots; ++i) {
    knots[i] = std::min(1.0, knots[i - 1] + kKnotInterval);
    drake::log()->info("Knot {}: {}", i, knots[i]);
  }
  for (int i = 0; i < kNumControlPoints; ++i) {
    PiecewisePolynomial<double> bspline =
        PiecewisePolynomial<double>::BSpline(i, kOrder, knots);
    PiecewisePolynomial<double> omega_j =
        PiecewisePolynomial<double>::BSplineOmega(i, kOrder, knots,
                                                  bspline.getSegmentTimes());
    PiecewisePolynomial<double> omega_j_plus_1 =
        PiecewisePolynomial<double>::BSplineOmega(i + 1, kOrder, knots,
                                                  bspline.getSegmentTimes());
    const VectorX<double> x{
        VectorX<double>::LinSpaced(kNumPlottingPoints, 0, 1)};
    VectorX<double> bspline_values(x.size());
    VectorX<double> omega_j_values(x.size());
    VectorX<double> omega_j_plus_1_values(x.size());
    for (int j = 0; j < kNumPlottingPoints; ++j) {
      bspline_values(j) = bspline.value(x(j))(0);
      omega_j_values(j) = omega_j.value(x(j))(0);
      omega_j_plus_1_values(j) = omega_j_plus_1.value(x(j))(0);
    }
    drake::log()->debug(
        "Knots: {}",
        Eigen::Map<VectorX<double>>(knots.data(), knots.size()).transpose());
    drake::log()->debug("Values: \n{}", bspline_values);
    CallMatlab(
        "plot", x, bspline_values, "LineWidth", 2.0, "DisplayName",
        "B_{" + std::to_string(i) + "," + std::to_string(kOrder) + "}(x)");
    CallMatlab("hold", "on");
    for (int k = 1; k <= kDerivativesToPlot; ++k) {
      VectorX<double> y(x.size());
      for (int j = 0; j < kNumPlottingPoints; ++j) {
        y(j) =
            bspline.derivative(k).value(x(j))(0) * std::pow(kKnotInterval, k);
      }
      CallMatlab("plot", x, y, "DisplayName",
                 "B_{" + std::to_string(k) + std::to_string(kOrder) +
                     "}^{" + std::string(k, '\'') + "}(x)");
    }
  }
  CallMatlab("legend","-DynamicLegend");
  CallMatlab("hold", "off");

  return 0;
}
} // namespace
} // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();
  return drake::DoMain();
}
