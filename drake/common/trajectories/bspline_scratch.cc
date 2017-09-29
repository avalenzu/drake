#include <Eigen/Dense>
#include <gflags/gflags.h>

#include "drake/common/eigen_types.h"
#include "drake/common/proto/call_matlab.h"
#include "drake/common/text_logging.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/common/trajectories/piecewise_polynomial.h"

using drake::common::CallMatlab;
using drake::common::CallMatlabSingleOutput;

DEFINE_int32(index, 0, "Index of the B-spline to plot");
DEFINE_int32(order, 6, "Order of the B-splines");
DEFINE_int32(num_unique_knots, 11, "Number of unique knot points");
DEFINE_int32(num_end_point_knots, 6, "Multiplicity of knots at 0 and 1.");
DEFINE_int32(num_plotting_points, 1000, "Number of points to use when plotting.");
DEFINE_int32(derivatives_to_plot, 0, "Order of derivatives to plot.");
namespace drake {
namespace {
int DoMain() {
  const int kIndex{FLAGS_index};
  const int kOrder = FLAGS_order;
  const int kNumUniqueKnots = FLAGS_num_unique_knots;
  const int kNumEndPointKnots = FLAGS_num_end_point_knots;
  const int kNumPlottingPoints = FLAGS_num_plotting_points;
  const int kNumKnots{kNumUniqueKnots + 2 * (kNumEndPointKnots - 1)};
  const double kKnotInterval{1.0 / static_cast<double>(kNumUniqueKnots-1)};
  const int kDerivativesToPlot = FLAGS_derivatives_to_plot;
  std::vector<double> knots(kNumKnots, 0.0);
  for (int i = kNumEndPointKnots; i < kNumKnots; ++i) {
    knots[i] = std::min(1.0, knots[i - 1] + kKnotInterval);
  }
  PiecewisePolynomial<double> bspline =
      PiecewisePolynomial<double>::BSpline(kIndex, kOrder, knots);
  PiecewisePolynomial<double> omega_j =
      PiecewisePolynomial<double>::BSplineOmega(kIndex, kOrder, knots,
                                                bspline.getSegmentTimes());
  PiecewisePolynomial<double> omega_j_plus_1 =
      PiecewisePolynomial<double>::BSplineOmega(kIndex + 1, kOrder, knots,
                                                bspline.getSegmentTimes());
  const VectorX<double> x{VectorX<double>::LinSpaced(kNumPlottingPoints, 0, 1)};
  VectorX<double> bspline_values(x.size());
  VectorX<double> omega_j_values(x.size());
  VectorX<double> omega_j_plus_1_values(x.size());
  for (int i = 0; i < kNumPlottingPoints; ++i) {
    bspline_values(i) = bspline.value(x(i))(0);
    omega_j_values(i) = omega_j.value(x(i))(0);
    omega_j_plus_1_values(i) = omega_j_plus_1.value(x(i))(0);
  }
  drake::log()->debug("Knots: {}", Eigen::Map<VectorX<double>>(knots.data(), knots.size()).transpose());
  drake::log()->debug("Values: \n{}", bspline_values);
  CallMatlab("plot", x, bspline_values, "LineWidth", 2.0, "DisplayName", "B_{" + std::to_string(kIndex) + std::to_string(kOrder) + "}(x)");
  CallMatlab("hold", "on");
  for (int i = 1; i <= kDerivativesToPlot; ++i) {
    VectorX<double> y(x.size());
    for (int j = 0; j < kNumPlottingPoints; ++j) {
      y(j) = bspline.derivative(i).value(x(j))(0)*std::pow(kKnotInterval, i);
    }
    CallMatlab("plot", x, y, "DisplayName", "B_{" + std::to_string(kIndex) + std::to_string(kOrder) + "}^{" + std::string(i,'\'') + "}(x)");
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
