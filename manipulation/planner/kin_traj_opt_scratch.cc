#include <chrono>
#include <thread>

#include <gflags/gflags.h>

#include "drake/common/proto/call_python.h"
#include "drake/common/text_logging.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/manipulation/planner/kinematic_trajectory_optimization.h"
#include "drake/solvers/constraint.h"

using drake::common::CallPython;
using drake::common::ToPythonKwargs;

DEFINE_int32(order, 4, "Order of the B-splines");
DEFINE_int32(num_control_points, 15, "Number of unique knot points");
DEFINE_int32(num_plotting_points, 100,
             "Number of points to use when plotting.");
DEFINE_int32(derivatives_to_plot, 3, "Order of derivatives to plot.");
DEFINE_double(max_velocity, -1, "Maximum allowed velocity in any dimension.");
DEFINE_double(max_acceleration, -1,
              "Maximum allowed acceleration in any dimension.");
DEFINE_double(max_jerk, -1, "Maximum allowed jerk in any dimension.");
DEFINE_double(velocity_weight, -1, "Weight on squared velocity cost.");
DEFINE_double(acceleration_weight, -1, "Weight on squared acceleration cost.");
DEFINE_double(jerk_weight, -1, "Weight on squared jerk cost.");
DEFINE_double(mid_trajectory_slope, -1,
              "Slope for linear constraint on mid-section of trajectory.");
DEFINE_double(mid_trajectory_intercept, 2,
              "Intercept for linear constraint on mid-section of trajectory.");
DEFINE_double(mid_trajectory_tolerance, 0.1,
              "Tolerance for linear constraint on mid-section of trajectory.");
DEFINE_double(mid_trajectory_duration, 0.2,
              "Tolerance for linear constraint on mid-section of trajectory.");

namespace drake {
namespace {
int DoMain() {
  const int order = FLAGS_order;
  const int num_control_points = FLAGS_num_control_points;
  const int num_plotting_points = FLAGS_num_plotting_points;
  const int num_positions = 2;
  const int derivatives_to_plot = FLAGS_derivatives_to_plot;
  const double max_velocity = FLAGS_max_velocity;
  const double max_acceleration = FLAGS_max_acceleration;
  const double max_jerk = FLAGS_max_jerk;
  const double velocity_weight = FLAGS_velocity_weight;
  const double acceleration_weight = FLAGS_acceleration_weight;
  const double jerk_weight = FLAGS_jerk_weight;

  drake::log()->info("Running with:");
  drake::log()->info("  order: {}", order);
  drake::log()->info("  num_control_points: {}", num_control_points);
  manipulation::planner::KinematicTrajectoryOptimization prog{
      num_positions, num_control_points, order};
  const auto zero_vector = Vector2<double>::Zero();
  prog.AddLinearConstraint(prog.position() == Vector2<double>(0.0, 1.0),
                           {{0.0, 0.0}});
  prog.AddLinearConstraint(prog.position() == Vector2<double>(1.0, 0.0),
                           {{1.0, 1.0}});
  prog.AddLinearConstraint(prog.velocity() == zero_vector, {{0.0, 0.0}});
  prog.AddLinearConstraint(prog.velocity() == zero_vector, {{1.0, 1.0}});
  prog.AddLinearConstraint(prog.acceleration() == zero_vector, {{0.0, 0.0}});
  prog.AddLinearConstraint(prog.acceleration() == zero_vector, {{1.0, 1.0}});
  prog.AddLinearConstraint(prog.jerk() == zero_vector, {{0.0, 0.0}});
  prog.AddLinearConstraint(prog.jerk() == zero_vector, {{1.0, 1.0}});

  if (max_velocity > 0) {
    prog.AddLinearConstraint(
        prog.velocity() <= Vector2<double>::Constant(max_velocity),
        {{0.0, 1.0}});
    prog.AddLinearConstraint(
        prog.velocity() >= Vector2<double>::Constant(-max_velocity),
        {{0.0, 1.0}});
  }
  if (max_acceleration > 0) {
    prog.AddLinearConstraint(
        prog.acceleration() <= Vector2<double>::Constant(max_acceleration),
        {{0.0, 1.0}});
    prog.AddLinearConstraint(
        prog.acceleration() >= Vector2<double>::Constant(-max_acceleration),
        {{0.0, 1.0}});
  }
  if (max_jerk > 0) {
    prog.AddLinearConstraint(prog.jerk() <= Vector2<double>::Constant(max_jerk),
                             {{0.0, 1.0}});
    prog.AddLinearConstraint(
        prog.jerk() >= Vector2<double>::Constant(-max_jerk), {{0.0, 1.0}});
  }
  if (velocity_weight > 0) {
    prog.AddQuadraticCost(prog.velocity().transpose() * prog.velocity());
  }
  if (acceleration_weight > 0) {
    prog.AddQuadraticCost(prog.acceleration().transpose() *
                          prog.acceleration());
  }
  if (jerk_weight > 0) {
    prog.AddQuadraticCost(prog.jerk().transpose() * prog.jerk());
  }

  Vector2<double> center;
  center << 0.0, 0.0;
  double min_radius = 1.0;
  double max_radius = 1.0;
  drake::log()->debug("center = {}, min_radius = {}, max_radius = {}",
                      center.transpose(), min_radius, max_radius);

  prog.AddGenericPositionConstraint(
      std::make_shared<solvers::QuadraticConstraint>(
          2 * Matrix2<double>::Identity(), -2 * center,
          min_radius * min_radius - center.transpose() * center,
          max_radius * max_radius - center.transpose() * center),
      {{0.0, 1.0}});
  CallPython("exec", "from matplotlib.patches import Arc");
  do {
    solvers::SolutionResult result = prog.Solve();
    drake::log()->info("Solution result: {}", result);

    const PiecewisePolynomial<double> curve = prog.GetPositionSolution();

    const VectorX<double> t{
        VectorX<double>::LinSpaced(num_plotting_points, 0, 1)};
    MatrixX<double> curve_values(num_positions, t.size());

    CallPython("figure", 1);
    CallPython("clf");
    auto fig_and_axes = CallPython("plt.subplots", derivatives_to_plot + 1, 1,
                                   ToPythonKwargs("squeeze", false, "num", 1));
    auto axes = fig_and_axes[1];
    for (int derivative_order = 0; derivative_order <= derivatives_to_plot;
         ++derivative_order) {
      for (int plotting_point_index = 0;
           plotting_point_index < num_plotting_points; ++plotting_point_index) {
        for (int i = 0; i < num_positions; ++i) {
          curve_values(i, plotting_point_index) =
              curve.derivative(derivative_order)
                  .value(t(plotting_point_index))(i, 0);
        }
      }
      axes[derivative_order][0].attr("plot")(t,
                                             curve_values.row(0).transpose());
      axes[derivative_order][0].attr("plot")(t,
                                             curve_values.row(1).transpose());
      if (derivative_order == 0) {
        CallPython("figure", 2);
        CallPython("clf");
        CallPython("plot", curve_values.row(0).transpose(),
                   curve_values.row(1).transpose(),
                   ToPythonKwargs("marker", "o"));
        auto min_arc = CallPython("Arc", center, 2 * min_radius, 2 * min_radius,
                                 ToPythonKwargs("theta2", 90, "fill", false));
        auto max_arc = CallPython("Arc", center, 2 * max_radius, 2 * max_radius,
                                 ToPythonKwargs("theta2", 90, "fill", false));
        CallPython("gca").attr("add_patch")(min_arc);
        CallPython("gca").attr("add_patch")(max_arc);
        auto axes2 = CallPython("gca");
        axes2.attr("axis")("equal");
        CallPython("axis('equal')");
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  } while (prog.UpdateGenericConstraints());
  return 0;
}
}  // namespace
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();
  return drake::DoMain();
}
