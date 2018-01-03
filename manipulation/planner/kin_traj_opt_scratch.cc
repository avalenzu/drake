#include <gflags/gflags.h>

#include "drake/common/proto/call_python.h"
#include "drake/common/text_logging.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/manipulation/planner/kinematic_trajectory_optimization.h"

using drake::common::CallPython;
using drake::common::ToPythonKwargs;

DEFINE_int32(order, 4, "Order of the B-splines");
DEFINE_int32(num_control_points, 11, "Number of unique knot points");
DEFINE_int32(num_plotting_points, 1000,
             "Number of points to use when plotting.");
DEFINE_int32(derivatives_to_plot, 0, "Order of derivatives to plot.");
DEFINE_double(max_velocity, -1, "Maximum allowed velocity in any dimension.");
DEFINE_double(max_acceleration, -1,
              "Maximum allowed acceleration in any dimension.");
DEFINE_double(max_jerk, -1, "Maximum allowed jerk in any dimension.");
DEFINE_double(velocity_weight, -1, "Weight on squared velocity cost.");
DEFINE_double(acceleration_weight, -1, "Weight on squared acceleration cost.");
DEFINE_double(jerk_weight, -1, "Weight on squared jerk cost.");

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
  prog.AddLinearConstraint(prog.position() == Vector2<double>(0.0, 1.0),
                           {{0.0, 0.0}});
  prog.AddLinearConstraint(prog.position()(0) + prog.position()(1) == 2.0,
                           {{0.4, 0.6}});
  prog.AddLinearConstraint(prog.position() == Vector2<double>(1.0, 0.0),
                           {{1.0, 1.0}});
  prog.AddLinearConstraint(prog.velocity() == Vector2<double>(0.0, 0.0),
                           {{0.0, 0.0}});
  prog.AddLinearConstraint(prog.velocity() == Vector2<double>(0.0, 0.0),
                           {{1.0, 1.0}});
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
    prog.AddQuadraticCost(prog.acceleration().transpose() * prog.acceleration());
  }
  if (jerk_weight > 0) {
    prog.AddQuadraticCost(prog.jerk().transpose() * prog.jerk());
  }
  // prog.AddQuadraticCost(prog.acceleration().transpose() *
  // prog.acceleration());
  // prog.AddQuadraticCost(prog.jerk().transpose() * prog.jerk());
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
    axes[derivative_order][0].attr("plot")(t, curve_values.row(0).transpose());
    axes[derivative_order][0].attr("plot")(t, curve_values.row(1).transpose());
    if (derivative_order == 0) {
      CallPython("figure", 2);
      CallPython("clf");
      CallPython("plot", curve_values.row(0).transpose(),
                 curve_values.row(1).transpose(),
                 ToPythonKwargs("marker", "o"));
      auto axes2 = CallPython("gca");
      axes2.attr("axis")("equal");
      CallPython("axis('equal')");
    }
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
