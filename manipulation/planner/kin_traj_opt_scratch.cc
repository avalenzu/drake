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

namespace drake {
namespace {
int DoMain() {
  const int order = FLAGS_order;
  const int num_control_points = FLAGS_num_control_points;
  const int num_plotting_points = FLAGS_num_plotting_points;
  const int num_positions = 2;
  drake::log()->info("Running with:");
  drake::log()->info("  order: {}", order);
  drake::log()->info("  num_control_points: {}", num_control_points);
  manipulation::planner::KinematicTrajectoryOptimization prog{
      num_positions, num_control_points, order};
  prog.AddLinearConstraint(prog.position() == Vector2<double>(0.0, 0.0),
                           {{0, 0}});
  prog.AddLinearConstraint(prog.position() == Vector2<double>(1.0, 0.0),
                           {{1, 1}});
  solvers::SolutionResult result = prog.Solve();
  drake::log()->info("Solution result: {}", result);

  const auto curve = prog.GetPositionSolution();

  const VectorX<double> t{
      VectorX<double>::LinSpaced(num_plotting_points, 0, 1)};
  MatrixX<double> curve_values(num_positions, t.size());
  for (int plotting_point_index = 0; plotting_point_index < num_plotting_points;
       ++plotting_point_index) {
    for (int i = 0; i < num_positions; ++i) {
      curve_values(i, plotting_point_index) =
          curve.value(t(plotting_point_index))(i, 0);
    }
    //drake::log()->debug("Plotting point {}: {}", plotting_point_index, curve_values.col(plotting_point_index).transpose());
  }

  CallPython("figure", 2);
  CallPython("clf");
  CallPython("plot", curve_values.row(0).transpose(),
             curve_values.row(1).transpose(), ToPythonKwargs("marker", "o"));
  return 0;
}
}  // namespace
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();
  return drake::DoMain();
}
