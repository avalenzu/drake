#include <gflags/gflags.h>

#include "drake/common/text_logging.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/manipulation/planner/kinematic_trajectory_optimization.h"

DEFINE_int32(order, 4, "Order of the B-splines");
DEFINE_int32(num_control_points, 11, "Number of unique knot points");

namespace drake {
namespace {
int DoMain() {
  const int order = FLAGS_order;
  const int num_control_points = FLAGS_num_control_points;
  const int num_positions = 2;
  manipulation::planner::KinematicTrajectoryOptimization prog{num_positions, num_control_points,
                                       order};
  return 0;
}
}  // namespace
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();
  return drake::DoMain();
}
