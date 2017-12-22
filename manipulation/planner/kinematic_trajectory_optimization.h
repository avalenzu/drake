#pragma once

#include "drake/manipulation/planner/bspline_curve.h"

namespace drake {
namespace manipulation {
namespace planner {

/**
 * Optimizes the position trajectory of a multibody model. The trajectory is
 * represented as a B-form spline.
 */
class KinematicTrajectoryOptimization {
 public:
  /**
   * Constructs a mathematical program whose decision variables are the control
   * points of a @p spline_order B-form spline. Constraints are enforced at @p
   * num_evaluation_points evenly spaced points along the trajectory, as well as
   * points corresponding to their start and end times.
   */
  KinematicTrajectoryOptimization(int num_positions, int num_control_points,
                                  int spline_order = 4, double duration = 1);
 private:
  BsplineCurve<double> position_curve;
};
}  // namespace planner
}  // namespace manipulation
}  // namespace drake
