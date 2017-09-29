#pragma once

#include "drake/solvers/mathematical_program.h"
#include "drake/manipulation/planner/kinematic_planning_problem.h"

namespace drake {
namespace manipulation {
namespace planner {

/**
 * Optimizes the position trajectory of a multibody model. The trajectory is
 * represented as a 6-th order B-form spline.
 */
class KinematicTrajectoryOptimization : public solvers::MathematicalProgram {
 public:
  KinematicTrajectoryOptimization(const KinematicPlanningProblem& problem, int num_control_points);
};

} // namespace planner
} // namespace manipulation
} // namespace drake
