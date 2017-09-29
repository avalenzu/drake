#pragma once

#include "drake/manipulation/planner/kinematic_planning_problem.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace manipulation {
namespace planner {

/**
 * Optimizes the position trajectory of a multibody model. The trajectory is
 * represented as a B-form spline.
 */
class KinematicTrajectoryOptimization : public solvers::MathematicalProgram {
 public:
  /**
   * Constructs a
   */
  KinematicTrajectoryOptimization(const KinematicPlanningProblem& problem,
                                  int num_control_points,
                                  int num_evaluation_points = -1,
                                  int spline_order = 4);

 private:
  const int kNumControlPoints_;
  const int kNumEvaluationPoints_;
  const int kOrder_;
  const int kNumKnots_;
  const int kNumPositions_;

  const solvers::MatrixXDecisionVariable control_points_;
};

}  // namespace planner
}  // namespace manipulation
}  // namespace drake
