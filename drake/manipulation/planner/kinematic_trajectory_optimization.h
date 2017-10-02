#pragma once

#include <vector>

#include "drake/common/symbolic.h"
#include "drake/common/trajectories/piecewise_polynomial_trajectory.h"
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
   * Constructs a mathematical program whose decision variables are the control
   * points of a @p spline_order B-form spline. Constraints are enforced at @p
   * num_evaluation_points evenly spaced points along the trajectory, as well as
   * points corresponding to their start and end times.
   */
  KinematicTrajectoryOptimization(int num_positions, int num_control_points,
                                  int num_evaluation_points = -1,
                                  int spline_order = 4);

  /**
   * Constructs a mathematical program whose decision variables are the control
   * points of a @p spline_order B-form spline. Constraints are enforced at @p
   * num_evaluation_points evenly spaced points along the trajectory, as well as
   * points corresponding to their start and end times.
   */
  KinematicTrajectoryOptimization(const KinematicPlanningProblem* problem,
                                  int num_control_points,
                                  int num_evaluation_points = -1,
                                  int spline_order = 4);

  const solvers::MatrixXDecisionVariable& control_points() const {
    return control_points_;
  }

  const VectorX<symbolic::Expression> position(double evaluation_time) const;

  const VectorX<symbolic::Expression> velocity(double evaluation_time) const;

  const VectorX<symbolic::Expression> acceleration(double evaluation_time) const;

  const VectorX<symbolic::Expression> jerk(double evaluation_time) const;

  PiecewisePolynomialTrajectory ReconstructPositionTrajectory() const;

 private:
  void AddCost(const solvers::Binding<solvers::Cost>& cost,
               const Vector2<double> plan_interval);
  const KinematicPlanningProblem* problem_;
  const int kNumControlPoints_;
  const int kNumEvaluationPoints_;
  const int kOrder_;
  const int kNumKnots_;
  const int kNumInternalIntervals_;
  const int kNumPositions_;

  std::vector<double> evaluation_times_;
  const solvers::MatrixXDecisionVariable control_points_;

  std::vector<double> knots_;
  std::vector<PiecewisePolynomial<double>> basis_;
};

}  // namespace planner
}  // namespace manipulation
}  // namespace drake
