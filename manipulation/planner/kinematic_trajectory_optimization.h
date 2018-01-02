#pragma once

#include "drake/manipulation/planner/bspline_curve.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace manipulation {
namespace planner {

/**
 * Optimizes the position trajectory of a multibody model. The trajectory is
 * represented as a B-form spline.
 */
class KinematicTrajectoryOptimization {
 public:
  /// Constructs a mathematical program whose decision variables are the control
  /// points of a @p spline_order B-form spline. Constraints are enforced at @p
  /// num_evaluation_points evenly spaced points along the trajectory, as well
  /// as
  /// points corresponding to their start and end times.
  KinematicTrajectoryOptimization(int num_positions, int num_control_points,
                                  int spline_order = 4, double duration = 1);

  /// Returns a placeholder decision variable (not actually declared as a
  /// decision variable in the MathematicalProgram) associated with the time, t.
  /// This variable will be substituted for real decision variables at
  /// particular times in methods like AddRunningCost.  Passing this variable
  /// directly into objectives/constraints for the parent classes will result
  /// in an error.
  const solvers::VectorDecisionVariable<1>& time() const {
    return placeholder_t_var_;
  }

  /// Returns a placeholder decision variable (not actually declared as a
  /// decision variable in the MathematicalProgram) associated with the
  /// generalized position vector, q.  This variable will be substituted for
  /// real decision variables at particular times in methods like
  /// AddRunningCost.  Passing this variable directly into
  /// objectives/constraints for the parent classes will result in an error.
  const solvers::VectorXDecisionVariable& position() const {
    return placeholder_q_vars_;
  }

  /// Returns a placeholder decision variable (not actually declared as a
  /// decision variable in the MathematicalProgram) associated with the
  /// generalized velocity vector, q.  This variable will be substituted for
  /// real decision variables at particular times in methods like
  /// AddRunningCost.  Passing this variable directly into
  /// objectives/constraints for the parent classes will result in an error.
  const solvers::VectorXDecisionVariable& velocity() const {
    return placeholder_v_vars_;
  }

  /// Returns a placeholder decision variable (not actually declared as a
  /// decision variable in the MathematicalProgram) associated with the
  /// generalized acceleration vector, q.  This variable will be substituted for
  /// real decision variables at particular times in methods like
  /// AddRunningCost.  Passing this variable directly into
  /// objectives/constraints for the parent classes will result in an error.
  const solvers::VectorXDecisionVariable& acceleration() const {
    return placeholder_a_vars_;
  }

  /// Returns a placeholder decision variable (not actually declared as a
  /// decision variable in the MathematicalProgram) associated with the
  /// generalized jerk vector, q.  This variable will be substituted for
  /// real decision variables at particular times in methods like
  /// AddRunningCost.  Passing this variable directly into
  /// objectives/constraints for the parent classes will result in an error.
  const solvers::VectorXDecisionVariable& jerk() const {
    return placeholder_j_vars_;
  }

  int num_positions() const {
    return position_curve_.control_points().front().rows();
  };

  void AddLinearConstraint(const symbolic::Formula& f,
                           Vector2<double> plan_interval);

 private:
  struct FormulaWrapper {
    symbolic::Formula formula;
    Vector2<double> plan_interval;
  };

  struct CostWrapper {
    std::shared_ptr<solvers::Cost> cost;
    solvers::VectorXDecisionVariable vars;
    Vector2<double> plan_interval;
  };

  struct ConstraintWrapper {
    std::shared_ptr<solvers::Constraint> constraint;
    solvers::VectorXDecisionVariable vars;
    Vector2<double> plan_interval;
  };

  void AddLinearConstraintToProgram(
      const FormulaWrapper& constraint,
      solvers::MathematicalProgram* prog);
  // See description of the public time(), position(), velocity(),
  // acceleration() and jerk() accessor methods
  // for details about the placeholder variables.
  const solvers::VectorDecisionVariable<1> placeholder_t_var_;
  const solvers::VectorXDecisionVariable placeholder_q_vars_;
  const solvers::VectorXDecisionVariable placeholder_v_vars_;
  const solvers::VectorXDecisionVariable placeholder_a_vars_;
  const solvers::VectorXDecisionVariable placeholder_j_vars_;

  BsplineCurve<double> position_curve_;

  std::vector<std::unique_ptr<const FormulaWrapper>>
      formula_linear_constraints_;
};
}  // namespace planner
}  // namespace manipulation
}  // namespace drake
