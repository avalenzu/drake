#pragma once

#include <array>

#include "drake/common/drake_optional.h"
#include "drake/manipulation/planner/bspline_curve.h"
#include "drake/solvers/binding.h"
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

  bool AreVariablesPresentInProgram(symbolic::Variables vars) const;

  void AddGenericPositionConstraint(
      const std::shared_ptr<solvers::Constraint>& constraint,
      const std::array<double, 2>& plan_interval);

  void AddLinearConstraint(const symbolic::Formula& f,
                           const std::array<double, 2>& plan_interval);

  void AddQuadraticCost(const symbolic::Expression& expression,
                        const std::array<double, 2>& plan_interval = {{0, 1}});

  solvers::SolutionResult Solve();

  PiecewisePolynomial<double> GetPositionSolution() const {
    return *position_curve_.piecwise_polynomial();
  }

  /// Add evaluation points to generic constraints if necessary.
  /// @returns true if constraints have been modified.
  bool UpdateGenericConstraints();

 private:
  struct FormulaWrapper {
    symbolic::Formula formula;
    std::array<double, 2> plan_interval;
  };

  struct ExpressionWrapper {
    symbolic::Expression expression;
    std::array<double, 2> plan_interval;
  };

  struct ConstraintWrapper {
    std::shared_ptr<solvers::Constraint> constraint;
    std::array<double, 2> plan_interval;
    int num_evaluation_points{2};
  };

  void AddLinearConstraintToProgram(const FormulaWrapper& constraint);

  void AddQuadraticCostToProgram(const ExpressionWrapper& cost);

  void AddGenericPositionConstraintToProgram(const ConstraintWrapper& constraint);

  std::vector<symbolic::Substitution> ConstructPlaceholderVariableSubstitution(
      const std::vector<solvers::MatrixXDecisionVariable>& control_points,
      const std::array<double, 2>& plan_interval) const;

  std::vector<symbolic::Formula> SubstitutePlaceholderVariables(
      const symbolic::Formula& f,
      const std::vector<solvers::MatrixXDecisionVariable>& control_points,
      const std::array<double, 2>& plan_interval) const;

  std::vector<symbolic::Expression> SubstitutePlaceholderVariables(
      const symbolic::Expression& expression,
      const std::vector<solvers::MatrixXDecisionVariable>& control_points,
      const std::array<double, 2>& plan_interval) const;

  // See description of the public time(), position(), velocity(),
  // acceleration() and jerk() accessor methods
  // for details about the placeholder variables.
  const solvers::VectorXDecisionVariable placeholder_q_vars_;
  const solvers::VectorXDecisionVariable placeholder_v_vars_;
  const solvers::VectorXDecisionVariable placeholder_a_vars_;
  const solvers::VectorXDecisionVariable placeholder_j_vars_;

  BsplineCurve<double> position_curve_;

  std::vector<std::unique_ptr<const FormulaWrapper>>
      formula_linear_constraints_;

  std::vector<std::unique_ptr<const ExpressionWrapper>>
      expression_quadratic_costs_;

  std::vector<std::unique_ptr<ConstraintWrapper>> generic_position_constraints_;

  optional<solvers::MathematicalProgram> prog_;
  std::vector<solvers::MatrixXDecisionVariable> control_point_variables_;

  int num_evaluation_points_{100};
};
}  // namespace planner
}  // namespace manipulation
}  // namespace drake
