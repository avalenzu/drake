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

  KinematicTrajectoryOptimization(
      const BsplineCurve<double>& position_curve_seed);

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

  int num_evaluation_points() const { return num_evaluation_points_; }

  double min_knot_resolution() const { return min_knot_resolution_; }

  int initial_num_evaluation_points() const {
    return initial_num_evaluation_points_;
  }

  void set_num_evaluation_points(int num_evaluation_points) {
    num_evaluation_points_ = num_evaluation_points;
  }

  void set_min_knot_resolution(double min_knot_resolution) {
    min_knot_resolution_ = min_knot_resolution;
  }

  void set_initial_num_evaluation_points(int initial_num_evaluation_points) {
    initial_num_evaluation_points_ = initial_num_evaluation_points;
  }

  bool AreVariablesPresentInProgram(symbolic::Variables vars) const;

  void AddGenericPositionConstraint(
      const std::shared_ptr<solvers::Constraint>& constraint,
      const std::array<double, 2>& plan_interval);

  void AddLinearConstraint(const symbolic::Formula& f,
                           const std::array<double, 2>& plan_interval);

  void AddQuadraticCost(const symbolic::Expression& expression,
                        const std::array<double, 2>& plan_interval = {{0, 1}});

  solvers::SolutionResult Solve(bool always_update_curve = true);

  PiecewisePolynomial<double> GetPositionSolution(
      double time_scaling = 1) const;

  /// Add evaluation points to generic constraints if necessary.
  /// @returns true if constraints have been modified.
  bool UpdateGenericConstraints();

  /// Add knots to the position curve.
  /// @returns true if knots were added.
  bool AddKnots();

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

  void AddLinearConstraintToProgram(const FormulaWrapper& constraint,
                                    solvers::MathematicalProgram* prog) const;

  void AddQuadraticCostToProgram(const ExpressionWrapper& cost,
                                 solvers::MathematicalProgram* prog) const;

  void AddGenericPositionConstraintToProgram(
      const ConstraintWrapper& constraint,
      solvers::MathematicalProgram* prog) const;

  void AddPositionPointConstraintToProgram(
      const ConstraintWrapper& constraint, double evaluation_time,
      solvers::MathematicalProgram* prog) const;

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
  solvers::VectorXDecisionVariable placeholder_q_vars_;
  solvers::VectorXDecisionVariable placeholder_v_vars_;
  solvers::VectorXDecisionVariable placeholder_a_vars_;
  solvers::VectorXDecisionVariable placeholder_j_vars_;

  BsplineCurve<double> position_curve_;

  std::vector<FormulaWrapper> formula_linear_constraints_;

  std::vector<ExpressionWrapper> expression_quadratic_costs_;

  std::vector<ConstraintWrapper> generic_position_constraints_;

  std::vector<solvers::MatrixXDecisionVariable> control_point_variables_;

  int num_evaluation_points_{100};

  double min_knot_resolution_{1e-2};

  int initial_num_evaluation_points_{3};

  std::unique_ptr<solvers::MathematicalProgram> prog_{};

  bool is_program_empty_{true};
};
}  // namespace planner
}  // namespace manipulation
}  // namespace drake
