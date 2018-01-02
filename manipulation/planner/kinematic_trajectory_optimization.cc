#include "drake/manipulation/planner/kinematic_trajectory_optimization.h"

namespace drake {
namespace manipulation {
namespace planner {
namespace {
solvers::VectorXDecisionVariable MakeNamedVariables(const std::string& prefix,
                                                    int num) {
  solvers::VectorXDecisionVariable vars(num);
  for (int i = 0; i < num; i++)
    vars(i) = symbolic::Variable(prefix + std::to_string(i));
  return vars;
}
}
KinematicTrajectoryOptimization::KinematicTrajectoryOptimization(
    int num_positions, int num_control_points, int spline_order,
    double duration)
    : placeholder_t_var_(
          solvers::VectorDecisionVariable<1>(symbolic::Variable("t"))),
      placeholder_q_vars_(MakeNamedVariables("q", num_positions)),
      placeholder_v_vars_(MakeNamedVariables("v", num_positions)),
      placeholder_a_vars_(MakeNamedVariables("a", num_positions)),
      placeholder_j_vars_(MakeNamedVariables("j", num_positions)),
      position_curve_(
          BsplineBasis(spline_order, num_control_points),
          std::vector<MatrixX<double>>(
              num_control_points, MatrixX<double>::Zero(num_positions, 1))){};

void KinematicTrajectoryOptimization::AddLinearConstraint(
    const symbolic::Formula& f, Vector2<double> plan_interval) {
  formula_linear_constraints_.emplace_back(
      new FormulaWrapper({f, plan_interval}));
}

void KinematicTrajectoryOptimization::AddLinearConstraintToProgram(
    const FormulaWrapper& constraint, MathematicalProgram* prog) {
  std::vector<int> active_knots{
      ActiveKnotsForPlanInterval(*prog, constraint.plan_interval)};
  for (int index : active_knots) {
    prog->AddLinearConstraint(
        SubstitutePlaceholderVariables(constraint.formula, *prog, index));
  }
}
}  // namespace planner
}  // namespace manipulation
}  // namespace drake
