#include "drake/manipulation/planner/kinematic_trajectory_optimization.h"
#include "drake/common/text_logging.h"
#include "drake/solvers/scs_solver.h"

namespace drake {
namespace manipulation {
namespace planner {

using solvers::MatrixXDecisionVariable;
namespace {
solvers::VectorXDecisionVariable MakeNamedVariables(const std::string& prefix,
                                                    int num) {
  solvers::VectorXDecisionVariable vars(num);
  for (int i = 0; i < num; i++)
    vars(i) = symbolic::Variable(prefix + std::to_string(i));
  return vars;
}
}  // namespace
KinematicTrajectoryOptimization::KinematicTrajectoryOptimization(
    int num_positions, int num_control_points, int spline_order,
    double duration)
    : placeholder_q_vars_(MakeNamedVariables("q", num_positions)),
      placeholder_v_vars_(MakeNamedVariables("v", num_positions)),
      placeholder_a_vars_(MakeNamedVariables("a", num_positions)),
      placeholder_j_vars_(MakeNamedVariables("j", num_positions)),
      position_curve_(
          BsplineBasis(spline_order, num_control_points),
          std::vector<MatrixX<double>>(
              num_control_points, MatrixX<double>::Zero(num_positions, 1))){};

void KinematicTrajectoryOptimization::AddLinearConstraint(
    const symbolic::Formula& f, std::array<double, 2> plan_interval) {
  formula_linear_constraints_.emplace_back(
      new FormulaWrapper({f, plan_interval}));
}

void KinematicTrajectoryOptimization::AddQuadraticCost(
    const symbolic::Expression& f, std::array<double, 2> plan_interval) {
  expression_quadratic_costs_.emplace_back(
      new ExpressionWrapper({f, plan_interval}));
}

std::vector<symbolic::Substitution>
KinematicTrajectoryOptimization::ConstructPlaceholderVariableSubstitution(
    const std::vector<MatrixXDecisionVariable>& control_points,
    const std::vector<int>& active_control_point_indices) const {
  std::vector<symbolic::Substitution> sub{active_control_point_indices.size()};

  // Create symbolic curves
  BsplineCurve<symbolic::Expression> symbolic_q_curve{position_curve_.basis(),
                                                      control_points};
  BsplineCurve<symbolic::Expression> symbolic_v_curve{
      symbolic_q_curve.Derivative()};
  drake::log()->debug("symbolic_v_curve.control_points().size() = {}", symbolic_v_curve.control_points().size());
  BsplineCurve<symbolic::Expression> symbolic_a_curve{
      symbolic_v_curve.Derivative()};
  BsplineCurve<symbolic::Expression> symbolic_j_curve{
      symbolic_a_curve.Derivative()};

  for (int i = 0; i < num_positions(); ++i) {
    for (int j = 0; j < active_control_point_indices.size(); ++j) {
      sub[j].emplace(
          placeholder_q_vars_(i),
          symbolic_q_curve.control_points()[active_control_point_indices[j]](
              i));
      if (0 < j) {
        sub[j].emplace(
            placeholder_v_vars_(i),
            symbolic_v_curve
                .control_points()[active_control_point_indices[j-1]](i).Expand());
      }
      if (1 < j) {
        sub[j].emplace(
            placeholder_a_vars_(i),
            symbolic_a_curve
                .control_points()[active_control_point_indices[j - 2]](i).Expand());
      }
      if (2 < j) {
        sub[j].emplace(
            placeholder_j_vars_(i),
            symbolic_j_curve
                .control_points()[active_control_point_indices[j - 3]](i).Expand());
      }
    }
  }
  return sub;
}

std::vector<symbolic::Formula>
KinematicTrajectoryOptimization::SubstitutePlaceholderVariables(
    const symbolic::Formula& f,
    const std::vector<MatrixXDecisionVariable>& control_points,
    const std::vector<int>& active_control_point_indices) const {
  std::vector<symbolic::Formula> substitution_results;
  substitution_results.reserve(active_control_point_indices.size());
  for (const auto& substitution : ConstructPlaceholderVariableSubstitution(
           control_points, active_control_point_indices)) {
    substitution_results.push_back(f.Substitute(substitution));
  }
  return substitution_results;
}

std::vector<symbolic::Expression>
KinematicTrajectoryOptimization::SubstitutePlaceholderVariables(
    const symbolic::Expression& expression,
    const std::vector<MatrixXDecisionVariable>& control_points,
    const std::vector<int>& active_control_point_indices) const {
  std::vector<symbolic::Expression> substitution_results;
  substitution_results.reserve(active_control_point_indices.size());
  for (const auto& substitution : ConstructPlaceholderVariableSubstitution(
           control_points, active_control_point_indices)) {
    substitution_results.push_back(expression.Substitute(substitution));
  }
  return substitution_results;
}

bool KinematicTrajectoryOptimization::AreVariablesPresentInProgram(
    symbolic::Variables vars) const {
  if (!symbolic::intersect(symbolic::Variables(placeholder_q_vars_), vars)
           .empty()) {
    return false;
  }
  if (!symbolic::intersect(symbolic::Variables(placeholder_v_vars_), vars)
           .empty()) {
    return false;
  }
  if (!symbolic::intersect(symbolic::Variables(placeholder_a_vars_), vars)
           .empty()) {
    return false;
  }
  if (!symbolic::intersect(symbolic::Variables(placeholder_j_vars_), vars)
           .empty()) {
    return false;
  }
  return true;
}

void KinematicTrajectoryOptimization::AddLinearConstraintToProgram(
    const FormulaWrapper& constraint) {
  DRAKE_ASSERT(prog_ != nullopt);
  const std::vector<int> active_control_point_indices{
      position_curve_.basis().ComputeActiveControlPointIndices(
          constraint.plan_interval)};
  for (const auto& index : active_control_point_indices) {
    drake::log()->debug("Control point {} is active.", index);
  }
  const std::vector<symbolic::Formula> per_control_point_formulae =
      SubstitutePlaceholderVariables(constraint.formula,
                                     control_point_variables_,
                                     active_control_point_indices);
  for (const auto& f : per_control_point_formulae) {
    if (AreVariablesPresentInProgram(f.GetFreeVariables())) {
      drake::log()->debug("Adding linear constraint: {}", f);
      prog_->AddLinearConstraint(f);
    } else {
      drake::log()->debug("Failed to add linear constraint: {}", f);
    }
  }
}

void KinematicTrajectoryOptimization::AddQuadraticCostToProgram(
    const ExpressionWrapper& cost) {
  DRAKE_ASSERT(prog_ != nullopt);
  const std::vector<int> active_control_point_indices{
      position_curve_.basis().ComputeActiveControlPointIndices(
          cost.plan_interval)};
  for (const auto& index : active_control_point_indices) {
    drake::log()->debug("Control point {} is active.", index);
  }
  const std::vector<symbolic::Expression> per_control_point_expressions =
      SubstitutePlaceholderVariables(cost.expression, control_point_variables_,
                                     active_control_point_indices);
  for (const auto& expression : per_control_point_expressions) {
    if (AreVariablesPresentInProgram(expression.GetVariables())) {
      drake::log()->debug("Adding quadratic cost: {}", expression);
      prog_->AddQuadraticCost(expression.Expand());
    } else {
      drake::log()->debug("Failed to add quadratic cost: {}", expression);
    }
  }
}

solvers::SolutionResult KinematicTrajectoryOptimization::Solve() {
  prog_.emplace();
  const int num_control_points = position_curve_.num_control_points();
  control_point_variables_.clear();
  control_point_variables_.reserve(num_control_points);
  drake::log()->debug("Num control points: {}", num_control_points);
  for (int i = 0; i < num_control_points; ++i) {
    control_point_variables_.push_back(prog_->NewContinuousVariables(
        num_positions(), 1, "control_point_" + std::to_string(i)));
  }

  for (const auto& formula_constraint : formula_linear_constraints_) {
    AddLinearConstraintToProgram(*formula_constraint);
  }

  for (const auto& expression_cost : expression_quadratic_costs_) {
    AddQuadraticCostToProgram(*expression_cost);
  }

  solvers::SolutionResult result = prog_->Solve();
  drake::log()->info("Solver used: {}", prog_->GetSolverId().value().name());

  std::vector<MatrixX<double>> new_control_points;
  new_control_points.reserve(num_control_points);
  drake::log()->debug("Num control point variables: {}",
                      control_point_variables_.size());
  for (const auto& control_point_variable : control_point_variables_) {
    drake::log()->debug("control point: {}",
                        prog_->GetSolution(control_point_variable).transpose());
    new_control_points.push_back(prog_->GetSolution(control_point_variable));
  }
  position_curve_ =
      BsplineCurve<double>(position_curve_.basis(), new_control_points);
  return result;
}

}  // namespace planner
}  // namespace manipulation
}  // namespace drake
