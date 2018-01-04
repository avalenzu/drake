#include "drake/manipulation/planner/kinematic_trajectory_optimization.h"

#include "drake/common/text_logging.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
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

class PointConstraint : public solvers::Constraint {
 public:
  PointConstraint(std::shared_ptr<solvers::Constraint> wrapped_constraint,
                  const std::vector<double>& basis_function_values)
      : Constraint(
            wrapped_constraint->num_outputs(),
            basis_function_values.size() * wrapped_constraint->num_vars(),
            wrapped_constraint->lower_bound(),
            wrapped_constraint->upper_bound()),
        wrapped_constraint_(wrapped_constraint),
        basis_function_values_(basis_function_values) {}

  virtual void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
                      Eigen::VectorXd& y) const {
    AutoDiffVecXd y_t;
    Eval(math::initializeAutoDiff(x), y_t);
    y = math::autoDiffToValueMatrix(y_t);
  }

  virtual void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
                      AutoDiffVecXd& y) const {
    AutoDiffVecXd x_sum = basis_function_values_[0] *
                          x.segment(0, wrapped_constraint_->num_vars());
    const int num_terms = basis_function_values_.size();
    for (int i = 1; i < num_terms; ++i) {
      x_sum += basis_function_values_[i] *
               x.segment(i * wrapped_constraint_->num_vars(),
                         wrapped_constraint_->num_vars());
    }
    wrapped_constraint_->Eval(x_sum, y);
  }

  int numOutputs() const { return wrapped_constraint_->num_outputs(); };

 private:
  std::shared_ptr<solvers::Constraint> wrapped_constraint_;
  std::vector<double> basis_function_values_;
};

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

void KinematicTrajectoryOptimization::AddGenericPositionConstraint(
    const std::shared_ptr<solvers::Constraint>& constraint,
    const std::array<double, 2>& plan_interval) {
  generic_position_constraints_.emplace_back(
      new ConstraintWrapper({constraint, plan_interval}));
}

void KinematicTrajectoryOptimization::AddLinearConstraint(
    const symbolic::Formula& f, const std::array<double, 2>& plan_interval) {
  formula_linear_constraints_.emplace_back(
      new FormulaWrapper({f, plan_interval}));
}

void KinematicTrajectoryOptimization::AddQuadraticCost(
    const symbolic::Expression& f, const std::array<double, 2>& plan_interval) {
  expression_quadratic_costs_.emplace_back(
      new ExpressionWrapper({f, plan_interval}));
}

std::vector<symbolic::Substitution>
KinematicTrajectoryOptimization::ConstructPlaceholderVariableSubstitution(
    const std::vector<MatrixXDecisionVariable>& control_points,
    const std::array<double, 2>& plan_interval) const {
  // Create symbolic curves
  BsplineCurve<symbolic::Expression> symbolic_q_curve{position_curve_.basis(),
                                                      control_points};
  BsplineCurve<symbolic::Expression> symbolic_v_curve{
      symbolic_q_curve.Derivative()};
  BsplineCurve<symbolic::Expression> symbolic_a_curve{
      symbolic_v_curve.Derivative()};
  BsplineCurve<symbolic::Expression> symbolic_j_curve{
      symbolic_a_curve.Derivative()};

  std::vector<symbolic::Substitution> sub;
  if (plan_interval.back() - plan_interval.front() <
      PiecewiseFunction::kEpsilonTime) {
    sub.resize(1);
    const VectorX<symbolic::Expression> symbolic_q_value =
        symbolic_q_curve.value(plan_interval.front());
    const VectorX<symbolic::Expression> symbolic_v_value =
        symbolic_v_curve.value(plan_interval.front());
    const VectorX<symbolic::Expression> symbolic_a_value =
        symbolic_a_curve.value(plan_interval.front());
    const VectorX<symbolic::Expression> symbolic_j_value =
        symbolic_j_curve.value(plan_interval.front());
    for (int i = 0; i < num_positions(); ++i) {
      sub[0].emplace(placeholder_q_vars_(i), symbolic_q_value(i));
      sub[0].emplace(placeholder_v_vars_(i), symbolic_v_value(i));
      sub[0].emplace(placeholder_a_vars_(i), symbolic_a_value(i));
      sub[0].emplace(placeholder_j_vars_(i), symbolic_j_value(i));
    }
  } else {
    const std::vector<int> active_control_point_indices{
        position_curve_.basis().ComputeActiveControlPointIndices(
            plan_interval)};
    sub.resize(active_control_point_indices.size());

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
                  .control_points()[active_control_point_indices[j - 1]](i)
                  .Expand());
        }
        if (1 < j) {
          sub[j].emplace(
              placeholder_a_vars_(i),
              symbolic_a_curve
                  .control_points()[active_control_point_indices[j - 2]](i)
                  .Expand());
        }
        if (2 < j) {
          sub[j].emplace(
              placeholder_j_vars_(i),
              symbolic_j_curve
                  .control_points()[active_control_point_indices[j - 3]](i)
                  .Expand());
        }
      }
    }
  }
  return sub;
}

std::vector<symbolic::Formula>
KinematicTrajectoryOptimization::SubstitutePlaceholderVariables(
    const symbolic::Formula& formula,
    const std::vector<MatrixXDecisionVariable>& control_points,
    const std::array<double, 2>& plan_interval) const {
  std::vector<symbolic::Formula> substitution_results;
  std::vector<symbolic::Substitution> substitutions =
      ConstructPlaceholderVariableSubstitution(control_points, plan_interval);
  substitution_results.reserve(substitutions.size());
  for (const auto& substitution : substitutions) {
    substitution_results.push_back(formula.Substitute(substitution));
  }
  return substitution_results;
}

std::vector<symbolic::Expression>
KinematicTrajectoryOptimization::SubstitutePlaceholderVariables(
    const symbolic::Expression& expression,
    const std::vector<MatrixXDecisionVariable>& control_points,
    const std::array<double, 2>& plan_interval) const {
  std::vector<symbolic::Expression> substitution_results;
  std::vector<symbolic::Substitution> substitutions =
      ConstructPlaceholderVariableSubstitution(control_points, plan_interval);
  substitution_results.reserve(substitutions.size());
  for (const auto& substitution : substitutions) {
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
  const std::vector<symbolic::Formula> per_control_point_formulae =
      SubstitutePlaceholderVariables(constraint.formula,
                                     control_point_variables_,
                                     constraint.plan_interval);
  for (const auto& f : per_control_point_formulae) {
    if (AreVariablesPresentInProgram(f.GetFreeVariables())) {
      drake::log()->trace("Adding linear constraint: {}", f);
      prog_->AddLinearConstraint(f);
    } else {
      drake::log()->trace("Failed to add linear constraint: {}", f);
    }
  }
}

void KinematicTrajectoryOptimization::AddQuadraticCostToProgram(
    const ExpressionWrapper& cost) {
  DRAKE_ASSERT(prog_ != nullopt);
  const std::vector<symbolic::Expression> per_control_point_expressions =
      SubstitutePlaceholderVariables(cost.expression, control_point_variables_,
                                     cost.plan_interval);
  for (const auto& expression : per_control_point_expressions) {
    if (AreVariablesPresentInProgram(expression.GetVariables())) {
      drake::log()->trace("Adding quadratic cost: {}", expression);
      prog_->AddQuadraticCost(expression.Expand());
    } else {
      drake::log()->trace("Failed to add quadratic cost: {}", expression);
    }
  }
}

void KinematicTrajectoryOptimization::AddGenericPositionConstraintToProgram(
    const ConstraintWrapper& constraint) {
  DRAKE_ASSERT(prog_ != nullopt);
  const auto evaluation_times = VectorX<double>::LinSpaced(
      constraint.num_evaluation_points, constraint.plan_interval.front(),
      constraint.plan_interval.back());
  for (int evaluation_time_index = 0;
       evaluation_time_index < evaluation_times.size();
       ++evaluation_time_index) {
    double evaluation_time = evaluation_times(evaluation_time_index);
    std::vector<double> basis_function_values;
    basis_function_values.reserve(position_curve_.order());
    solvers::VectorXDecisionVariable var_vector(position_curve_.order() *
                                                num_positions());
    std::vector<int> active_control_point_indices =
        position_curve_.basis().ComputeActiveControlPointIndices(
            {{evaluation_time, evaluation_time}});
    for (int i = 0; i < position_curve_.order(); ++i) {
      const int control_point_index = active_control_point_indices[i];
      basis_function_values.push_back(
          position_curve_.basis().polynomials()[control_point_index].value(
              evaluation_time)(0));
      var_vector.segment(i * num_positions(), num_positions()) =
          control_point_variables_[control_point_index];
    }
    drake::log()->trace("Adding constraint at t = {}", evaluation_time);
    prog_->AddConstraint(std::make_shared<PointConstraint>(
                             constraint.constraint, basis_function_values),
                         var_vector);
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
    prog_->SetInitialGuess(control_point_variables_.back(),
                           position_curve_.control_points()[i]);
  }

  for (const auto& formula_constraint : formula_linear_constraints_) {
    AddLinearConstraintToProgram(*formula_constraint);
  }

  for (const auto& expression_cost : expression_quadratic_costs_) {
    AddQuadraticCostToProgram(*expression_cost);
  }

  for (const auto& generic_position_constraint :
       generic_position_constraints_) {
    AddGenericPositionConstraintToProgram(*generic_position_constraint);
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

bool KinematicTrajectoryOptimization::UpdateGenericConstraints() {
  bool constraints_have_been_modified{false};
  for (auto& constraint : generic_position_constraints_) {
    const VectorX<double> t{VectorX<double>::LinSpaced(
        num_evaluation_points_, constraint->plan_interval.front(),
        constraint->plan_interval.back())};
    for (int i = 0; i < num_evaluation_points_; ++i) {
      if (!constraint->constraint->CheckSatisfied(position_curve_.value(t(i)),
                                                  1e-3)) {
        constraints_have_been_modified = true;
        constraint->num_evaluation_points +=
            constraint->num_evaluation_points - 1;
        break;
      }
    }
  }
  return constraints_have_been_modified;
}

bool KinematicTrajectoryOptimization::AddKnots() {
  auto second_to_last_knot = position_curve_.knots().end() - 1;
  bool knots_have_been_added{false};
  for (auto knot = position_curve_.knots().begin(); knot != second_to_last_knot;
       ++knot) {
    drake::log()->debug("knot[i] = {}", *knot);
  }
  std::vector<double> old_knots = position_curve_.knots();
  const int num_knots = old_knots.size();
  for (int i = 0; i < num_knots - 1; ++i) {
    double new_knot = 0.5 * (old_knots[i] + old_knots[i + 1]);
    drake::log()->debug("knot[i] = {}, knot[i+1] = {}, new_knot = {}",
                        old_knots[i], old_knots[i + 1], new_knot);
    if (new_knot - old_knots[i] > min_knot_resolution()) {
      knots_have_been_added = true;
      drake::log()->debug("Adding knot at t = {}", new_knot);
      position_curve_.InsertKnot(new_knot);
    }
  }
  return knots_have_been_added;
}

}  // namespace planner
}  // namespace manipulation
}  // namespace drake
