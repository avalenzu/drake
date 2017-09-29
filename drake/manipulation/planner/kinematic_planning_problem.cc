#include "drake/manipulation/planner/kinematic_planning_problem.h"
#include "drake/math/autodiff.h"

namespace drake {
namespace manipulation {
namespace planner {

using solvers::Binding;
using solvers::Constraint;
using solvers::Cost;
using solvers::VectorDecisionVariable;
using solvers::VectorXDecisionVariable;

namespace {
solvers::VectorXDecisionVariable MakeNamedVariables(const std::string& prefix,
                                                    int num) {
  solvers::VectorXDecisionVariable vars(num);
  for (int i = 0; i < num; i++)
    vars(i) = symbolic::Variable(prefix + std::to_string(i));
  return vars;
}
}  // namespace

KinematicPlanningProblem::KinematicPlanningProblem(
    const RigidBodyTree<double>& tree)
    : num_positions_(tree.get_num_positions()),
      num_velocities_(tree.get_num_velocities()),
      placeholder_time_var_(
          solvers::VectorDecisionVariable<1>(symbolic::Variable("t"))),
      placeholder_position_vars_(
          MakeNamedVariables("q", tree.get_num_positions())),
      placeholder_velocity_vars_(
          MakeNamedVariables("v", tree.get_num_velocities())),
      placeholder_acceleration_vars_(
          MakeNamedVariables("a", tree.get_num_velocities())),
      placeholder_jerk_vars_(
          MakeNamedVariables("j", tree.get_num_velocities()))
{}

void KinematicPlanningProblem::AddFixedBoxToWorld(Vector3<double> size,
                                                  Isometry3<double> X_WB) {
  world_geometry_.emplace_back(std::make_unique<DrakeShapes::Box>(size), X_WB);
}

void KinematicPlanningProblem::AddDurationBounds(double lower_bound,
                                                 double upper_bound) {
  duration_lower_bound_ = lower_bound;
  duration_upper_bound_ = upper_bound;
}

void KinematicPlanningProblem::AddCost(const Binding<Cost>& binding,
                                       double plan_time) {
  costs_.emplace_back(new CostWrapper(
      {binding.constraint(), binding.variables(), {plan_time, plan_time}}));
}

void KinematicPlanningProblem::AddRunningCost(
    const Binding<Cost>& binding, const Vector2<double>& plan_interval) {
  costs_.emplace_back(new CostWrapper(
      {binding.constraint(), binding.variables(), plan_interval}));
}

void KinematicPlanningProblem::AddConstraint(const Binding<Constraint>& binding,
                                             double plan_time) {
  constraints_.emplace_back(new ConstraintWrapper(
      {binding.constraint(), binding.variables(), {plan_time, plan_time}}));
}

void KinematicPlanningProblem::AddConstraint(
    const Binding<Constraint>& binding, const Vector2<double>& plan_interval) {
  constraints_.emplace_back(new ConstraintWrapper(
      {binding.constraint(), binding.variables(), plan_interval}));
}

void KinematicPlanningProblem::AddCollisionAvoidanceConstraint(
    double threshold, const Vector2<double>& plan_interval) {
  collision_constraints_.emplace_back(
      new CollisionAvoidanceWrapper({threshold, position(), plan_interval}));
}

const VectorDecisionVariable<1>& KinematicPlanningProblem::time() const {
  return placeholder_time_var_;
}

const VectorXDecisionVariable& KinematicPlanningProblem::position() const {
  return placeholder_position_vars_;
}

const VectorXDecisionVariable& KinematicPlanningProblem::velocity() const {
  return placeholder_velocity_vars_;
}

const VectorXDecisionVariable& KinematicPlanningProblem::acceleration() const {
  return placeholder_acceleration_vars_;
}

const VectorXDecisionVariable& KinematicPlanningProblem::jerk() const {
  return placeholder_jerk_vars_;
}

}  // namespace planner
}  // namespace manipulation
}  // namespace drake
