#include "drake/manipulation/planner/kinematic_trajectory_optimization.h"

#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/systems/trajectory_optimization/direct_collocation.h"

using Eigen::MatrixXd;
using drake::systems::System;
using drake::systems::LinearSystem;
using drake::systems::trajectory_optimization::DirectCollocation;
using drake::systems::trajectory_optimization::MultipleShooting;
using drake::solvers::SolutionResult;
using drake::solvers::VectorXDecisionVariable;
using drake::solvers::Cost;
using drake::solvers::Constraint;
using drake::symbolic::Expression;
using drake::symbolic::Formula;

namespace drake {
namespace manipulation {
namespace planner {
namespace {
class SpatialVelocityConstraint : public Constraint {
 public:
  SpatialVelocityConstraint(const RigidBodyTree<double>& tree,
                            const RigidBody<double>& body)
      : Constraint(6, tree.get_num_positions() + tree.get_num_velocities() + 6,
                   Vector6<double>::Zero(), Vector6<double>::Zero()),
        tree_(tree),
        body_(body) {}

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
    const AutoDiffVecXd q = x.head(tree_.get_num_positions());
    const AutoDiffVecXd v =
        x.segment(tree_.get_num_positions(), tree_.get_num_velocities());
    const AutoDiffVecXd spatial_velocity =
        x.segment(tree_.get_num_positions() + tree_.get_num_velocities(), 6);

    KinematicsCache<AutoDiffXd> cache = tree_.doKinematics(q, v);
    y = spatial_velocity -
        tree_.CalcBodySpatialVelocityInWorldFrame(cache, body_);
    //Isometry3<AutoDiffXd> X_FW{tree_.CalcBodyPoseInWorldFrame(cache, body_).inverse()};
    //const AutoDiffVecXd wv_WF{tree_.CalcBodySpatialVelocityInWorldFrame(cache, body_)};
    //const Vector3<AutoDiffXd> w_WF{wv_WF.head(3)};
    //const Vector3<AutoDiffXd> v_WF{wv_WF.tail(3)};
    //y.head(3) = spatial_velocity.head(3) - X_FW*w_WF;
    //y.tail(3) = spatial_velocity.tail(3) - X_FW*v_WF;
  }

  int numOutputs() const { return 6; };

 private:
  const RigidBodyTree<double>& tree_;
  const RigidBody<double>& body_;
};

class BodyPoseConstraint : public Constraint {
 public:
  BodyPoseConstraint(const RigidBodyTree<double>& tree,
                     const RigidBody<double>& body,
                     const Isometry3<double>& X_WFd,
                     double orientation_tolerance, double position_tolerance,
                     Isometry3<double> X_BF = Isometry3<double>::Identity())
      : Constraint(2, tree.get_num_positions(),
                   Vector2<double>(cos(orientation_tolerance), 0.0),
                   Vector2<double>(1.0, std::pow(position_tolerance, 2.0))),
        tree_(tree),
        body_(body),
        X_WFd_(X_WFd),
        X_BF_(X_BF) {}

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
    const AutoDiffVecXd q = x.head(tree_.get_num_positions());

    KinematicsCache<AutoDiffXd> cache = tree_.doKinematics(q);
    const Isometry3<AutoDiffXd> X_WF{tree_.CalcFramePoseInWorldFrame(cache, body_, X_BF_.cast<AutoDiffXd>())};
    Isometry3<AutoDiffXd> X_FdF = X_WFd_.inverse().cast<AutoDiffXd>()*X_WF;

    AutoDiffXd prod{Quaternion<AutoDiffXd>(X_FdF.linear())
                        .coeffs()
                        .dot(Quaternion<AutoDiffXd>::Identity().coeffs())};
    y(0) = 2.0 * prod * prod - 1;
    y(1) = X_FdF.translation().squaredNorm();
  }

  int numOutputs() const { return 2; };

 private:
  const RigidBodyTree<double>& tree_;
  const RigidBody<double>& body_;
  const Isometry3<double> X_WFd_;
  const Isometry3<double> X_BF_;
};

class CollisionAvoidanceConstraint : public Constraint {
 public:
  CollisionAvoidanceConstraint(const RigidBodyTree<double>& tree,
                               double collision_avoidance_threshold)
      : Constraint(1, tree.get_num_positions(), Vector1<double>(0),
                   Vector1<double>(exp(-1))),
        tree_(tree),
        collision_avoidance_threshold_(collision_avoidance_threshold) {}

  virtual void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
                      Eigen::VectorXd& y) const {
    AutoDiffVecXd y_t(1);
    Eval(math::initializeAutoDiff(x), y_t);
    y = math::autoDiffToValueMatrix(y_t);
  }

  virtual void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
                      AutoDiffVecXd& y) const {
    const AutoDiffVecXd q = x.head(tree_.get_num_positions());
    const VectorX<double> q_value = math::autoDiffToValueMatrix(q);

    KinematicsCache<AutoDiffXd> autodiff_cache = tree_.doKinematics(q);
    KinematicsCache<double> cache = tree_.doKinematics(q_value);
    VectorX<double> distance_value;
    Matrix3X<double> xA, xB, normal;
    std::vector<int> idxA_tmp;
    std::vector<int> idxB_tmp;
    const_cast<RigidBodyTree<double>&>(tree_).collisionDetect(cache, distance_value, normal, xA, xB, idxA_tmp, idxB_tmp);
    const int kNumPairs = distance_value.size();
    y(0) = 0;
    y(0).derivatives().resize(q.size());
    y(0).derivatives().setZero();
    if (kNumPairs > 0) {
      VectorX<int> idxA(kNumPairs);
      VectorX<int> idxB(kNumPairs);
      for (int i = 0; i < kNumPairs; ++i) {
        idxA(i) = idxA_tmp[i];
        idxB(i) = idxB_tmp[i];
      }
      MatrixX<double> J;
      tree_.computeContactJacobians(cache, idxA, idxB, xA, xB, J);
      MatrixX<double> ddist_dq{kNumPairs, q.size()};
      for (int i = 0; i < kNumPairs; ++i) {
        ddist_dq.row(i) = normal.col(i).transpose() * J.middleRows(3*i, 3);
      }
      AutoDiffVecXd distance{kNumPairs};
      math::initializeAutoDiffGivenGradientMatrix(distance_value, ddist_dq,
                                                  distance);
      for (int i = 0; i < kNumPairs; ++i) {
        if (distance(i) < 2*collision_avoidance_threshold_) {
          distance(i) /= collision_avoidance_threshold_;
          distance(i) -= 2;
          y(0) += -distance(i) * exp(1 / distance(i));
        }
      }
    }
  }

  int numOutputs() const { return 1; };

 private:
  const RigidBodyTree<double>& tree_;
  double collision_avoidance_threshold_;
};

solvers::VectorXDecisionVariable MakeNamedVariables(const std::string& prefix,
                                                    int num) {
  solvers::VectorXDecisionVariable vars(num);
  for (int i = 0; i < num; i++)
    vars(i) = symbolic::Variable(prefix + std::to_string(i));
  return vars;
}
}  // namespace

bool KinematicTrajectoryOptimization::IsPositionTrajectoryCollisionFree(
    double threshold) const {
  const int kNumTimesToCheck{10 * num_time_samples_};
  const double kStartTime{position_trajectory_.get_start_time()};
  const double kEndTime{position_trajectory_.get_end_time()};
  const VectorX<double> kTimesToCheck{
      VectorX<double>::LinSpaced(kNumTimesToCheck, kStartTime, kEndTime)};
  const CollisionAvoidanceConstraint kConstraint{*tree_, threshold};

  for (int i = 0; i < kNumTimesToCheck; ++i) {
    if (!kConstraint.CheckSatisfied(
            position_trajectory_.value(kTimesToCheck(i)))) {
      return false;
    }
  }
  return true;
}

KinematicTrajectoryOptimization::KinematicTrajectoryOptimization(
    std::unique_ptr<RigidBodyTree<double>> tree, int num_time_samples,
    double minimum_timestep, double maximum_timestep)
    : tree_(std::move(tree)),
      num_time_samples_(num_time_samples),
      placeholder_t_var_(solvers::VectorDecisionVariable<1>(symbolic::Variable("t"))),
      placeholder_q_vars_(MakeNamedVariables("q", tree_->get_num_positions())),
      placeholder_v_vars_(MakeNamedVariables("v", tree_->get_num_velocities())),
      placeholder_a_vars_(MakeNamedVariables("a", tree_->get_num_velocities())),
      placeholder_j_vars_(MakeNamedVariables("j", tree_->get_num_velocities())),
      minimum_timestep_(minimum_timestep),
      maximum_timestep_(maximum_timestep),
      initial_position_trajectory_(PiecewisePolynomial<double>::ZeroOrderHold(
          {0, 1},
          {tree_->getZeroConfiguration(), tree_->getZeroConfiguration()})),
      position_trajectory_(
          PiecewisePolynomialTrajectory(initial_position_trajectory_)){
      };

void KinematicTrajectoryOptimization::AddFinalCost(const symbolic::Expression& g) {
  final_cost_expressions_.emplace_back(new symbolic::Expression(g));
}

void KinematicTrajectoryOptimization::AddEqualTimeIntervalsConstraints() {
  has_equal_time_intervals_ = true;
};

void KinematicTrajectoryOptimization::AddDurationBounds(double lower_bound,
                                                        double upper_bound) {
  duration_lower_bound_ = lower_bound;
  duration_upper_bound_ = upper_bound;
};

void KinematicTrajectoryOptimization::TrackSpatialVelocityOfBody(
    const std::string& body_name) {
  VectorXDecisionVariable vars{num_positions() + num_velocities() + 6};
  vars.head(num_positions()) = position();
  vars.segment(num_positions(), num_velocities()) = velocity();
  vars.tail(6) =
      placeholder_spatial_velocity_vars_
          .emplace(body_name,
                   MakeNamedVariables(body_name + "_spatial_velocity", 6))
          .first->second;
  const RigidBody<double>* body = tree_->FindBody(body_name);
  object_constraints_.emplace_back(new ConstraintWrapper({std::make_shared<SpatialVelocityConstraint>(
      *tree_, *body), vars, {0,1}}));
}

void KinematicTrajectoryOptimization::AddBodyPoseConstraint(
    double time, const std::string& body_name, const Isometry3<double>& X_WFd,
    double orientation_tolerance, double position_tolerance,
    const Isometry3<double>& X_BF) {
  AddBodyPoseConstraint({time, time}, body_name, X_WFd, orientation_tolerance,
                        position_tolerance, X_BF);
}

bool KinematicTrajectoryOptimization::IsValidPlanInterval(
    const Vector2<double>& plan_interval) {
  return 0 <= plan_interval(0) && plan_interval(0) <= 1 &&
         0 <= plan_interval(1) && plan_interval(1) <= 1 &&
         plan_interval(0) <= plan_interval(1);
}

void KinematicTrajectoryOptimization::AddBodyPoseConstraint(
    Vector2<double> plan_interval, const std::string& body_name, const Isometry3<double>& X_WFd,
    double orientation_tolerance, double position_tolerance,
    const Isometry3<double>& X_BF) {
  DRAKE_THROW_UNLESS(IsValidPlanInterval(plan_interval));
  const RigidBody<double>* body = tree_->FindBody(body_name);
  VectorXDecisionVariable vars{num_positions()};
  vars.head(num_positions()) = position();
  object_constraints_.emplace_back(new ConstraintWrapper({std::make_shared<BodyPoseConstraint>(
      *tree_, *body, X_WFd, orientation_tolerance, position_tolerance, X_BF), vars, plan_interval}));
}

void KinematicTrajectoryOptimization::AddCollisionAvoidanceConstraint(
    Vector2<double> plan_interval, double collision_avoidance_threshold) {
  DRAKE_THROW_UNLESS(IsValidPlanInterval(plan_interval));
  auto constraint = std::make_shared<CollisionAvoidanceConstraint>(
      *tree_, collision_avoidance_threshold);
  VectorXDecisionVariable vars{num_positions()};
  vars.head(num_positions()) = position();
  object_constraints_.emplace_back(new ConstraintWrapper({std::make_shared<CollisionAvoidanceConstraint>(
      *tree_, collision_avoidance_threshold), vars, plan_interval}));
}

void KinematicTrajectoryOptimization::AddCollisionAvoidanceConstraint(
    double collision_avoidance_threshold) {
  AddCollisionAvoidanceConstraint({0, 1}, collision_avoidance_threshold);
}

void KinematicTrajectoryOptimization::AddRunningCost(
    const symbolic::Expression& g) {
  running_cost_expressions_.emplace_back(new symbolic::Expression(g));
};

std::unique_ptr<systems::System<double>> KinematicTrajectoryOptimization::CreateSystem() const {
  MatrixX<double> A, B, C, D;
  switch (system_order_) {
    case 1: {
      // State is q. Input is v.
      const int kNumStates{num_positions()};
      const int kNumSpatialVelocityInputs(
          6 * placeholder_spatial_velocity_vars_.size());
      const int kNumInputs(num_velocities() + kNumSpatialVelocityInputs);
      const int kNumOutputs{0};

      const MatrixXd kZero{MatrixXd::Zero(num_positions(), num_positions())};
      const MatrixXd kIdentity{
          MatrixXd::Identity(num_positions(), num_positions())};

      A.resize(kNumStates, kNumStates);
      A << kZero;

      B.resize(kNumStates, kNumInputs);
      B.setZero();
      B.block(0, 0, num_velocities(), num_velocities()) = kIdentity;

      C.resize(kNumOutputs, kNumStates);
      D.resize(kNumOutputs, kNumInputs);
    } break;
    case 2: {
      // State is (q, v). Input is a.
      const int kNumStates{2 * num_positions()};
      const int kNumSpatialVelocityInputs(
          6 * placeholder_spatial_velocity_vars_.size());
      const int kNumInputs(num_velocities() + kNumSpatialVelocityInputs);
      const int kNumOutputs{0};

      const MatrixXd kZero{MatrixXd::Zero(num_positions(), num_positions())};
      const MatrixXd kIdentity{
          MatrixXd::Identity(num_positions(), num_positions())};

      A.resize(kNumStates, kNumStates);
      A << kZero, kIdentity, kZero, kZero;

      B.resize(kNumStates, kNumInputs);
      B.setZero();
      B.block(num_positions(), 0, num_velocities(), num_velocities()) =
          kIdentity;

      C.resize(kNumOutputs, kNumStates);
      D.resize(kNumOutputs, kNumInputs);
    } break;
    case 3: {
      const int kNumStates{3 * num_positions()};
      const int kNumSpatialVelocityInputs(
          6 * placeholder_spatial_velocity_vars_.size());
      const int kNumInputs(num_positions() + kNumSpatialVelocityInputs);
      const int kNumOutputs{0};

      const MatrixXd kZero{MatrixXd::Zero(num_positions(), num_positions())};
      const MatrixXd kIdentity{
          MatrixXd::Identity(num_positions(), num_positions())};

      A.resize(kNumStates, kNumStates);
      A << kZero, kIdentity, kZero, kZero, kZero, kIdentity, kZero, kZero,
          kZero;

      B.resize(kNumStates, kNumInputs);
      B.setZero();
      B.block(num_positions() + num_velocities(), 0, num_velocities(),
              num_velocities()) = kIdentity;

      C.resize(kNumOutputs, kNumStates);
      D.resize(kNumOutputs, kNumInputs);
    } break;
    default: { DRAKE_THROW_UNLESS(false); } break;
  }

  return std::make_unique<LinearSystem<double>>(A, B, C, D);
}

std::unique_ptr<MultipleShooting>
KinematicTrajectoryOptimization::CreateMathematicalProgram() const {
  return std::make_unique<DirectCollocation>(
      system_.get(), *(system_->CreateDefaultContext()), num_time_samples_,
      minimum_timestep_, maximum_timestep_);
}

const solvers::VectorXDecisionVariable
KinematicTrajectoryOptimization::GetStateVariablesFromProgram(
    const MultipleShooting& prog, int index) const {
  if (index < 0) {
    return prog.state();
  } else {
    return prog.state(index);
  }
}

const solvers::VectorXDecisionVariable
KinematicTrajectoryOptimization::GetInputVariablesFromProgram(
    const MultipleShooting& prog, int index) const {
  if (index < 0) {
    return prog.input();
  } else {
    return prog.input(index);
  }
}

const solvers::VectorXDecisionVariable
KinematicTrajectoryOptimization::GetPositionVariablesFromProgram(
    const MultipleShooting& prog, int index) const {
  // Position variables are always at the head of the state.
  return GetStateVariablesFromProgram(prog, index).head(num_positions());
}

const solvers::VectorXDecisionVariable
KinematicTrajectoryOptimization::GetVelocityVariablesFromProgram(
    const MultipleShooting& prog, int index) const {
  // For first-order systems, velocity variables are the first inputs. For all
  // others, they follow the position variables in the state.
  if (system_order_ < 2) {
    return GetInputVariablesFromProgram(prog, index).head(num_velocities());
  } else {
    return GetStateVariablesFromProgram(prog, index).segment(num_positions(),
        num_velocities());
  }
}

const solvers::VectorXDecisionVariable
KinematicTrajectoryOptimization::GetAccelerationVariablesFromProgram(
    const MultipleShooting& prog, int index) const {
  // For first-order systems, acceleration variables are ignored. For
  // second-order systems, they are the first inputs. For all others, they
  // follow the velocity variables in the state.
  if (system_order_ < 2) {
    return placeholder_a_vars_;
  } else if (system_order_ < 3) {
    return GetInputVariablesFromProgram(prog, index).head(num_velocities());
  } else {
    return GetStateVariablesFromProgram(prog, index).segment(
        num_positions() + num_velocities(), num_velocities());
  }
}

const solvers::VectorXDecisionVariable
KinematicTrajectoryOptimization::GetJerkVariablesFromProgram(
    const MultipleShooting& prog, int index) const {
  // For less-than-third-order systems, jerk variables are ignored. For
  // third-order systems, they are the first inputs.
  if (system_order_ < 3) {
    return placeholder_j_vars_;
  } else {
    return GetInputVariablesFromProgram(prog, index).head(num_velocities());
  }
}

const solvers::VectorXDecisionVariable
KinematicTrajectoryOptimization::GetBodySpatialVelocityVariablesFromProgram(
    const MultipleShooting& prog, int index) const {
  // For {first, second, third}-order systems, the body spatial velocity
  // variables come after the {velocity, acceleration, jerk} variables in the
  // input.
  return GetInputVariablesFromProgram(prog, index).segment(
      num_velocities(), num_body_spatial_velocity_variables());
}

symbolic::Substitution
KinematicTrajectoryOptimization::ConstructPlaceholderVariableSubstitution(
    const MultipleShooting& prog, int index) const {
  symbolic::Substitution sub;
  if (index < 0) {
    sub.emplace(placeholder_t_var_(0), prog.time()(0));
  }
  // Get the actual decision variables from the mathematical program.
  VectorXDecisionVariable program_q_vars{
      GetPositionVariablesFromProgram(prog, index)};
  VectorXDecisionVariable program_v_vars{
      GetVelocityVariablesFromProgram(prog, index)};
  VectorXDecisionVariable program_a_vars{
      GetAccelerationVariablesFromProgram(prog, index)};
  VectorXDecisionVariable program_j_vars{
      GetJerkVariablesFromProgram(prog, index)};
  VectorXDecisionVariable program_body_spatial_velocity_vars{
      GetBodySpatialVelocityVariablesFromProgram(prog, index)};
  for (int i = 0; i < num_positions(); ++i) {
    sub.emplace(placeholder_q_vars_(i), program_q_vars(i));
  }
  for (int i = 0; i < num_positions(); ++i) {
    sub.emplace(placeholder_v_vars_(i), program_v_vars(i));
  }
  for (int i = 0; i < num_positions(); ++i) {
    sub.emplace(placeholder_a_vars_(i), program_a_vars(i));
  }
  for (int i = 0; i < num_positions(); ++i) {
    sub.emplace(placeholder_j_vars_(i), program_j_vars(i));
  }
  int input_index = 0;
  for (auto& placeholder_vars : placeholder_spatial_velocity_vars_) {
    for (int i = 0; i < 6; ++i) {
      sub.emplace(placeholder_vars.second(i),
                  program_body_spatial_velocity_vars(input_index));
      ++input_index;
    }
  }
  return sub;
}

solvers::VectorXDecisionVariable KinematicTrajectoryOptimization::SubstitutePlaceholderVariables(
    const solvers::VectorXDecisionVariable& vars,
    const systems::trajectory_optimization::MultipleShooting& prog, int index) {
  VectorXDecisionVariable vars_out{vars.size()};
  for (int i = 0; i < vars.size(); ++i) {
    vars_out(i) =
        *vars.cast<symbolic::Expression>()(i)
             .Substitute(ConstructPlaceholderVariableSubstitution(prog, index))
             .GetVariables()
             .begin();
  }
  return vars_out;
}

Formula KinematicTrajectoryOptimization::SubstitutePlaceholderVariables(
    const Formula& f,
    const systems::trajectory_optimization::MultipleShooting& prog, int index) {
  return f.Substitute(ConstructPlaceholderVariableSubstitution(prog, index));
}

Expression KinematicTrajectoryOptimization::SubstitutePlaceholderVariables(
    const Expression& g,
    const systems::trajectory_optimization::MultipleShooting& prog, int index) {
  return g.Substitute(ConstructPlaceholderVariableSubstitution(prog, index));
}

std::vector<int> KinematicTrajectoryOptimization::ActiveKnotsForPlanInterval(
    const systems::trajectory_optimization::MultipleShooting& prog,
    const Vector2<double>& plan_interval) {
  std::vector<int> active_knots;
  if (plan_interval(1) - plan_interval(0) < 1.0 / num_time_samples_) {
    // It's possible that no knots will fall inside the interval. We don't want
    // to loose the constraint, so apply it at the knot closest to the mid-point
    // of the interval.
    const double interval_center{plan_interval.mean()};
    active_knots.push_back(
        std::round(interval_center * (num_time_samples_ - 1)));
  } else {
    for (int i = 0; i < num_time_samples_; ++i) {
      const double knot_time{static_cast<double>(i) /
                             (static_cast<double>(num_time_samples_) - 1)};
      if (plan_interval(0) <= knot_time && knot_time <= plan_interval(1)) {
        active_knots.push_back(i);
      }
    }
  }
  return active_knots;
}

bool KinematicTrajectoryOptimization::AreVariablesPresentInProgram(
    symbolic::Variables vars) const {
  if (system_order_ < 3) {
    if (!symbolic::intersect(symbolic::Variables(placeholder_j_vars_), vars)
             .empty()) {
      return false;
    }
    if (system_order_ < 2) {
      if (!symbolic::intersect(symbolic::Variables(placeholder_a_vars_), vars)
               .empty()) {
        return false;
      }
    }
  }
  return true;
}

void KinematicTrajectoryOptimization::AddConstraintToProgram(
    const ConstraintWrapper& constraint, MultipleShooting* prog) {
  // Ignore constraints on higher order terms not present in the model.
  if (!AreVariablesPresentInProgram(symbolic::Variables(constraint.vars))) return;
  std::vector<int> active_knots{
      ActiveKnotsForPlanInterval(*prog, constraint.plan_interval)};
  for (int index : active_knots) {
    prog->AddConstraint(
        constraint.constraint,
        SubstitutePlaceholderVariables(constraint.vars, *prog, index));
  }
}

void KinematicTrajectoryOptimization::AddLinearConstraintToProgram(
    const FormulaWrapper& constraint, MultipleShooting* prog) {
  if (!AreVariablesPresentInProgram(constraint.formula.GetFreeVariables())) return;
  std::vector<int> active_knots{
      ActiveKnotsForPlanInterval(*prog, constraint.plan_interval)};
  for (int index : active_knots) {
    prog->AddLinearConstraint(SubstitutePlaceholderVariables(constraint.formula, *prog, index));
  }
}

void KinematicTrajectoryOptimization::AddRunningCostToProgram(
    const Expression& cost, MultipleShooting* prog) {
  if (!AreVariablesPresentInProgram(cost.GetVariables())) return;
  drake::log()->debug("Adding cost: {}", SubstitutePlaceholderVariables(cost, *prog));
  prog->AddRunningCost(SubstitutePlaceholderVariables(cost, *prog));
}

void KinematicTrajectoryOptimization::AddFinalCostToProgram(
    const Expression& cost, MultipleShooting* prog) {
  if (!AreVariablesPresentInProgram(cost.GetVariables())) return;
  drake::log()->debug("Adding cost: {}", SubstitutePlaceholderVariables(cost, *prog));
  prog->AddFinalCost(SubstitutePlaceholderVariables(cost, *prog));
}


void KinematicTrajectoryOptimization::AddCostToProgram(const CostWrapper& cost,
                                                       MultipleShooting* prog) {
  if (!AreVariablesPresentInProgram(symbolic::Variables(cost.vars))) return;
  std::vector<int> active_knots{
      ActiveKnotsForPlanInterval(*prog, cost.plan_interval)};
  for (int index : active_knots) {
    prog->AddCost(cost.cost,
                  SubstitutePlaceholderVariables(cost.vars, *prog, index));
  }
}

void KinematicTrajectoryOptimization::SetInitialTrajectoryOnProgram(
    MultipleShooting* prog) {
  // Set initial guess
  const std::vector<double> t_values{
      initial_position_trajectory_.getSegmentTimes()};
  const int kNumTimeSamplesInit(t_values.size());
  const VectorX<Expression> x_expression{prog->state()};
  const VectorX<Expression> u_expression{prog->input()};
  std::vector<MatrixXd> u_values(kNumTimeSamplesInit);
  std::vector<MatrixXd> x_values(kNumTimeSamplesInit);
  std::vector<MatrixXd> xdot_values(kNumTimeSamplesInit);
  drake::log()->debug("x_values.size() = {}", x_values.size());
  drake::log()->debug("t_values.size() = {}", t_values.size());
  for (int i = 0; i < kNumTimeSamplesInit; ++i) {
    symbolic::Environment x_environment;
    symbolic::Environment xdot_environment;
    const VectorX<double> position{
        initial_position_trajectory_.value(t_values[i])};
    const VectorX<double> velocity{
        initial_position_trajectory_.derivative(1).value(t_values[i])};
    const VectorX<double> acceleration{
        initial_position_trajectory_.derivative(2).value(t_values[i])};
    const VectorX<double> jerk{
        initial_position_trajectory_.derivative(3).value(t_values[i])};
    // TODO(avalenzu) Actually compute the spatial velocities and accelerations.
    const VectorX<double> body_spatial_velocity {
      VectorX<double>::Zero(num_body_spatial_velocity_variables())};
    const VectorX<double> body_spatial_acceleration {
      VectorX<double>::Zero(num_body_spatial_velocity_variables())};
    // TODO(avalenzu) Remove assumption that num_positions() == num_velocities()
    for (int j = 0; j < num_positions(); ++j) {
      x_environment.insert(GetPositionVariablesFromProgram(*prog)(j),
                         position(j));
      xdot_environment.insert(GetPositionVariablesFromProgram(*prog)(j),
                         velocity(j));
    }
    for (int j = 0; j < num_velocities(); ++j) {
      x_environment.insert(GetVelocityVariablesFromProgram(*prog)(j),
                         velocity(j));
      x_environment.insert(GetAccelerationVariablesFromProgram(*prog)(j),
                         acceleration(j));
      x_environment.insert(GetJerkVariablesFromProgram(*prog)(j), jerk(j));

      xdot_environment.insert(GetVelocityVariablesFromProgram(*prog)(j),
                         acceleration(j));
      xdot_environment.insert(GetAccelerationVariablesFromProgram(*prog)(j),
                         jerk(j));
    }
    for (int j = 0; j < num_body_spatial_velocity_variables(); ++j) {
      x_environment.insert(GetBodySpatialVelocityVariablesFromProgram(*prog)(j),
                         body_spatial_velocity(i));
      xdot_environment.insert(GetBodySpatialVelocityVariablesFromProgram(*prog)(j),
                         body_spatial_acceleration(i));
    }
    u_values[i].resizeLike(u_expression);
    x_values[i].resizeLike(x_expression);
    xdot_values[i].resizeLike(x_expression);
    for (int j = 0; j < static_cast<int>(u_expression.size()); ++j) {
      u_values[i](j) = u_expression(j).Evaluate(x_environment);
    }
    for (int j = 0; j < static_cast<int>(x_expression.size()); ++j) {
      x_values[i](j) = x_expression(j).Evaluate(x_environment);
      xdot_values[i](j) = x_expression(j).Evaluate(xdot_environment);
    }
  }
  PiecewisePolynomial<double> traj_init_u =
      PiecewisePolynomial<double>::FirstOrderHold(t_values, u_values);
  PiecewisePolynomial<double> traj_init_x =
      PiecewisePolynomial<double>::Cubic(t_values, x_values, xdot_values);
  prog->SetInitialTrajectory(traj_init_u, traj_init_x);
}

SolutionResult KinematicTrajectoryOptimization::Solve() {
  system_ = CreateSystem();
  std::unique_ptr<MultipleShooting> prog{CreateMathematicalProgram()};
  auto position_variables{GetPositionVariablesFromProgram(*prog)};
  prog->AddConstraintToAllKnotPoints(position_variables >=
                                     tree_->joint_limit_min);
  prog->AddConstraintToAllKnotPoints(position_variables <=
                                     tree_->joint_limit_max);
  if (has_equal_time_intervals_) {
    prog->AddEqualTimeIntervalsConstraints();
  }
  prog->AddDurationBounds(duration_lower_bound_, duration_upper_bound_);

  for (const auto& constraint : object_constraints_) {
    AddConstraintToProgram(*constraint, prog.get());
  }
  for (const auto& constraint : formula_linear_constraints_) {
    AddLinearConstraintToProgram(*constraint, prog.get());
  }
  for (const auto& cost : running_cost_objects_) {
    AddCostToProgram(*cost, prog.get());
  }
  for (const auto& cost : running_cost_expressions_) {
    AddRunningCostToProgram(*cost, prog.get());
  }
  for (const auto& cost : final_cost_expressions_) {
    AddFinalCostToProgram(*cost, prog.get());
  }

  SetInitialTrajectoryOnProgram(prog.get());

  drake::log()->debug("Solving MathematicalProgram with {} decision variables.",
                      prog->decision_variables().size());
  SolutionResult result{prog->Solve()};

  UpdatePositionTrajectory(*prog);
  
  return result;
}

void KinematicTrajectoryOptimization::UpdatePositionTrajectory(
    const systems::trajectory_optimization::MultipleShooting& prog) {
  VectorX<double> times{prog.GetSampleTimes()};
  std::vector<double> times_vec(num_time_samples_);
  std::vector<MatrixX<double>> positions(num_time_samples_);
  std::vector<MatrixX<double>> velocities(num_time_samples_);
  for (int i = 0; i < num_time_samples_; ++i) {
    times_vec[i] = times(i);
    positions[i] = prog.GetSolution(GetPositionVariablesFromProgram(prog, i));
    velocities[i] =
        prog.GetSolution(GetVelocityVariablesFromProgram(prog, i));
  }
  position_trajectory_ = PiecewisePolynomialTrajectory(
      PiecewisePolynomial<double>::Cubic(times_vec, positions, velocities));
}

PiecewisePolynomialTrajectory
KinematicTrajectoryOptimization::GetPositionTrajectory() const {
  return position_trajectory_;
};

template <typename T>
void KinematicTrajectoryOptimization::SetSolverOption(
    const solvers::SolverId& solver_id, const std::string& solver_option,
    T option_value) {
  solver_options_container_.SetSolverOption(solver_id, solver_option,
                                            option_value);
}

void KinematicTrajectoryOptimization::AddLinearConstraint(
    const symbolic::Formula& f) {
  AddLinearConstraint(f, {0, 1});
}

void KinematicTrajectoryOptimization::AddLinearConstraint(
    const symbolic::Formula& f, double time) {
  AddLinearConstraint(f, {time, time});
}

void KinematicTrajectoryOptimization::AddLinearConstraint(
    const symbolic::Formula& f, Vector2<double> plan_interval) {
  formula_linear_constraints_.emplace_back(new FormulaWrapper({f, plan_interval}));
}

// Explicit instantiations of SetSolverOption()
template void KinematicTrajectoryOptimization::SetSolverOption(
    const solvers::SolverId& solver_id, const std::string& solver_option,
    double option_value);

template void KinematicTrajectoryOptimization::SetSolverOption(
    const solvers::SolverId& solver_id, const std::string& solver_option,
    int option_value);

template void KinematicTrajectoryOptimization::SetSolverOption(
    const solvers::SolverId& solver_id, const std::string& solver_option,
    const std::string& option_value);

}  // namespace planner
}  // namespace manipulation
}  // namespace drake
