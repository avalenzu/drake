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
    //std::vector<drake::multibody::collision::PointPair> pairs =
        //const_cast<RigidBodyTree<double>*>(&tree_)
            //->ComputeMaximumDepthCollisionPoints(cache, true);
    //const int kNumPairs = pairs.size();
    y(0) = 0;
    y(0).derivatives().resize(q.size());
    y(0).derivatives().setZero();
    if (kNumPairs > 0) {
      //drake::log()->debug("Number of collision pairs: {}", kNumPairs);
      VectorX<int> idxA(kNumPairs);
      VectorX<int> idxB(kNumPairs);
      for (int i = 0; i < kNumPairs; ++i) {
        idxA(i) = idxA_tmp[i];
        idxB(i) = idxB_tmp[i];
        //idxA(i) = pairs.at(i).elementA->get_body()->get_body_index();
        //idxB(i) = pairs.at(i).elementB->get_body()->get_body_index();
        //xA.col(i) = pairs.at(i).ptA;
        //xB.col(i) = pairs.at(i).ptB;
        //distance_value(i) = pairs.at(i).distance;
        //drake::log()->debug(
            //"\t{} and {}: {} m", pairs.at(i).elementA->get_body()->get_name(),
            //pairs.at(i).elementB->get_body()->get_name(), distance_value(i));
      }
      MatrixX<double> J;
      tree_.computeContactJacobians(cache, idxA, idxB, xA, xB, J);
      MatrixX<double> ddist_dq{kNumPairs, q.size()};
      for (int i = 0; i < kNumPairs; ++i) {
        //ddist_dq.row(i) = pairs.at(i).normal.transpose() * J.middleRows(3*i, 3);
        ddist_dq.row(i) = normal.col(i).transpose() * J.middleRows(3*i, 3);
      }
      AutoDiffVecXd distance{kNumPairs};
      math::initializeAutoDiffGivenGradientMatrix(distance_value, ddist_dq,
                                                  distance);
      for (int i = 0; i < kNumPairs; ++i) {
        if (distance(i) < 2*collision_avoidance_threshold_) {
          //drake::log()->debug(
              //"\t{} and {}: {} m", tree_.get_body(idxA(i)).get_name(),
              //tree_.get_body(idxB(i)).get_name(), distance_value(i));
          distance(i) /= collision_avoidance_threshold_;
          distance(i) -= 2;
          //y(0) += distance(i) * distance(i);
          y(0) += -distance(i) * exp(1 / distance(i));
        }
      }
      //drake::log()->debug("Constraint value: {}", math::autoDiffToValueMatrix(y));
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
  const int kNumStates{3 * num_positions()};
  const int kNumSpatialVelocityInputs(6*placeholder_spatial_velocity_vars_.size());
  const int kNumInputs(num_positions() + kNumSpatialVelocityInputs);
  const int kNumOutputs{0};

  const MatrixXd kZero{MatrixXd::Zero(num_positions(), num_positions())};
  const MatrixXd kIdentity{
      MatrixXd::Identity(num_positions(), num_positions())};

  MatrixX<double> A{kNumStates, kNumStates};
  A << kZero, kIdentity, kZero, kZero, kZero, kIdentity, kZero, kZero, kZero;

  MatrixXd B{kNumStates, kNumInputs};
  B.setZero();
  B.block(num_positions() + num_velocities(), 0, num_velocities(),
          num_velocities()) = kIdentity;

  MatrixXd C{kNumOutputs, kNumStates};
  MatrixXd D{kNumOutputs, kNumInputs};

  return std::make_unique<LinearSystem<double>>(A, B, C, D);
}

std::unique_ptr<MultipleShooting>
KinematicTrajectoryOptimization::CreateMathematicalProgram() const {
  return std::make_unique<DirectCollocation>(
      system_.get(), *(system_->CreateDefaultContext()), num_time_samples_,
      minimum_timestep_, maximum_timestep_);
}

const solvers::VectorXDecisionVariable
KinematicTrajectoryOptimization::GetPositionVariablesFromProgram(
    const MultipleShooting& prog) const {
  return prog.state().head(num_positions());
}

const solvers::VectorXDecisionVariable
KinematicTrajectoryOptimization::GetPositionVariablesFromProgram(
    const MultipleShooting& prog, int index) const {
  return prog.state(index).head(num_positions());
}

symbolic::Substitution
KinematicTrajectoryOptimization::ConstructPlaceholderVariableSubstitution(
    const MultipleShooting& prog) const {
  symbolic::Substitution sub;
  sub.emplace(placeholder_t_var_(0), prog.time()(0));
  for (int i = 0; i < num_positions(); ++i) {
    sub.emplace(placeholder_q_vars_(i), prog.state()(i));
  }
  for (int i = 0; i < num_velocities(); ++i) {
    sub.emplace(placeholder_v_vars_(i), prog.state()(num_positions() + i));
  }
  for (int i = 0; i < num_velocities(); ++i) {
    sub.emplace(placeholder_a_vars_(i), prog.state()(num_positions() + num_velocities() + i));
  }
  for (int i = 0; i < num_velocities(); ++i) {
    sub.emplace(placeholder_j_vars_(i), prog.input()(i));
  }
  int input_index = num_velocities();
  for (auto& placeholder_vars : placeholder_spatial_velocity_vars_) {
    for (int i = 0; i < 6; ++i) {
      sub.emplace(placeholder_vars.second(i), prog.input()(input_index));
      ++input_index;
    }
  }
  return sub;
}

symbolic::Substitution
KinematicTrajectoryOptimization::ConstructPlaceholderVariableSubstitution(
    const MultipleShooting& prog, int index) const {
  symbolic::Substitution sub;
  for (int i = 0; i < num_positions(); ++i) {
    sub.emplace(placeholder_q_vars_(i), prog.state(index)(i));
  }
  for (int i = 0; i < num_positions(); ++i) {
    sub.emplace(placeholder_v_vars_(i), prog.state(index)(num_positions() + i));
  }
  for (int i = 0; i < num_positions(); ++i) {
    sub.emplace(placeholder_a_vars_(i), prog.state(index)(num_positions() + num_velocities() + i));
  }
  for (int i = 0; i < num_positions(); ++i) {
    sub.emplace(placeholder_j_vars_(i), prog.input(index)(i));
  }
  int input_index = num_velocities();
  for (auto& placeholder_vars : placeholder_spatial_velocity_vars_) {
    for (int i = 0; i < 6; ++i) {
      sub.emplace(placeholder_vars.second(i), prog.input(index)(input_index));
      ++input_index;
    }
  }
  return sub;
}

solvers::VectorXDecisionVariable KinematicTrajectoryOptimization::SubstitutePlaceholderVariables(
    const solvers::VectorXDecisionVariable& vars,
    const systems::trajectory_optimization::MultipleShooting& prog) {
  VectorXDecisionVariable vars_out{vars.size()};
  for (int i = 0; i < vars.size(); ++i) {
    vars_out(i) =
        *vars.cast<symbolic::Expression>()(i)
             .Substitute(ConstructPlaceholderVariableSubstitution(prog))
             .GetVariables()
             .begin();
  }
  return vars_out;
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
    const systems::trajectory_optimization::MultipleShooting& prog) {
  return g.Substitute(ConstructPlaceholderVariableSubstitution(prog));
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

void KinematicTrajectoryOptimization::AddConstraintToProgram(
    const ConstraintWrapper& constraint, MultipleShooting* prog) {
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
  std::vector<int> active_knots{
      ActiveKnotsForPlanInterval(*prog, constraint.plan_interval)};
  for (int index : active_knots) {
    prog->AddLinearConstraint(SubstitutePlaceholderVariables(constraint.formula, *prog, index));
  }
}

void KinematicTrajectoryOptimization::AddRunningCostToProgram(
    const Expression& cost, MultipleShooting* prog) {
  drake::log()->debug("Adding cost: {}", SubstitutePlaceholderVariables(cost, *prog));
  prog->AddRunningCost(SubstitutePlaceholderVariables(cost, *prog));
}

void KinematicTrajectoryOptimization::AddFinalCostToProgram(
    const Expression& cost, MultipleShooting* prog) {
  drake::log()->debug("Adding cost: {}", SubstitutePlaceholderVariables(cost, *prog));
  prog->AddFinalCost(SubstitutePlaceholderVariables(cost, *prog));
}


void KinematicTrajectoryOptimization::AddCostToProgram(const CostWrapper& cost,
                                                       MultipleShooting* prog) {
  std::vector<int> active_knots{
      ActiveKnotsForPlanInterval(*prog, cost.plan_interval)};
  for (int index : active_knots) {
    prog->AddCost(cost.cost,
                  SubstitutePlaceholderVariables(cost.vars, *prog, index));
  }
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
    positions[i] = prog.GetSolution(prog.state(i).head(num_positions()));
    velocities[i] = prog.GetSolution(
        prog.state(i).segment(num_positions(), num_velocities()));
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
