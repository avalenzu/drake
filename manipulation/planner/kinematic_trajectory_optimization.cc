#include "drake/manipulation/planner/kinematic_trajectory_optimization.h"

#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/rigid_body_tree.h"

using Eigen::MatrixXd;
using drake::solvers::Constraint;
using drake::solvers::Cost;
using drake::solvers::MathematicalProgram;
using drake::solvers::SolutionResult;
using drake::solvers::VectorXDecisionVariable;
using drake::symbolic::Expression;
using drake::symbolic::Formula;
using drake::systems::LinearSystem;
using drake::systems::System;

namespace drake {
namespace manipulation {
namespace planner {
namespace {
class SpatialVelocityConstraint : public Constraint {
 public:
  SpatialVelocityConstraint(const RigidBodyTree<double>& tree,
                            const RigidBody<double>& body, double tolerance)
      : Constraint(6, tree.get_num_positions() + tree.get_num_velocities() + 6,
                   -tolerance * Vector6<double>::Ones(),
                   tolerance * Vector6<double>::Ones()),
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
    // Isometry3<AutoDiffXd> X_FW{tree_.CalcBodyPoseInWorldFrame(cache,
    // body_).inverse()};  const AutoDiffVecXd
    // wv_WF{tree_.CalcBodySpatialVelocityInWorldFrame(cache, body_)};  const
    // Vector3<AutoDiffXd> w_WF{wv_WF.head(3)};  const Vector3<AutoDiffXd>
    // v_WF{wv_WF.tail(3)};  y.head(3) = spatial_velocity.head(3) - X_FW*w_WF;
    // y.tail(3) = spatial_velocity.tail(3) - X_FW*v_WF;
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
                     Isometry3<double> X_BF,
                     const VectorX<double> spline_weights)
      : Constraint(2, spline_weights.size() * tree.get_num_positions(),
                   Vector2<double>(cos(orientation_tolerance), 0.0),
                   Vector2<double>(1.0, std::pow(position_tolerance, 2.0))),
        tree_(tree),
        body_(body),
        X_WFd_(X_WFd),
        X_BF_(X_BF),
        spline_weights_(spline_weights),
        kNumActiveControlPoints_(spline_weights.size()) {}

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
    const int kNumPositions{tree_.get_num_positions()};
    AutoDiffVecXd q{x.head(kNumPositions) * spline_weights_(0)};
    for (int i = 1; i < spline_weights_.size(); ++i) {
      q += x.segment(i * kNumPositions, kNumPositions) * spline_weights_(i);
    }

    KinematicsCache<AutoDiffXd> cache = tree_.doKinematics(q);
    const Isometry3<AutoDiffXd> X_WF{tree_.CalcFramePoseInWorldFrame(
        cache, body_, X_BF_.cast<AutoDiffXd>())};
    Isometry3<AutoDiffXd> X_FdF = X_WFd_.inverse().cast<AutoDiffXd>() * X_WF;

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
  const VectorX<double> spline_weights_;
  const int kNumActiveControlPoints_;
};

AutoDiffVecXd CollisionAvoidancePenalty(const RigidBodyTree<double>& tree,
                          double collision_avoidance_threshold,
                          const Eigen::Ref<const AutoDiffVecXd>& q) {
  const VectorX<double> q_value = math::autoDiffToValueMatrix(q);

  KinematicsCache<AutoDiffXd> autodiff_cache = tree.doKinematics(q);
  KinematicsCache<double> cache = tree.doKinematics(q_value);
  VectorX<double> distance_value;
  Matrix3X<double> xA, xB, normal;
  std::vector<int> idxA_tmp;
  std::vector<int> idxB_tmp;
  const_cast<RigidBodyTree<double>&>(tree).collisionDetect(
      cache, distance_value, normal, xA, xB, idxA_tmp, idxB_tmp);
  const int num_pairs = distance_value.size();
  AutoDiffVecXd y(1);
  y(0) = 0;
  y(0).derivatives().resize(q(0).derivatives().size());
  y(0).derivatives().setZero();
  if (num_pairs > 0) {
    VectorX<int> idxA(num_pairs);
    VectorX<int> idxB(num_pairs);
    for (int i = 0; i < num_pairs; ++i) {
      idxA(i) = idxA_tmp[i];
      idxB(i) = idxB_tmp[i];
    }
    MatrixX<double> J;
    tree.computeContactJacobians(cache, idxA, idxB, xA, xB, J);
    MatrixX<double> ddist_dq{num_pairs, q(0).derivatives().size()};
    for (int i = 0; i < num_pairs; ++i) {
      ddist_dq.row(i) = normal.col(i).transpose() * J.middleRows(3 * i, 3) *
                        math::autoDiffToGradientMatrix(q);
    }
    AutoDiffVecXd distance{num_pairs};
    math::initializeAutoDiffGivenGradientMatrix(distance_value, ddist_dq,
                                                distance);
    for (int i = 0; i < num_pairs; ++i) {
      if (distance(i) < 2 * collision_avoidance_threshold) {
        distance(i) /= collision_avoidance_threshold;
        distance(i) -= 2;
        y(0) += -distance(i) * exp(1 / distance(i));
      }
    }
  }
  return y;
}

double CollisionAvoidancePenalty(const RigidBodyTree<double>& tree,
                                 double collision_avoidance_threshold,
                                 const Eigen::Ref<const VectorX<double>>& q) {
  return CollisionAvoidancePenalty(tree, collision_avoidance_threshold,
                                   math::initializeAutoDiff(q))(0)
      .value();
}

class CollisionAvoidanceConstraint : public Constraint {
 public:
  CollisionAvoidanceConstraint(const RigidBodyTree<double>& tree,
                               double collision_avoidance_threshold,
                               const VectorX<double> spline_weights)
      : Constraint(1, spline_weights.size() * tree.get_num_positions(),
                   Vector1<double>(0), Vector1<double>(exp(-1))),
        tree_(tree),
        collision_avoidance_threshold_(collision_avoidance_threshold),
        spline_weights_(spline_weights),
        kNumActiveControlPoints_(spline_weights.size()) {}

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
    const int kNumPositions{tree_.get_num_positions()};
    AutoDiffVecXd q{x.head(kNumPositions) * spline_weights_(0)};
    for (int i = 1; i < spline_weights_.size(); ++i) {
      q += x.segment(i * kNumPositions, kNumPositions) * spline_weights_(i);
    }
    y = CollisionAvoidancePenalty(tree_, collision_avoidance_threshold_, q);
  }

  int numOutputs() const { return 1; };

 private:
  const RigidBodyTree<double>& tree_;
  double collision_avoidance_threshold_;
  const VectorX<double> spline_weights_;
  const int kNumActiveControlPoints_;
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

  for (int i = 0; i < kNumTimesToCheck; ++i) {
    if (CollisionAvoidancePenalty(
            tree(), threshold, position_trajectory_.value(kTimesToCheck(i))) >
        0.0) {
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
      placeholder_t_var_(
          solvers::VectorDecisionVariable<1>(symbolic::Variable("t"))),
      placeholder_q_vars_(MakeNamedVariables("q", tree_->get_num_positions())),
      placeholder_v_vars_(MakeNamedVariables("v", tree_->get_num_velocities())),
      placeholder_a_vars_(MakeNamedVariables("a", tree_->get_num_velocities())),
      placeholder_j_vars_(MakeNamedVariables("j", tree_->get_num_velocities())),
      initial_position_trajectory_(PiecewisePolynomial<double>::ZeroOrderHold(
          {0, 1},
          {tree_->getZeroConfiguration(), tree_->getZeroConfiguration()})),
      position_trajectory_(
          PiecewisePolynomialTrajectory(initial_position_trajectory_)){};

void KinematicTrajectoryOptimization::AddFixedBoxToWorld(
    Vector3<double> size, Isometry3<double> X_WB) {
  DrakeShapes::Box geom(size);
  RigidBody<double>& world = tree_->world();
  Eigen::Vector4d color;
  color << 0.5, 0.5, 0.5, 1;
  world.AddVisualElement(DrakeShapes::VisualElement(geom, X_WB, color));
  tree_->addCollisionElement(
      drake::multibody::collision::Element(geom, X_WB, &world), world,
      "terrain");
  tree_->compile();
}

void KinematicTrajectoryOptimization::AddFinalCost(
    const symbolic::Expression& g) {
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
    const std::string& body_name, double tolerance) {
  VectorXDecisionVariable vars{num_positions() + num_velocities() + 6};
  vars.head(num_positions()) = position();
  vars.segment(num_positions(), num_velocities()) = velocity();
  vars.tail(6) =
      placeholder_spatial_velocity_vars_
          .emplace(body_name,
                   MakeNamedVariables(body_name + "_spatial_velocity", 6))
          .first->second;
  const RigidBody<double>* body = tree_->FindBody(body_name);
  object_constraints_.emplace_back(new ConstraintWrapper(
      {std::make_shared<SpatialVelocityConstraint>(*tree_, *body, tolerance),
       vars,
       {0, 1}}));
}

void KinematicTrajectoryOptimization::AddBodyPoseConstraint(
    double time, const std::string& body_name, const Isometry3<double>& X_WFd,
    double orientation_tolerance, double position_tolerance,
    const Isometry3<double>& X_BF) {
  AddBodyPoseConstraint({time, time}, body_name, X_WFd, orientation_tolerance,
                        position_tolerance, X_BF);
}

void KinematicTrajectoryOptimization::AddSpatialVelocityCost(
    const std::string& body_name, double weight) {
  TrackSpatialVelocityOfBody(body_name, spatial_velocity_tolerance_);
  auto spatial_velocity_vars = spatial_velocity_of_body(body_name);
  AddRunningCost(weight * spatial_velocity_vars.transpose() *
                 spatial_velocity_vars);
}

bool KinematicTrajectoryOptimization::IsValidPlanInterval(
    const Vector2<double>& plan_interval) {
  return 0 <= plan_interval(0) && plan_interval(0) <= 1 &&
         0 <= plan_interval(1) && plan_interval(1) <= 1 &&
         plan_interval(0) <= plan_interval(1);
}

void KinematicTrajectoryOptimization::AddBodyPoseConstraint(
    Vector2<double> plan_interval, const std::string& body_name,
    const Isometry3<double>& X_WFd, double orientation_tolerance,
    double position_tolerance, const Isometry3<double>& X_BF) {
  DRAKE_THROW_UNLESS(IsValidPlanInterval(plan_interval));
  const RigidBody<double>* body = tree_->FindBody(body_name);
  VectorXDecisionVariable vars{num_positions()};
  vars.head(num_positions()) = position();
  object_constraints_.emplace_back(
      new ConstraintWrapper({std::make_shared<BodyPoseConstraint>(
                                 *tree_, *body, X_WFd, orientation_tolerance,
                                 position_tolerance, X_BF),
                             vars, plan_interval}));
}

void KinematicTrajectoryOptimization::AddCollisionAvoidanceConstraint(
    Vector2<double> plan_interval, double collision_avoidance_threshold) {
  DRAKE_THROW_UNLESS(IsValidPlanInterval(plan_interval));
  auto constraint = std::make_shared<CollisionAvoidanceConstraint>(
      *tree_, collision_avoidance_threshold);
  VectorXDecisionVariable vars{num_positions()};
  vars.head(num_positions()) = position();
  object_constraints_.emplace_back(
      new ConstraintWrapper({std::make_shared<CollisionAvoidanceConstraint>(
                                 *tree_, collision_avoidance_threshold),
                             vars, plan_interval}));
}

void KinematicTrajectoryOptimization::AddCollisionAvoidanceConstraint(
    double collision_avoidance_threshold) {
  AddCollisionAvoidanceConstraint({0, 1}, collision_avoidance_threshold);
}

void KinematicTrajectoryOptimization::AddRunningCost(
    const symbolic::Expression& g) {
  running_cost_expressions_.emplace_back(new symbolic::Expression(g));
};

const VectorX<Expression>
KinematicTrajectoryOptimization::GetSplineVariableExpression(
    double evaluation_time, int derivative_order) const {
  evaluation_time = ScaleTime(evaluation_time);
  VectorX<Expression> expression(kNumPositions_);
  for (int i = 0; i < kNumPositions_; ++i) {
    expression(i) = Expression::Zero();
    for (int j = 0; j < kNumControlPoints_; ++j) {
      if (knots_[j] <= evaluation_time &&
          evaluation_time <= knots_[j + kOrder_]) {
        const double basis_function_value{
            basis_[j].derivative(derivative_order).value(evaluation_time)(0)};
        if (std::abs<double>(basis_function_value) > 1e-6) {
          expression(i) += control_points_(i, j) * basis_function_value;
        }
      }
    }
  }
  return expression;
}

const Expression KinematicTrajectoryOptimization::ConstructPositionExpression(
    const MathematicalProgram& prog, double time) const {
  // Position variables are always at the head of the state.
  return GetStateVariablesFromProgram(prog, index).head(num_positions());
}

const Expression KinematicTrajectoryOptimization::ConstructVelocityExpression(
    const MathematicalProgram& prog, double time) const {
  // For first-order systems, velocity variables are the first inputs. For all
  // others, they follow the position variables in the state.
  if (system_order_ < 2) {
    return GetInputVariablesFromProgram(prog, index).head(num_velocities());
  } else {
    return GetStateVariablesFromProgram(prog, index)
        .segment(num_positions(), num_velocities());
  }
}

const solvers::VectorXDecisionVariable
KinematicTrajectoryOptimization::GetAccelerationVariablesFromProgram(
    const MathematicalProgram& prog, int index) const {
  // For first-order systems, acceleration variables are ignored. For
  // second-order systems, they are the first inputs. For all others, they
  // follow the velocity variables in the state.
  if (system_order_ < 2) {
    return placeholder_a_vars_;
  } else if (system_order_ < 3) {
    return GetInputVariablesFromProgram(prog, index).head(num_velocities());
  } else {
    return GetStateVariablesFromProgram(prog, index)
        .segment(num_positions() + num_velocities(), num_velocities());
  }
}

const solvers::VectorXDecisionVariable
KinematicTrajectoryOptimization::GetJerkVariablesFromProgram(
    const MathematicalProgram& prog, int index) const {
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
    const MathematicalProgram& prog, int index) const {
  // For {first, second, third}-order systems, the body spatial velocity
  // variables come after the {velocity, acceleration, jerk} variables in the
  // input.
  return GetInputVariablesFromProgram(prog, index)
      .segment(num_velocities(), num_body_spatial_velocity_variables());
}

symbolic::Substitution
KinematicTrajectoryOptimization::ConstructPlaceholderVariableSubstitution(
    const VectorXDecisionVariable& position_variables, int index) const {
  symbolic::Substitution sub;
  // Get the actual decision variables from the mathematical program.
  for (int i = 0; i < num_positions(); ++i) {
    sub.emplace(placeholder_q_vars_(i), position_variables.col(index)(i));
  }
  return sub;
}

solvers::VectorXDecisionVariable
KinematicTrajectoryOptimization::SubstitutePlaceholderVariables(
    const solvers::VectorXDecisionVariable& vars,
    const VectorXDecisionVariable& position_variables, int index) {
  VectorXDecisionVariable vars_out{vars.size()};
  for (int i = 0; i < vars.size(); ++i) {
    vars_out(i) = *vars.cast<symbolic::Expression>()(i)
                       .Substitute(ConstructPlaceholderVariableSubstitution(
                           position_variables, index))
                       .GetVariables()
                       .begin();
  }
  return vars_out;
}

Formula KinematicTrajectoryOptimization::SubstitutePlaceholderVariables(
    const Formula& f,
    const MathematicalProgram& prog, int index) {
  return f.Substitute(ConstructPlaceholderVariableSubstitution(prog, index));
}

Expression KinematicTrajectoryOptimization::SubstitutePlaceholderVariables(
    const Expression& g,
    const MathematicalProgram& prog, int index) {
  return g.Substitute(ConstructPlaceholderVariableSubstitution(prog, index));
}

std::vector<int> KinematicTrajectoryOptimization::ActiveKnotsForPlanInterval(
    const MathematicalProgram& prog,
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
    const ConstraintWrapper& constraint, MathematicalProgram* prog) {
  // Ignore constraints on higher order terms not present in the model.
  if (!AreVariablesPresentInProgram(symbolic::Variables(constraint.vars)))
    return;
  std::vector<int> active_knots{
      ActiveKnotsForPlanInterval(*prog, constraint.plan_interval)};
  for (int index : active_knots) {
    prog->AddConstraint(
        constraint.constraint,
        SubstitutePlaceholderVariables(constraint.vars, *prog, index));
  }
}

void KinematicTrajectoryOptimization::AddLinearConstraintToProgram(
    const FormulaWrapper& constraint, MathematicalProgram* prog) {
  if (!AreVariablesPresentInProgram(constraint.formula.GetFreeVariables()))
    return;
  std::vector<int> active_knots{
      ActiveKnotsForPlanInterval(*prog, constraint.plan_interval)};
  for (int index : active_knots) {
    prog->AddLinearConstraint(
        SubstitutePlaceholderVariables(constraint.formula, *prog, index));
  }
}

void KinematicTrajectoryOptimization::AddRunningCostToProgram(
    const Expression& cost, MathematicalProgram* prog) {
  if (!AreVariablesPresentInProgram(cost.GetVariables())) return;
  drake::log()->debug("Adding cost: {}",
                      SubstitutePlaceholderVariables(cost, *prog));
  prog->AddRunningCost(SubstitutePlaceholderVariables(cost, *prog));
}

void KinematicTrajectoryOptimization::AddFinalCostToProgram(
    const Expression& cost, MathematicalProgram* prog) {
  if (!AreVariablesPresentInProgram(cost.GetVariables())) return;
  drake::log()->debug("Adding cost: {}",
                      SubstitutePlaceholderVariables(cost, *prog));
  prog->AddFinalCost(SubstitutePlaceholderVariables(cost, *prog));
}

void KinematicTrajectoryOptimization::AddCostToProgram(const CostWrapper& cost,
                                                       MathematicalProgram* prog) {
  if (!AreVariablesPresentInProgram(symbolic::Variables(cost.vars))) return;
  std::vector<int> active_knots{
      ActiveKnotsForPlanInterval(*prog, cost.plan_interval)};
  for (int index : active_knots) {
    prog->AddCost(cost.cost,
                  SubstitutePlaceholderVariables(cost.vars, *prog, index));
  }
}

void KinematicTrajectoryOptimization::SetInitialTrajectoryOnProgram(
    MathematicalProgram* prog) {
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
    // TODO(avalenzu) Actually compute the spatial accelerations.
    KinematicsCache<double> cache = tree_->doKinematics(position, velocity);
    std::map<std::string, VectorX<double>> body_spatial_velocity;
    for (const auto& pair : placeholder_spatial_velocity_vars_) {
      body_spatial_velocity.emplace(pair.first,
                                    tree_->CalcBodySpatialVelocityInWorldFrame(
                                        cache, *tree_->FindBody(pair.first)));
    }
    const VectorX<double> body_spatial_acceleration{
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
    int spatial_velocity_index{0};
    for (const auto& pair : placeholder_spatial_velocity_vars_) {
      const VectorX<double> spatial_velocity{
          body_spatial_velocity.at(pair.first)};
      for (int j = 0; j < 6; ++j) {
        x_environment.insert(GetBodySpatialVelocityVariablesFromProgram(*prog)(
                                 spatial_velocity_index),
                             spatial_velocity(j));
        xdot_environment.insert(GetBodySpatialVelocityVariablesFromProgram(
                                    *prog)(spatial_velocity_index),
                                0.0);
        spatial_velocity_index++;
      }
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
  DRAKE_THROW_UNLESS(num_control_points >= spline_order);
  // Populate the knots vector. The knots vector has the form:
  //
  // 	(t[0] ..., t[kOrder], ..., t[kNumControlPoints], ...,
  //    					t[kNumControlPoints + kOrder])
  // where, t[0] == t[1] == ... == t[kOrder-1] == 0, t[kNumControlPoints
  const double kInteriorInterval{kDuration_ / kNumInternalIntervals_};
  knots_.resize(kNumKnots_, 0.0);
  for (int i = spline_order; i < kNumKnots_; ++i) {
    knots_[i] = std::min(kDuration_, knots_[i - 1] + kInteriorInterval);
  }
  for (int i = 0; i < num_control_points; ++i) {
    basis_.emplace_back(
        PiecewisePolynomial<double>::BSpline(i, spline_order, knots_));
    drake::log()->debug("B_{},{}(t) = {}", i, spline_order,
                        basis_.back().getPolynomial(0));
  }

  MathematicalProgram prog{};
  for (const auto& solver_id : solver_options_solver_ids_) {
    for (const auto& option_pair :
         solver_options_container_.GetSolverOptionsInt(solver_id)) {
      prog->SetSolverOption(solver_id, option_pair.first, option_pair.second);
    }
    for (const auto& option_pair :
         solver_options_container_.GetSolverOptionsStr(solver_id)) {
      prog->SetSolverOption(solver_id, option_pair.first, option_pair.second);
    }
    for (const auto& option_pair :
         solver_options_container_.GetSolverOptionsDouble(solver_id)) {
      prog->SetSolverOption(solver_id, option_pair.first, option_pair.second);
    }
  }
  auto position_variables{
      prog.NewContinuousVariables(num_positions(), num_control_points, "q")};
  auto position_variables{GetPositionVariablesFromProgram(*prog)};
  // Add joint limits
  for (int i = 0; i < num_control_points; ++i) {
    prog.AddLinearConstraint(position_variables.col(i) >=
                                       tree_->joint_limit_min);
    prog.AddLinearConstraint(position_variables.col(i) <=
                                       tree_->joint_limit_max);
  }

  for (const auto& constraint : object_constraints_) {
    std::vector<int> active_knots{
        ActiveKnotsForPlanInterval(*prog, constraint.plan_interval)};
    for (int index : active_knots) {
      prog->AddConstraint(constraint.constraint,
                          SubstitutePlaceholderVariables(
                              constraint.vars, position_variables, index));
    }
  }
  for (const auto& constraint : formula_linear_constraints_) {
    AddLinearConstraintToProgram(*constraint, &prog);
  }
  //for (const auto& cost : running_cost_objects_) {
    //AddCostToProgram(*cost, &prog);
  //}
  //for (const auto& cost : running_cost_expressions_) {
    //AddRunningCostToProgram(*cost, &prog);
  //}
  //for (const auto& cost : final_cost_expressions_) {
    //AddFinalCostToProgram(*cost, &prog);
  //}

  SetInitialTrajectoryOnProgram(&prog);

  drake::log()->debug("Solving MathematicalProgram with {} decision variables.",
                      prog->decision_variables().size());
  SolutionResult result{prog.Solve()};

  UpdatePositionTrajectory(*prog);

  return result;
}

void KinematicTrajectoryOptimization::UpdatePositionTrajectory(
    const MathematicalProgram& prog) {
  VectorX<double> times{prog.GetSampleTimes()};
  std::vector<double> times_vec(num_time_samples_);
  std::vector<MatrixX<double>> positions(num_time_samples_);
  std::vector<MatrixX<double>> velocities(num_time_samples_);
  for (int i = 0; i < num_time_samples_; ++i) {
    times_vec[i] = times(i);
    positions[i] = prog.GetSolution(GetPositionVariablesFromProgram(prog, i));
    velocities[i] = prog.GetSolution(GetVelocityVariablesFromProgram(prog, i));
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
  solver_options_solver_ids_.push_back(solver_id);
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
  formula_linear_constraints_.emplace_back(
      new FormulaWrapper({f, plan_interval}));
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
