#include "drake/manipulation/planner/kinematic_trajectory_optimization.h"

#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/systems/trajectory_optimization/direct_transcription.h"

using Eigen::MatrixXd;
using drake::systems::LinearSystem;
using drake::systems::trajectory_optimization::DirectTranscription;
using drake::solvers::SolutionResult;
using drake::solvers::VectorXDecisionVariable;
using drake::solvers::Cost;
using drake::solvers::Constraint;

namespace drake {
namespace manipulation {
namespace planner {
namespace {
class SpatialVelocityCost : public Cost {
 public:
  SpatialVelocityCost(const RigidBodyTree<double>& tree,
                      const RigidBody<double>& body, double dt_)
      : Cost(tree.get_num_positions() + tree.get_num_velocities()),
        tree_(tree),
        body_(body),
        dt_(dt_){}

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
    const AutoDiffVecXd v = x.tail(tree_.get_num_velocities());

    KinematicsCache<AutoDiffXd> cache = tree_.doKinematics(q, v);
    y(0) =
        dt_ *
        tree_.CalcBodySpatialVelocityInWorldFrame(cache, body_).squaredNorm();
  }

  int numOutputs() const { return 1; };

 private:
  const RigidBodyTree<double>& tree_;
  const RigidBody<double>& body_;
  const double dt_;
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

  int numOutputs() const { return 1; };

 private:
  const RigidBodyTree<double>& tree_;
  const RigidBody<double>& body_;
  const Isometry3<double> X_WFd_;
  const Isometry3<double> X_BF_;
};
}  // namespace

KinematicTrajectoryOptimization::KinematicTrajectoryOptimization(
    const RigidBodyTree<double>& tree, int num_time_samples)
    : tree_(tree.Clone()),
      num_time_samples_(num_time_samples) {
  const int kNumStates{3 * num_positions()};
  const int kNumInputs{num_positions()};
  const int kNumOutputs{0};

  const MatrixXd kZero{MatrixXd::Zero(num_positions(), num_positions())};
  const MatrixXd kIdentity{
      MatrixXd::Identity(num_positions(), num_positions())};

  MatrixX<double> A{kNumStates, kNumStates};
  A << kIdentity, dt_*kIdentity, kZero, kZero, kIdentity, dt_*kIdentity, kZero, kZero, kIdentity;

  MatrixXd B{kNumStates, kNumInputs};
  B << kZero, kZero, dt_*kIdentity;

  MatrixXd C{kNumOutputs, kNumStates};
  MatrixXd D{kNumOutputs, kNumInputs};

  system_ = std::make_unique<LinearSystem<double>>(A, B, C, D, dt_);

  prog_ = std::make_unique<DirectTranscription>(
      system_.get(), *(system_->CreateDefaultContext()), num_time_samples_);

  prog_->AddConstraintToAllKnotPoints(prog_->state().head(num_positions()) >=
                                      tree_->joint_limit_min);
  prog_->AddConstraintToAllKnotPoints(prog_->state().head(num_positions()) <=
                                      tree_->joint_limit_max);

};

void KinematicTrajectoryOptimization::AddSpatialVelocityCost(const std::string& body_name, double weight) {
  const RigidBody<double>* body = tree_->FindBody(body_name);
  for (int i = 0; i < num_time_samples_ - 1; ++i) {
    auto cost = std::make_shared<SpatialVelocityCost>(*tree_, *body, weight*dt_);
    VectorXDecisionVariable vars{num_positions() + num_velocities()};
    vars.head(num_positions() + num_velocities()) =
        prog_->state(i).head(num_positions() + num_velocities());
    prog_->AddCost(cost, vars);
  }
}

void KinematicTrajectoryOptimization::AddBodyPoseConstraint(
    int index, const std::string& body_name, const Isometry3<double>& X_WFd,
    double orientation_tolerance, double position_tolerance,
    const Isometry3<double>& X_BF) {
  const RigidBody<double>* body = tree_->FindBody(body_name);
  auto constraint = std::make_shared<BodyPoseConstraint>(
      *tree_, *body, X_WFd, orientation_tolerance, position_tolerance, X_BF);
  VectorXDecisionVariable vars{num_positions()};
  vars.head(num_positions()) = prog_->state(index).head(num_positions());
  prog_->AddConstraint(constraint, vars);
}

void KinematicTrajectoryOptimization::AddRunningCost(
    const symbolic::Expression& g) {
  prog_->AddRunningCost(g);
};

SolutionResult KinematicTrajectoryOptimization::Solve() {
    return prog_->Solve();
};

}  // namespace planner
}  // namespace manipulation
}  // namespace drake
