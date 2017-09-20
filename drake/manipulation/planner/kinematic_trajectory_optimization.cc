#include "drake/manipulation/planner/kinematic_trajectory_optimization.h"

#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/systems/trajectory_optimization/direct_collocation.h"

using Eigen::MatrixXd;
using drake::systems::LinearSystem;
using drake::systems::trajectory_optimization::DirectCollocation;
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
                      const RigidBody<double>& body, double weight)
      : Cost(tree.get_num_positions() + tree.get_num_velocities()),
        tree_(tree),
        body_(body),
        weight_(weight){}

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
        weight_ *
        tree_.CalcBodySpatialVelocityInWorldFrame(cache, body_).squaredNorm();
  }

  int numOutputs() const { return 1; };

 private:
  const RigidBodyTree<double>& tree_;
  const RigidBody<double>& body_;
  const double weight_;
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
    AutoDiffVecXd y_t;
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
}  // namespace

KinematicTrajectoryOptimization::KinematicTrajectoryOptimization(
    std::unique_ptr<RigidBodyTree<double>> tree, int num_time_samples,
    double minimum_timestep, double maximum_timestep)
    : tree_(std::move(tree)), num_time_samples_(num_time_samples) {
  const int kNumStates{3 * num_positions()};
  const int kNumInputs{num_positions()};
  const int kNumOutputs{0};

  const MatrixXd kZero{MatrixXd::Zero(num_positions(), num_positions())};
  const MatrixXd kIdentity{
      MatrixXd::Identity(num_positions(), num_positions())};

  MatrixX<double> A{kNumStates, kNumStates};
  A << kZero, kIdentity, kZero, kZero, kZero, kIdentity, kZero, kZero, kZero;

  MatrixXd B{kNumStates, kNumInputs};
  B << kZero, kZero, kIdentity;

  MatrixXd C{kNumOutputs, kNumStates};
  MatrixXd D{kNumOutputs, kNumInputs};

  system_ = std::make_unique<LinearSystem<double>>(A, B, C, D);

  prog_ = std::make_unique<DirectCollocation>(
      system_.get(), *(system_->CreateDefaultContext()), num_time_samples_,
      minimum_timestep, maximum_timestep);

  prog_->AddConstraintToAllKnotPoints(prog_->state().head(num_positions()) >=
                                      tree_->joint_limit_min);
  prog_->AddConstraintToAllKnotPoints(prog_->state().head(num_positions()) <=
                                      tree_->joint_limit_max);

};

void KinematicTrajectoryOptimization::AddFinalCost(const symbolic::Expression& g) {
  prog_->AddFinalCost(g);
}

void KinematicTrajectoryOptimization::AddEqualTimeIntervalsConstraints() {
  prog_->AddEqualTimeIntervalsConstraints();
};

void KinematicTrajectoryOptimization::AddDurationBounds(double lower_bound,
                                                        double upper_bound) {
  prog_->AddDurationBounds(lower_bound, upper_bound);
};

void KinematicTrajectoryOptimization::AddConstraintToAllKnotPoints(
    const symbolic::Formula& f) {
  prog_->AddConstraintToAllKnotPoints(f);
}

void KinematicTrajectoryOptimization::AddSpatialVelocityCost(const std::string& body_name, double weight) {
  const RigidBody<double>* body = tree_->FindBody(body_name);
  for (int i = 0; i < num_time_samples_ - 1; ++i) {
    auto cost = std::make_shared<SpatialVelocityCost>(*tree_, *body, weight);
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

void KinematicTrajectoryOptimization::AddCollisionAvoidanceConstraint(
    double collision_avoidance_threshold, int index) {
  DRAKE_DEMAND(index >= 0 && index < num_time_samples_);
  auto constraint = std::make_shared<CollisionAvoidanceConstraint>(
      *tree_, collision_avoidance_threshold);
  VectorXDecisionVariable vars{num_positions()};
  vars.head(num_positions()) = prog_->state(index).head(num_positions());
  prog_->AddConstraint(constraint, vars);
}

void KinematicTrajectoryOptimization::AddCollisionAvoidanceConstraint(
    double collision_avoidance_threshold) {
  for (int i = 0; i < num_time_samples_; ++i) {
    AddCollisionAvoidanceConstraint(collision_avoidance_threshold, i);
  }
}

void KinematicTrajectoryOptimization::AddRunningCost(
    const symbolic::Expression& g) {
  prog_->AddRunningCost(g);
};

SolutionResult KinematicTrajectoryOptimization::Solve() {
    return prog_->Solve();
};

PiecewisePolynomialTrajectory
KinematicTrajectoryOptimization::ReconstructStateTrajectory() const {
  return prog_->ReconstructStateTrajectory();
};

PiecewisePolynomialTrajectory
KinematicTrajectoryOptimization::ReconstructInputTrajectory() const {
  return prog_->ReconstructInputTrajectory();
};

template <typename T>
void KinematicTrajectoryOptimization::SetSolverOption(
    const solvers::SolverId& solver_id, const std::string& solver_option,
    T option_value) {
  prog_->SetSolverOption(solver_id, solver_option, option_value);
};

void KinematicTrajectoryOptimization::AddLinearConstraint(
    const symbolic::Formula& f) {
  prog_->AddLinearConstraint(f);
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
