#include "drake/manipulation/planner/kinematic_trajectory_optimization.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"

namespace drake {
namespace manipulation {
namespace planner {

using solvers::Constraint;
using solvers::VectorXDecisionVariable;
using symbolic::Substitution;

// namespace {
// class BodyPoseConstraint : public Constraint {
// public:
// BodyPoseConstraint(const RigidBodyTree<double>& tree,
// const RigidBody<double>& body,
// const Isometry3<double>& X_WFd,
// double orientation_tolerance, double position_tolerance,
// Isometry3<double> X_BF = Isometry3<double>::Identity())
//: Constraint(2, tree.get_num_positions(),
// Vector2<double>(cos(orientation_tolerance), 0.0),
// Vector2<double>(1.0, std::pow(position_tolerance, 2.0))),
// tree_(tree),
// body_(body),
// X_WFd_(X_WFd),
// X_BF_(X_BF) {}

// virtual void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
// TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
// Eigen::VectorXd& y) const {
// AutoDiffVecXd y_t;
// Eval(math::initializeAutoDiff(x), y_t);
// y = math::autoDiffToValueMatrix(y_t);
//}

// virtual void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
// TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
// AutoDiffVecXd& y) const {
// const AutoDiffVecXd q = x.head(tree_.get_num_positions());

// KinematicsCache<AutoDiffXd> cache = tree_.doKinematics(q);
// const Isometry3<AutoDiffXd> X_WF{tree_.CalcFramePoseInWorldFrame(
// cache, body_, X_BF_.cast<AutoDiffXd>())};
// Isometry3<AutoDiffXd> X_FdF = X_WFd_.inverse().cast<AutoDiffXd>() * X_WF;

// AutoDiffXd prod{Quaternion<AutoDiffXd>(X_FdF.linear())
//.coeffs()
//.dot(Quaternion<AutoDiffXd>::Identity().coeffs())};
// y(0) = 2.0 * prod * prod - 1;
// y(1) = X_FdF.translation().squaredNorm();
//}

// int numOutputs() const { return 2; };

// private:
// const RigidBodyTree<double>& tree_;
// const RigidBody<double>& body_;
// const Isometry3<double> X_WFd_;
// const Isometry3<double> X_BF_;
//};
// class EvaluationPointConstraint : public Constraint {
// virtual void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
// TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
// Eigen::VectorXd& y) const {
// AutoDiffVecXd y_t;
// Eval(math::initializeAutoDiff(x), y_t);
// y = math::autoDiffToValueMatrix(y_t);
//}

// virtual void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
// TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
// AutoDiffVecXd& y) const {
// const AutoDiffVecXd q = x.head(tree_.get_num_positions());

//}

// int numOutputs() const { return 2; };

// private:
// std::shared_ptr<solvers::Constraint> wrapped_constraint;
// std::vector<
//};
//}  // namespace
KinematicTrajectoryOptimization::KinematicTrajectoryOptimization(
    int num_positions, int num_control_points, int num_evaluation_points,
    int spline_order)
    : kNumControlPoints_(num_control_points),
      kNumEvaluationPoints_(num_evaluation_points > 0 ? num_evaluation_points
                                                      : num_control_points),
      kOrder_(spline_order),
      kNumKnots_(num_control_points + spline_order),
      kNumPositions_(num_positions),
      control_points_(
          NewContinuousVariables(kNumPositions_, kNumControlPoints_, "q")) {
  // Populate the knots vector. The knots vector has the form:
  //
  // 	(t[0] ..., t[kOrder], ..., t[kNumControlPoints], ...,
  //    					t[kNumControlPoints + kOrder])
  // where, t[0] == t[1] == ... == t[kOrder-1] == 0, t[kNumControlPoints
  knots_.resize(kNumKnots_, 0);
}

KinematicTrajectoryOptimization::KinematicTrajectoryOptimization(
    const KinematicPlanningProblem* problem, int num_control_points,
    int num_evaluation_points, int spline_order)
    : KinematicTrajectoryOptimization(problem->num_positions(),
                                      num_control_points, num_evaluation_points,
                                      spline_order) {
  problem_ = problem;
  // for (const auto& cost : problem->costs()) {
  // AddCost(cost->cost, cost->plan_interval);
  //}
}

// symbolic::Substitution
// KinematicTrajectoryOptimization::ConstructPlaceholderVariableSubstitution(
// const MultipleShooting& prog, int index) const {
// symbolic::Substitution sub;
// for (int i = 0; i < num_positions(); ++i) {
// sub.emplace(placeholder_q_vars_(i), prog.state(index)(i));
//}
// for (int i = 0; i < num_positions(); ++i) {
// sub.emplace(placeholder_v_vars_(i), prog.state(index)(num_positions() + i));
//}
// for (int i = 0; i < num_positions(); ++i) {
// sub.emplace(placeholder_a_vars_(i), prog.state(index)(num_positions() +
// num_velocities() + i));
//}
// for (int i = 0; i < num_positions(); ++i) {
// sub.emplace(placeholder_j_vars_(i), prog.input(index)(i));
//}
// return sub;
//}

}  // namespace planner
}  // namespace manipulation
}  // namespace drake
