#include "drake/manipulation/planner/kinematic_trajectory_optimization.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"

namespace drake {
namespace manipulation {
namespace planner {

using solvers::Constraint;
using solvers::VectorXDecisionVariable;
using symbolic::Expression;
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
    int spline_order, double duration)
    : kNumControlPoints_(num_control_points),
      kNumEvaluationPoints_(num_evaluation_points > 0 ? num_evaluation_points
                                                      : num_control_points),
      kOrder_(spline_order),
      kNumKnots_(num_control_points + spline_order),
      kNumInternalIntervals_(num_control_points - spline_order + 1),
      kNumPositions_(num_positions),
      kDuration_(duration),
      control_points_(
          NewContinuousVariables(kNumPositions_, kNumControlPoints_, "q")) {
  DRAKE_DEMAND(kNumControlPoints_ >= kOrder_);
  // Populate the knots vector. The knots vector has the form:
  //
  // 	(t[0] ..., t[kOrder], ..., t[kNumControlPoints], ...,
  //    					t[kNumControlPoints + kOrder])
  // where, t[0] == t[1] == ... == t[kOrder-1] == 0, t[kNumControlPoints
  const double kInteriorInterval{kDuration_ / (kNumControlPoints_ - (kOrder_ - 1))};
  knots_.resize(kNumKnots_, 0.0);
  for (int i = kOrder_; i < kNumKnots_; ++i) {
    knots_[i] = std::min(kDuration_, knots_[i - 1] + kInteriorInterval);
  }
  for (int i = 0; i < kNumControlPoints_; ++i) {
    basis_.emplace_back(
        PiecewisePolynomial<double>::BSpline(i, kOrder_, knots_));
    drake::log()->debug("B_{},{}(t) = {}", i, kOrder_,
                        basis_.back().getPolynomial(0));
  }
}

KinematicTrajectoryOptimization::KinematicTrajectoryOptimization(
    const KinematicPlanningProblem* problem, int num_control_points,
    int num_evaluation_points, int spline_order, double duration)
    : KinematicTrajectoryOptimization(problem->num_positions(),
                                      num_control_points, num_evaluation_points,
                                      spline_order, duration) {
  problem_ = problem;
  // for (const auto& cost : problem->costs()) {
  // AddCost(cost->cost, cost->plan_interval);
  //}
}

const VectorX<Expression>
KinematicTrajectoryOptimization::GetSplineVariableExpression(
    double evaluation_time, int derivative_order) const {
  DRAKE_DEMAND(0.0 <= evaluation_time && evaluation_time <= 1.0);
  evaluation_time *= kDuration_;
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

const VectorX<Expression> KinematicTrajectoryOptimization::position(
    double evaluation_time) const {
  return GetSplineVariableExpression(evaluation_time, 0);
}

const VectorX<Expression> KinematicTrajectoryOptimization::velocity(
    double evaluation_time) const {
  return GetSplineVariableExpression(evaluation_time, 1);
}

const VectorX<Expression> KinematicTrajectoryOptimization::acceleration(
    double evaluation_time) const {
  return GetSplineVariableExpression(evaluation_time, 2);
}

const VectorX<Expression> KinematicTrajectoryOptimization::jerk(
    double evaluation_time) const {
  return GetSplineVariableExpression(evaluation_time, 3);
}

PiecewisePolynomialTrajectory
KinematicTrajectoryOptimization::ReconstructPositionTrajectory() const {
  std::vector<MatrixX<Polynomial<double>>> position_polynomials(
      kNumInternalIntervals_);
  for (int i = 0; i < kNumInternalIntervals_; ++i) {
    position_polynomials[i] =
        MatrixX<Polynomial<double>>::Zero(kNumPositions_, 1);
    for (int j = 0; j < kNumPositions_; ++j) {
      // TODO(avalenzu): Only do this for the elements of the basis whose
      // support includes the i-th interval.
      for (int k = 0; k < kNumControlPoints_; ++k) {
        position_polynomials[i](j) += basis_[k].getPolynomial(i, 0, 0) *
                                      GetSolution(control_points_(j, k));
      }
    }
  }
  return PiecewisePolynomialTrajectory(PiecewisePolynomial<double>(
      position_polynomials, basis_.front().getSegmentTimes()));
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
