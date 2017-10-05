#include "drake/manipulation/planner/kinematic_trajectory_optimization.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"

namespace drake {
namespace manipulation {
namespace planner {

using solvers::Constraint;
using solvers::MatrixXDecisionVariable;
using solvers::VectorXDecisionVariable;
using symbolic::Expression;
using symbolic::Substitution;

namespace {
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
                      // TODO(#2274) Fix NOLINTNEXTLINE(runtime / references).
                      Eigen::VectorXd& y) const {
    AutoDiffVecXd y_t;
    Eval(math::initializeAutoDiff(x), y_t);
    y = math::autoDiffToValueMatrix(y_t);
  }

  virtual void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                      // TODO(#2274) Fix NOLINTNEXTLINE(runtime / references).
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
    const VectorX<double> q_value = math::autoDiffToValueMatrix(q);

    KinematicsCache<AutoDiffXd> autodiff_cache = tree_.doKinematics(q);
    KinematicsCache<double> cache = tree_.doKinematics(q_value);
    VectorX<double> distance_value;
    Matrix3X<double> xA, xB, normal;
    std::vector<int> idxA_tmp;
    std::vector<int> idxB_tmp;
    const_cast<RigidBodyTree<double>&>(tree_).collisionDetect(
        cache, distance_value, normal, xA, xB, idxA_tmp, idxB_tmp);
    const int kNumPairs = distance_value.size();
    y(0) = 0;
    y(0).derivatives().resize(x.size());
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
      MatrixX<double> ddist_dq{kNumPairs, x.size()};
      for (int i = 0; i < kNumPairs; ++i) {
        ddist_dq.row(i) = normal.col(i).transpose() * J.middleRows(3 * i, 3) *
                          math::autoDiffToGradientMatrix(q);
      }
      AutoDiffVecXd distance{kNumPairs};
      math::initializeAutoDiffGivenGradientMatrix(distance_value, ddist_dq,
                                                  distance);
      for (int i = 0; i < kNumPairs; ++i) {
        if (distance(i) < 2 * collision_avoidance_threshold_) {
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
  const VectorX<double> spline_weights_;
  const int kNumActiveControlPoints_;
};

}  // namespace

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
  const double kInteriorInterval{kDuration_ / kNumInternalIntervals_};
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
  for (int i = 0; i < kNumControlPoints_; ++i) {
    AddLinearConstraint(control_points_.col(i) >=
                        problem->tree().joint_limit_min);
    AddLinearConstraint(control_points_.col(i) <=
                        problem->tree().joint_limit_max);
  }
}

void KinematicTrajectoryOptimization::AddBodyPoseConstraint(
    double evaluation_time, const std::string& body_name,
    const Isometry3<double> X_WFd, double position_tolerance,
    double orientation_tolerance, const Isometry3<double> X_BF) {
  evaluation_time = ScaleTime(evaluation_time);
  // Compute the control points that contribute to the position at
  // evaluation_time.
  const std::vector<int> active_control_point_indices =
      ComputeActiveControlPointIndices(evaluation_time);

  // Compute the basis values at evaluation_time
  VectorX<double> basis_function_values(kOrder_);
  for (int i = 0; i < kOrder_; ++i) {
    basis_function_values(i) =
        basis_[active_control_point_indices[i]].value(evaluation_time)(0);
  }

  const RigidBody<double>* body = problem_->tree().FindBody(body_name);
  VectorXDecisionVariable vars{num_positions() *
                               active_control_point_indices.size()};
  for (int i = 0; i < kOrder_; ++i) {
    vars.segment(i * kNumPositions_, kNumPositions_) =
        control_points_.col(active_control_point_indices[i]);
  }
  AddConstraint(std::make_shared<BodyPoseConstraint>(
                    problem_->tree(), *body, X_WFd, orientation_tolerance,
                    position_tolerance, X_BF, basis_function_values),
                vars);
}

void KinematicTrajectoryOptimization::AddCollisionAvoidanceConstraint(
    double collision_avoidance_threshold, Vector2<double> plan_interval) {
  // Re-scale the plan interval
  plan_interval(0) = ScaleTime(plan_interval(0));
  plan_interval(1) = ScaleTime(plan_interval(1));

  // Evaluate constraints at points spaced by no more than one-half the interval
  // between knots.
  const double kInteriorInterval{kDuration_ / kNumInternalIntervals_};
  const int kNumEvaluationTimes(std::ceil(
      (plan_interval(1) - plan_interval(0)) / (0.5 * kInteriorInterval)));
  const VectorX<double> evaluation_times{VectorX<double>::LinSpaced(
      kNumEvaluationTimes, plan_interval(0), plan_interval(1))};
  for (int i = 0; i < kNumEvaluationTimes; ++i) {
    const std::vector<int> active_control_point_indices{
        ComputeActiveControlPointIndices(evaluation_times(i))};
    // Compute the basis values at evaluation_time
    VectorX<double> basis_function_values(kOrder_);
    for (int j = 0; j < kOrder_; ++j) {
      basis_function_values(j) =
          basis_[active_control_point_indices[j]].value(evaluation_times(i))(0);
    }
    auto constraint = std::make_shared<CollisionAvoidanceConstraint>(
        problem_->tree(), collision_avoidance_threshold, basis_function_values);
    VectorXDecisionVariable vars{num_positions() *
                                 active_control_point_indices.size()};
    for (int j = 0; j < kOrder_; ++j) {
      vars.segment(j * kNumPositions_, kNumPositions_) =
          control_points_.col(active_control_point_indices[j]);
    }
    AddConstraint(constraint, vars);
  }
}

double KinematicTrajectoryOptimization::ScaleTime(double time) const {
  DRAKE_DEMAND(0.0 <= time && time <= 1.0);
  return time * kDuration_;
}

std::vector<int>
KinematicTrajectoryOptimization::ComputeActiveControlPointIndices(
    double evaluation_time) const {
  std::vector<int> active_control_point_indices(kOrder_);
  for (int i = 0; i < kNumControlPoints_; ++i) {
    if (knots_[i] <= evaluation_time + kEpsilonTime_ &&
        evaluation_time <= knots_[i + kOrder_] + kEpsilonTime_) {
      active_control_point_indices[0] = i;
      break;
    }
  }
  for (int i = 1; i < kOrder_; ++i) {
    active_control_point_indices[i] = active_control_point_indices[i - 1] + 1;
  }
  return active_control_point_indices;
}

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

const VectorX<Expression> KinematicTrajectoryOptimization::position(
    double evaluation_time) const {
  drake::log()->debug("position({}) = \n{}", evaluation_time,
                      GetSplineVariableExpression(evaluation_time, 0));
  return GetSplineVariableExpression(evaluation_time, 0);
}

const VectorX<Expression> KinematicTrajectoryOptimization::velocity(
    double evaluation_time) const {
  drake::log()->debug("velocity({}) = \n{}", evaluation_time,
                      GetSplineVariableExpression(evaluation_time, 1));
  return GetSplineVariableExpression(evaluation_time, 1);
}

const VectorX<Expression> KinematicTrajectoryOptimization::acceleration(
    double evaluation_time) const {
  drake::log()->debug("acceleration({}) = \n{}", evaluation_time,
                      GetSplineVariableExpression(evaluation_time, 2));
  return GetSplineVariableExpression(evaluation_time, 2);
}

const VectorX<Expression> KinematicTrajectoryOptimization::jerk(
    double evaluation_time) const {
  drake::log()->debug("jerk({}) = \n{}", evaluation_time,
                      GetSplineVariableExpression(evaluation_time, 3));
  return GetSplineVariableExpression(evaluation_time, 3);
}

PiecewisePolynomialTrajectory
KinematicTrajectoryOptimization::ReconstructTrajectory(
    const int derivative_order) const {
  std::vector<MatrixX<Polynomial<double>>> polynomials(kNumInternalIntervals_);
  for (int i = 0; i < kNumInternalIntervals_; ++i) {
    polynomials[i] = MatrixX<Polynomial<double>>::Zero(
        (derivative_order + 1) * kNumPositions_, 1);
    for (int j = 0; j < kNumPositions_; ++j) {
      // TODO(avalenzu): Only do this for the elements of the basis whose
      // support includes the i-th interval.
      for (int k = 0; k < kNumControlPoints_; ++k) {
        for (int ii = 0; ii <= derivative_order; ++ii) {
          polynomials[i](ii * kNumPositions_ + j) +=
              basis_[k].derivative(ii).getPolynomial(i, 0, 0) *
              GetSolution(control_points_(j, k));
        }
      }
    }
  }
  return PiecewisePolynomialTrajectory(PiecewisePolynomial<double>(
      polynomials, basis_.front().getSegmentTimes()));
}

// symbolic::Substitution
// KinematicTrajectoryOptimization::ConstructPlaceholderVariableSubstitution(
// const MultipleShooting& prog, int index) const {
// symbolic::Substitution sub;
// for (int i = 0; i < num_positions(); ++i) {
// sub.emplace(placeholder_q_vars_(i), prog.state(index)(i));
//}
// for (int i = 0; i < num_positions(); ++i) {
// sub.emplace(placeholder_v_vars_(i), prog.state(index)(num_positions() +
// i));
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
