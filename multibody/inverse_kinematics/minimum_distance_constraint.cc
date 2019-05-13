#include "drake/multibody/inverse_kinematics/minimum_distance_constraint.h"

#include <limits>
#include <vector>

#include <Eigen/Dense>

#include "drake/multibody/inverse_kinematics/kinematic_constraint_utilities.h"

namespace drake {
namespace multibody {
using internal::RefFromPtrOrThrow;

namespace {
/** Computes log(∑exp(xᵢ)). Exploits the shift invariance of log-sum-exp to
avoid overflow. */
template <typename T>
T LogSumExp(const std::vector<T>& x) {
  using std::log;
  using std::exp;
  const T x_max = *std::max_element(x.begin(), x.end());
  T sum_exp{0.0};
  for (const T& xi : x) {
    sum_exp += exp(xi - x_max);
  }
  return x_max + log(sum_exp);
}

/** Computes a smooth approximation of max(x). */
template <typename T>
T SmoothMax(const std::vector<T>& x) {
  double alpha{100};
  std::vector<T> x_scaled{x};
  for (T& xi_scaled : x_scaled) {
    xi_scaled *= alpha;
  }
  drake::log()->trace("SmoothMax: return {}", LogSumExp(x_scaled) / alpha);
  return LogSumExp(x_scaled) / alpha;
}

}  // namespace

void ExponentiallySmoothedHingeLoss(double x, double* penalty,
                                    double* dpenalty_dx) {
  if (x >= 0) {
    *penalty = 0;
    if (dpenalty_dx) {
      *dpenalty_dx = 0;
    }
  } else {
    const double exp_one_over_x = std::exp(1.0 / x);
    *penalty = -x * exp_one_over_x;
    if (dpenalty_dx) {
      *dpenalty_dx = -exp_one_over_x + exp_one_over_x / x;
    }
  }
}

void QuadraticallySmoothedHingeLoss(double x, double* penalty,
                                    double* dpenalty_dx) {
  if (x >= 0) {
    *penalty = 0;
    if (dpenalty_dx) {
      *dpenalty_dx = 0;
    }
  } else {
    if (x > -1) {
      *penalty = x * x / 2;
      if (dpenalty_dx) {
        *dpenalty_dx = x;
      }
    } else {
      *penalty = -0.5 - x;
      if (dpenalty_dx) {
        *dpenalty_dx = -1;
      }
    }
  }
}

MinimumDistanceConstraint::MinimumDistanceConstraint(
    const multibody::MultibodyPlant<double>* const plant,
    double minimum_distance, systems::Context<double>* plant_context,
    MinimumDistancePenaltyFunction penalty_function, double threshold_distance)
    : solvers::Constraint(1, RefFromPtrOrThrow(plant).num_positions(),
                          Vector1d(0), Vector1d(1)),
      plant_{RefFromPtrOrThrow(plant)},
      minimum_distance_{minimum_distance},
      threshold_distance_{threshold_distance},
      penalty_scaling_{1.0/[&penalty_function](double x) {
        double scaling{};
        double dummy{};
        penalty_function(x, &scaling, &dummy);
        return scaling;
      }(-1.0)},
      plant_context_{plant_context},
      penalty_function_{penalty_function} {
  if (!plant_.geometry_source_is_registered()) {
    throw std::invalid_argument(
        "MinimumDistanceConstraint: MultibodyPlant has not registered its "
        "geometry source with SceneGraph yet. Please refer to "
        "AddMultibodyPlantSceneGraph on how to connect MultibodyPlant to "
        "SceneGraph.");
  }
  if (minimum_distance_ < 0) {
    throw std::invalid_argument(
        "MinimumDistanceConstraint: minimum_distance should be non-negative.");
  }
  if (threshold_distance_ <= minimum_distance) {
    throw std::invalid_argument(
        "MinimumDistanceConstraint: threshold_distance should be greater than "
        "minimum_distance.");
  }
  drake::log()->trace("MinimumDistanceConstraint: lower_bound: {}",
                     this->lower_bound().transpose());
  drake::log()->trace("MinimumDistanceConstraint: upper_bound: {}",
                     this->upper_bound().transpose());
}

namespace {
void InitializeY(const Eigen::Ref<const Eigen::VectorXd>&, Eigen::VectorXd* y) {
  (*y)(0) = 0;
}

void InitializeY(const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd* y) {
  (*y) = math::initializeAutoDiffGivenGradientMatrix(
      Vector1d(0), Eigen::RowVectorXd::Zero(x(0).derivatives().size()));
}

void Distance(const MultibodyPlant<double>&, const systems::Context<double>&,
              const geometry::SignedDistancePair<double> signed_distance_pair,
              double* distance) {
  DRAKE_ASSERT(distance != nullptr);
  *distance = signed_distance_pair.distance;
}

void Distance(const MultibodyPlant<double>& plant,
              const systems::Context<double>& plant_context,
              const geometry::SignedDistancePair<double> signed_distance_pair,
              AutoDiffXd* distance) {
  DRAKE_ASSERT(distance != nullptr);
  const auto& query_port = plant.get_geometry_query_input_port();
  if (!query_port.HasValue(plant_context)) {
    throw std::invalid_argument(
        "MinimumDistanceConstraint: Cannot get a valid geometry::QueryObject. "
        "Either the plant geometry_query_input_port() is not properly "
        "connected to the SceneGraph's output port, or the plant_context is "
        "incorrect. Please refer to AddMultibodyPlantSceneGraph on connecting "
        "MultibodyPlant to SceneGraph.");
  }
  Vector3<double> p_WCa, p_WCb;
  const auto& query_object =
      query_port.Eval<geometry::QueryObject<double>>(plant_context);
  const geometry::SceneGraphInspector<double>& inspector =
      query_object.inspector();
  const geometry::FrameId frame_A_id =
      inspector.GetFrameId(signed_distance_pair.id_A);
  const geometry::FrameId frame_B_id =
      inspector.GetFrameId(signed_distance_pair.id_B);
  const Frame<double>& frame_A =
      plant.GetBodyFromFrameId(frame_A_id)->body_frame();
  const Frame<double>& frame_B =
      plant.GetBodyFromFrameId(frame_B_id)->body_frame();
  plant.CalcPointsPositions(
      plant_context, frame_A,
      inspector.X_FG(signed_distance_pair.id_A) * signed_distance_pair.p_ACa,
      plant.world_frame(), &p_WCa);
  plant.CalcPointsPositions(
      plant_context, frame_B,
      inspector.X_FG(signed_distance_pair.id_B) * signed_distance_pair.p_BCb,
      plant.world_frame(), &p_WCb);

  // The distance is d = sign * |p_CbCa_B|, where the
  // closest points are Ca on object A, and Cb on object B.
  // So the gradient ∂d/∂q = p_CbCa_W * ∂p_BCa_B/∂q / d
  // where p_CbCa_W = p_WCa - p_WCb = p_WA + R_WA * p_ACa - (p_WB + R_WB *
  // p_BCb)
  Eigen::Matrix<double, 6, Eigen::Dynamic> Jq_V_BCa_W(6, plant.num_positions());
  plant.CalcJacobianSpatialVelocity(plant_context, JacobianWrtVariable::kQDot,
                                    frame_A, signed_distance_pair.p_ACa, frame_B,
                                    plant.world_frame(), &Jq_V_BCa_W);
  const Eigen::RowVectorXd ddistance_dq =
      (p_WCa - p_WCb).transpose() * Jq_V_BCa_W.bottomRows<3>() / signed_distance_pair.distance;
  AutoDiffVecXd q = plant.GetPositions(plant_context);
  *distance = AutoDiffXd{signed_distance_pair.distance,
                         ddistance_dq * math::autoDiffToGradientMatrix(q)};
}

}  // namespace

double MinimumDistanceConstraint::Penalty(double distance) const {
  double penalty{};
  double dummy{};
  penalty_function_(distance, &penalty, &dummy);
  return penalty;
}

AutoDiffXd MinimumDistanceConstraint::Penalty(AutoDiffXd distance) const {
  double penalty_double{};
  double dpenalty_ddistance{};
  penalty_function_(distance.value(), &penalty_double,
                    &dpenalty_ddistance);
  return math::initializeAutoDiffGivenGradientMatrix(
      Vector1d(penalty_double),
      Vector1d(dpenalty_ddistance) * distance.derivatives())(0);
}

template <typename T>
void MinimumDistanceConstraint::DoEvalGeneric(
    const Eigen::Ref<const VectorX<T>>& x, VectorX<T>* y) const {
  y->resize(1);

  internal::UpdateContextConfiguration(plant_context_, plant_, x);
  const auto& query_port = plant_.get_geometry_query_input_port();
  if (!query_port.HasValue(*plant_context_)) {
    throw std::invalid_argument(
        "MinimumDistanceConstraint: Cannot get a valid geometry::QueryObject. "
        "Either the plant geometry_query_input_port() is not properly "
        "connected to the SceneGraph's output port, or the plant_context_ is "
        "incorrect. Please refer to AddMultibodyPlantSceneGraph on connecting "
        "MultibodyPlant to SceneGraph.");
  }
  const auto& query_object =
      query_port.Eval<geometry::QueryObject<double>>(*plant_context_);

  const std::vector<geometry::SignedDistancePair<double>>
      signed_distance_pairs =
          query_object.ComputeSignedDistancePairwiseClosestPoints(
              threshold_distance_);

  InitializeY(x, y);

  std::vector<T> penalties;
  for (const auto& signed_distance_pair : signed_distance_pairs) {
    if (signed_distance_pair.distance < threshold_distance_) {
      T distance{};
      Distance(plant_, *plant_context_, signed_distance_pair, &distance);
      penalties.push_back(penalty_scaling_ *
                          Penalty((distance - threshold_distance_) /
                                  (threshold_distance_ - minimum_distance_)));
    }
  }
  if (!penalties.empty()) {
    (*y)(0) = SmoothMax(penalties);
  }
  drake::log()->trace("MinimumDistanceConstraint: Value: {}", y->transpose());
}

void MinimumDistanceConstraint::DoEval(
    const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd* y) const {
  DoEvalGeneric(x, y);
}

void MinimumDistanceConstraint::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                                       AutoDiffVecXd* y) const {
  DoEvalGeneric(x, y);
}
}  // namespace multibody
}  // namespace drake
