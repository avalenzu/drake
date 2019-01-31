#include "drake/math/bspline_basis.h"

#include <algorithm>
#include <set>

#include "drake/common/cond.h"
#include "drake/common/default_scalars.h"
#include "drake/common/text_logging.h"

using drake::cond;
using drake::MatrixX;

namespace drake {
namespace math {
namespace {

template <typename T>
std::vector<T> ConstructDefaultKnots(int order, int num_control_points,
                                     KnotVectorType type) {
  const int num_knots{num_control_points + order};
  std::vector<T> knots(num_knots, 0.0);
  switch (type) {
    case KnotVectorType::kClampedUniform: {
      const T knot_interval =
          1.0 / static_cast<double>(num_control_points - (order - 1));
      for (int i = order; i < num_knots; ++i) {
        if (i < num_control_points) {
          knots.at(i) = knots.at(i - 1) + knot_interval;
        } else {
          knots.at(i) = 1.0;
        }
      }
      break;
    }
    case KnotVectorType::kUniform: {
      for (int i = 0; i < num_knots; ++i) {
        knots.at(i) = static_cast<T>(i - (order - 1));
      }
    }
  }
  return knots;
}
}  // namespace

template <typename T>
BsplineBasis<T>::BsplineBasis(int order, std::vector<T> knots)
    : order_(order), num_control_points_(knots.size() - order), knots_(knots) {}

template <typename T>
BsplineBasis<T>::BsplineBasis(int order, int num_control_points,
                              KnotVectorType type)
    : BsplineBasis<T>(
          order, ConstructDefaultKnots<T>(order, num_control_points, type)) {}

template <typename T>
boolean<T> BsplineBasis<T>::IsControlPointActive(
    int control_point_index, const std::array<T, 2>& interval) const {
  DRAKE_ASSERT(knots_.at(order() - 1) <= interval.front());
  DRAKE_ASSERT(interval.back() <= knots_.at(num_control_points()));
  // If t ∈ [tᵣ, tᵣ₊₁), the only control points that contribute to the value of
  // the curve at t are P[r - p], ..., P[r - 1], P[r], where p = order - 1 is
  // the degree of the basis functions. We want to know which control points are
  // active over the interval [tₛ,  tₑ]. If we find the corresponding rₛ and
  // rₑ, then the active control points for the interval are
  // P[rₛ - p], ..., P[rₑ]
  //
  // Reference:
  // http://web.mit.edu/hyperbook/Patrikalakis-Maekawa-Cho/node17.html

  // Define short-hand references to match Patricalakis et al.:
  const std::vector<T>& tt = knots();
  const int& p = order() - 1;
  const T& t_s = interval[0];
  const T& t_e = interval[1];
  const int& r = control_point_index;

  return tt[r + p + 1] >= t_s && tt[r] <= t_e;
}

template <typename T>
boolean<T> BsplineBasis<T>::IsControlPointActive(int control_point_index,
                                              const T& parameter_value) const {
  return IsControlPointActive(control_point_index,
                              {{parameter_value, parameter_value}});
}

template <typename T>
std::vector<int> BsplineBasis<T>::ComputeActiveControlPointIndices(
    const std::array<T, 2>& plan_interval) const {
  std::vector<int> active_control_point_indices{};
  for (int i = 0; i < num_control_points(); ++i) {
    if (IsControlPointActive(i, plan_interval)) {
      active_control_point_indices.push_back(i);
    }
  }
  return active_control_point_indices;
}

template <typename T>
std::vector<int> BsplineBasis<T>::ComputeActiveControlPointIndices(
    const T& parameter_value) const {
  return ComputeActiveControlPointIndices({{parameter_value, parameter_value}});
}

template <typename T>
bool BsplineBasis<T>::operator==(const BsplineBasis<T>& other) const {
  return this->order() == other.order() && this->knots() == other.knots();
}

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class BsplineBasis)

}  // namespace math
}  // namespace drake
