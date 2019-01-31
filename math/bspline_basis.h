#pragma once

#include <array>
#include <vector>

#include "drake/common/drake_bool.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/math/knot_vector_type.h"

namespace drake {
namespace math {

template <typename T>
class BsplineBasis {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(BsplineBasis);
  BsplineBasis(int order, std::vector<T> knots);

  BsplineBasis(int order, int num_control_points,
               KnotVectorType type = KnotVectorType::kClampedUniform);

  int order() const { return order_; }

  int num_control_points() const { return num_control_points_; }

  const std::vector<T>& knots() const { return knots_; }

  boolean<T> IsControlPointActive(
      int control_point_index, const std::array<T, 2>& parameter_value) const;

  boolean<T> IsControlPointActive(int control_point_index,
                                      const T& parameter_value) const;

  std::vector<int> ComputeActiveControlPointIndices(
      const std::array<T, 2>& plan_interval) const;

  std::vector<int> ComputeActiveControlPointIndices(
      const T& plan_interval) const;

  bool operator==(const BsplineBasis& other) const;

 private:
  int order_;
  int num_control_points_;
  std::vector<T> knots_;
};
}  // namespace math
}  // namespace drake
