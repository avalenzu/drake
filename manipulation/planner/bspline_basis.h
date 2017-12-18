#pragma once

#include <array>
#include <vector>

#include "drake/common/drake_throw.h"
#include "drake/common/symbolic.h"
#include "drake/common/trajectories/piecewise_polynomial.h"

namespace drake {
namespace manipulation {
namespace planner {
class BsplineBasis {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(BsplineBasis);
  BsplineBasis(int order, std::vector<double> knots);

  BsplineBasis(int order, int num_control_points);

  int order() const { return order_; }

  int num_control_points() const { return num_control_points_; }

  const std::vector<double>& knots() const { return knots_; }

  const std::vector<trajectories::PiecewisePolynomial<double>> polynomials() const {
    return basis_;
  }

  trajectories::PiecewisePolynomial<double> ConstructBsplineCurve(
      const std::vector<MatrixX<double>>& control_points) const;

  MatrixX<symbolic::Expression> ConstructExpressionForCurveValue(
      const std::vector<MatrixX<symbolic::Expression>>& control_points,
      double time) const;

  std::vector<int> ComputeActiveControlPointIndices(
      std::array<double, 2> evaluation_time) const;

  std::vector<int> ComputeActiveControlPointIndices(
      double evaluation_time) const;

 private:
  int order_;
  int num_control_points_;
  std::vector<double> knots_;
  std::vector<trajectories::PiecewisePolynomial<double>> basis_;
  // TODO(avalenzu): Replace usage of this member with
  // PiecewisePolynomial<double>::kEpsilonTime. That ought to work, but it was
  // giving me linker errors.
  double kEpsilonTime_{1e-10};
};
}  // namespace planner
}  // namespace manipulation
}  // namespace drake
