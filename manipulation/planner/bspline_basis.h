#pragma once

#include <vector>

#include "drake/common/drake_throw.h"
#include "drake/common/symbolic.h"
#include "drake/common/trajectories/piecewise_polynomial.h"

namespace drake {
namespace manipulation {
namespace planner {
class BsplineBasis {
 public:
  BsplineBasis(int order, int num_control_points);

  int order() const { return order_; }

  int num_control_points() const { return num_control_points_; }

  const std::vector<double>& knots() const { return knots_; }

  const std::vector<PiecewisePolynomial<double>> polynomials() const {
    return basis_;
  }

  PiecewisePolynomial<double> ConstructBsplineCurve(
      const MatrixX<double>& control_points, int derivative_order = 0) const;

 private:
  const int order_;
  const int num_control_points_;
  std::vector<double> knots_;
  std::vector<PiecewisePolynomial<double>> basis_;
};
}  // namespace planner
}  // namespace manipulation
}  // namespace drake
