#pragma once

#include <vector>

#include "drake/common/drake_throw.h"
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

  PiecewisePolynomial<double> ConstructBsplineCurve(const MatrixX<double>& control_points,
                                               int derivative_order = 0) const {
    DRAKE_THROW_UNLESS(control_points.cols() == num_control_points_);
    const int num_y = control_points.rows();
    const int num_internal_intervals = num_control_points_ - order_ + 1;
    std::vector<MatrixX<Polynomial<double>>> polynomials(
        num_internal_intervals);
    for (int i = 0; i < num_internal_intervals; ++i) {
      polynomials[i] =
          MatrixX<Polynomial<double>>::Zero((derivative_order + 1) * num_y, 1);
      for (int j = 0; j < num_y; ++j) {
        // TODO(avalenzu): Only do this for the elements of the basis whose
        // support includes the i-th interval.
        for (int k = 0; k < num_control_points_; ++k) {
          for (int ii = 0; ii <= derivative_order; ++ii) {
            polynomials[i](ii * num_y + j) +=
                basis_[k].derivative(ii).getPolynomial(i, 0, 0) *
                control_points(j, k);
          }
        }
      }
    }
    return PiecewisePolynomial<double>(polynomials,
                                       basis_.front().getSegmentTimes());
  }

 private:
  const int order_;
  const int num_control_points_;
  std::vector<double> knots_;
  std::vector<PiecewisePolynomial<double>> basis_;
};
}  // namespace planner
}  // namespace manipulation
}  // namespace drake
