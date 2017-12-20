#include "drake/manipulation/planner/bspline_basis.h"

namespace drake {
namespace manipulation {
namespace planner {
BsplineBasis::BsplineBasis(int order, int num_control_points)
    : order_(order), num_control_points_(num_control_points) {
  // The knot vector will be [0, 0, 0, ..., 0, t₁
  const int num_knots{num_control_points_ + order_};
  knots_.resize(num_knots, 0.0);
  const double knot_interval =
      1.0 / static_cast<double>(num_control_points_ - (order_ - 1));
  for (int i = order; i < num_knots; ++i) {
    knots_[i] = std::min(1.0, knots_[i - 1] + knot_interval);
  }
  for (int i = 0; i < num_control_points; ++i) {
    basis_.push_back(PiecewisePolynomial<double>::BSpline(i, order_, knots_));
  }
}

PiecewisePolynomial<double> BsplineBasis::ConstructBsplineCurve(
    const MatrixX<double>& control_points, int derivative_order) const {
  DRAKE_THROW_UNLESS(control_points.cols() == num_control_points_);
  const int num_y = control_points.rows();
  const int num_internal_intervals = num_control_points_ - order_ + 1;
  std::vector<MatrixX<Polynomial<double>>> polynomials(num_internal_intervals);
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
}  // namespace planner
}  // namespace manipulation
}  // namespace drake
