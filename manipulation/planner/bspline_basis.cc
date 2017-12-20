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
    basis_.push_back(
        PiecewisePolynomial<double>::BSpline(i, order_, knots_));
  }
}
}  // namespace planner
}  // namespace manipulation
}  // namespace drake
