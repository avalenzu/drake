#include "drake/manipulation/planner/bspline_basis.h"

namespace drake {
namespace manipulation {
namespace planner {
namespace {
std::vector<double> ConstructDefaultKnots(int order, int num_control_points) {
  const int num_knots{num_control_points + order};
  std::vector<double> knots(num_knots, 0.0);
  const double knot_interval =
      1.0 / static_cast<double>(num_control_points - (order - 1));
  for (int i = order; i < num_knots; ++i) {
    knots[i] = std::min(1.0, knots[i - 1] + knot_interval);
  }
  return knots;
}
}  // namespace

BsplineBasis::BsplineBasis(int order, std::vector<double> knots)
    : order_(order), num_control_points_(knots.size() - order), knots_(knots) {
  DRAKE_THROW_UNLESS(std::is_sorted(knots.begin(), knots.end()));
  for (int i = 0; i < num_control_points_; ++i) {
    basis_.push_back(PiecewisePolynomial<double>::BSpline(i, order_, knots_));
  }
}

BsplineBasis::BsplineBasis(int order, int num_control_points)
    : BsplineBasis(order, ConstructDefaultKnots(order, num_control_points)) {}

PiecewisePolynomial<double> BsplineBasis::ConstructBsplineCurve(
    const std::vector<MatrixX<double>>& control_points) const {
  DRAKE_THROW_UNLESS(control_points.size() == num_control_points_);
  const int control_point_rows = control_points.front().rows();
  const int control_point_cols = control_points.front().cols();
  const int num_segments = basis_.front().getNumberOfSegments();
  std::vector<MatrixX<Polynomial<double>>> polynomials(num_segments);
  for (int segment_index = 0; segment_index < num_segments; ++segment_index) {
    polynomials[segment_index] = MatrixX<Polynomial<double>>::Zero(
        control_point_rows, control_point_cols);
    // TODO(avalenzu): Only do this for the elements of the basis whose
    // support includes the segment_index-th interval.
    for (int control_point_index = 0; control_point_index < num_control_points_;
         ++control_point_index) {
      DRAKE_THROW_UNLESS(control_points[control_point_index].rows() ==
                         control_point_rows);
      DRAKE_THROW_UNLESS(control_points[control_point_index].cols() ==
                         control_point_cols);
      for (int i = 0; i < control_point_rows; ++i) {
        for (int j = 0; j < control_point_cols; ++j) {
          polynomials[segment_index](i, j) +=
              basis_[control_point_index].getPolynomial(segment_index, 0, 0) *
              control_points[control_point_index](i, j);
        }
      }
    }
  }
  return PiecewisePolynomial<double>(polynomials,
                                     basis_.front().getSegmentTimes());
}

MatrixX<symbolic::Expression> BsplineBasis::ConstructExpressionForCurveValue(
    const std::vector<MatrixX<symbolic::Variable>>& control_points, double time,
    int derivative_order) const {
  // Compute the basis values at evaluation_time
  VectorX<double> basis_function_values(order_);
  const std::vector<int> active_control_point_indices =
      ComputeActiveControlPointIndices(time);
  MatrixX<symbolic::Expression> ret{control_points.front().rows(),
                                    control_points.front().cols()};
  for (int i = 0; i < order_; ++i) {
    ret += basis_[active_control_point_indices[i]].value(time)(0) *
           control_points[active_control_point_indices[i]];
  }
  return ret;
}

std::vector<int> BsplineBasis::ComputeActiveControlPointIndices(
    double time) const {
  std::vector<int> active_control_point_indices(order_);
  for (int i = 0; i < num_control_points_; ++i) {
    if (knots_[i] <= time + kEpsilonTime_ &&
        time <= knots_[i + order_] + kEpsilonTime_) {
      active_control_point_indices[0] = i;
      break;
    }
  }
  for (int i = 1; i < order_; ++i) {
    active_control_point_indices[i] = active_control_point_indices[i - 1] + 1;
  }
  return active_control_point_indices;
}

}  // namespace planner
}  // namespace manipulation
}  // namespace drake
