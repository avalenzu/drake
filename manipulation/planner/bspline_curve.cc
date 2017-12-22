#include "drake/manipulation/planner/bspline_curve.h"
#include "drake/common/text_logging.h"

namespace drake {
namespace manipulation {
namespace planner {
BsplineCurve::BsplineCurve(BsplineBasis basis,
                           std::vector<MatrixX<double>> control_points)
    : basis_(basis),
      control_points_(control_points),
      piecwise_polynomial_(basis.ConstructBsplineCurve(control_points)) {}

void BsplineCurve::InsertKnot(double time) {
  // Find the knot before this time.
  std::vector<double> new_knots;
  std::vector<MatrixX<double>> new_control_points;
  DRAKE_THROW_UNLESS(knots().front() <= time && time <= knots().back());
  int i = 0;
  while (knots()[i + degree()] < time) {
    new_knots.push_back(knots()[i]);
    new_control_points.push_back(control_points()[i]);
    ++i;
  }
  int k = i + degree() - 1;
  while (i <= k) {
    double a = (time - knots()[i]) / (knots()[i + degree()] - knots()[i]);
    new_knots.push_back(knots()[i]);
    new_control_points.push_back((1 - a) * (control_points()[i - 1]) +
                                 a * control_points()[i]);
    ++i;
  }
  new_knots.push_back(time);
  new_control_points.push_back(control_points()[i - 1]);
  while(i < control_points().size()) {
    new_knots.push_back(knots()[i]);
    new_control_points.push_back(control_points()[i]);
    ++i;
  }
  while(i < knots().size()) {
    new_knots.push_back(knots()[i]);
    ++i;
  }
  basis_ = BsplineBasis(order(), new_knots);
  control_points_ = new_control_points;
  piecwise_polynomial_ = basis_.ConstructBsplineCurve(control_points_);
}
}  // namespace planner
}  // namespace manipulation
}  // namespace drake
