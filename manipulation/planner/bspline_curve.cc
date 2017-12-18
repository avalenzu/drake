#include "drake/manipulation/planner/bspline_curve.h"
#include "drake/common/text_logging.h"

namespace drake {
namespace manipulation {
namespace planner {
namespace {
template <typename T, typename T_input>
std::vector<MatrixX<T>> CastControlPoints(
    const std::vector<MatrixX<T_input>>& control_points) {
  std::vector<MatrixX<T>> control_points_cast;
  std::transform(control_points.begin(), control_points.end(),
                 std::back_inserter(control_points_cast),
                 [](const MatrixX<T_input>& var) -> MatrixX<T> {
                   return var.template cast<T>();
                 });
  return control_points_cast;
}
}  // namespace

template <>
void BsplineCurve<double>::UpdatePiecewisePolynomial() {
  piecwise_polynomial_ = basis_.ConstructBsplineCurve(control_points_);
}

template <typename T>
void BsplineCurve<T>::UpdatePiecewisePolynomial() {
  // Piecewise polynomial doesn't support symbolic::{Expression,Variable}, so do
  // nothing here.
}

template <typename T>
BsplineCurve<T>::BsplineCurve(const BsplineBasis& basis,
                              const std::vector<MatrixX<T>>& control_points)
    : basis_(basis), control_points_(control_points) {
  UpdatePiecewisePolynomial();
}

template <typename T>
template <typename T_input>
BsplineCurve<T>::BsplineCurve(
    const BsplineBasis& basis,
    const std::vector<MatrixX<T_input>>& control_points)
    : BsplineCurve(basis, CastControlPoints<T>(control_points)) {}

template <>
const MatrixX<double> BsplineCurve<double>::value(double time) const {
  return piecwise_polynomial_->value(time);
}

template <>
const MatrixX<symbolic::Expression> BsplineCurve<symbolic::Expression>::value(
    double time) const {
  return basis_.ConstructExpressionForCurveValue(control_points(), time);
}

template <>
void BsplineCurve<double>::InsertKnot(const std::vector<double>& additional_knots) {
  auto knots = this->knots();
  for (const auto& time : additional_knots) {
    DRAKE_THROW_UNLESS(knots.front() <= time && time <= knots.back());
    int i = 0;
    while (knots[i + degree()] < time) {
      ++i;
    }
    int k = i + degree() - 1;
    auto control_point_i_minus_1 = control_points()[i - 1];
    while (i <= k) {
      double a = (time - knots[i]) / (knots[i + degree()] - knots[i]);
      auto new_control_point{(1 - a) * control_point_i_minus_1 +
                             a * control_points()[i]};
      control_point_i_minus_1 = control_points()[i];
      control_points_[i] = new_control_point;
      ++i;
    }
    knots.insert(std::next(knots.begin(), i), time);
    control_points_.insert(std::next(control_points_.begin(), i),
                           control_point_i_minus_1);
  }
  basis_ = BsplineBasis(order(), knots);
  UpdatePiecewisePolynomial();
}

template <typename T>
void BsplineCurve<T>::InsertKnot(const std::vector<double>& time) {
  // TODO(avalenzu): Figure out the right way to handle this. This method is
  // only for BsplineCurve<double>.
  DRAKE_THROW_UNLESS(false);
}

template <typename T>
BsplineCurve<T> BsplineCurve<T>::Derivative() const {
  std::vector<MatrixX<T>> derivative_control_points;
  std::vector<double> derivative_knots;
  const int num_derivative_knots = knots().size() - 1;
  for (int i = 1; i < num_derivative_knots; ++i) {
    derivative_knots.push_back(knots()[i]);
  }
  for (int i = 0; i < num_control_points() - 1; ++i) {
    derivative_control_points.push_back(
        degree() / (knots()[i + order()] - knots()[i + 1]) *
        (control_points()[i + 1] - control_points()[i]));
  }
  return BsplineCurve(BsplineBasis(order() - 1, derivative_knots),
                      derivative_control_points);
}

template class BsplineCurve<double>;
template class BsplineCurve<symbolic::Expression>;
template BsplineCurve<symbolic::Expression>::BsplineCurve(
    const BsplineBasis& basis, const std::vector<MatrixX<symbolic::Variable>>&);
}  // namespace planner
}  // namespace manipulation
}  // namespace drake
