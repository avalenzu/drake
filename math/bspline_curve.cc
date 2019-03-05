#include "drake/math/bspline_curve.h"

#include <algorithm>

#include "drake/common/symbolic.h"
#include "drake/common/text_logging.h"

using drake::MatrixX;
using drake::symbolic::Expression;
using drake::symbolic::Variable;
using drake::trajectories::Trajectory;

namespace drake {
namespace math {
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

template <typename T_parameter>
T_parameter DeBoorAlpha(const BsplineBasis<T_parameter>& basis, int i, int j,
                        const T_parameter& parameter_value) {
  const int& k = basis.order();
  const std::vector<T_parameter>& tt = basis.knots();
  const T_parameter& t = parameter_value;
  drake::log()->trace("tt.at(i = {}) = {}", i, tt.at(i));
  drake::log()->trace("tt.at(i+k-j = {}) = {}", i + k + j, tt.at(i + k - j));
  return (t - tt.at(i)) / (tt.at(i + k - j) - tt.at(i));
}

template <typename T_control_point>
MatrixX<T_control_point> NewDeBoorPoint(
    const BsplineBasis<double>& basis,
    const std::vector<MatrixX<T_control_point>>& control_points, int i, int j,
    double parameter_value) {
  drake::log()->trace("i = {}, j = {}", i, j);
  const std::vector<MatrixX<T_control_point>>& pp = control_points;
  if (j == 0) {
    return pp.at(i);
  } else {
    double alpha = DeBoorAlpha(basis, i, j, parameter_value);
    drake::log()->trace("alpha = {}", alpha);
    MatrixX<T_control_point> A = NewDeBoorPoint(basis, control_points, i - 1, j - 1,
                                                parameter_value);
    MatrixX<T_control_point> B = NewDeBoorPoint(basis, control_points, i, j - 1, parameter_value);
    return (1.0 - alpha) * A + alpha * B;
  }
}

}  // namespace

template <typename T>
BsplineCurve<T>::BsplineCurve(const BsplineBasis<double>& basis,
                              const std::vector<MatrixX<T>>& control_points)
    : basis_(basis), control_points_(control_points) {}

template <typename T>
template <typename T_input>
BsplineCurve<T>::BsplineCurve(
    const BsplineBasis<double>& basis,
    const std::vector<MatrixX<T_input>>& control_points)
    : BsplineCurve(basis, CastControlPoints<T>(control_points)) {}

template <typename T>
MatrixX<T> BsplineCurve<T>::value(double time) const {
  int ell = order() - 1;
  for (; ell < num_control_points() - 1; ++ell) {
    if (knots()[ell + 1] >= time) break;
  }
  return NewDeBoorPoint(basis(), control_points(), ell, order() - 1, time);
}

template <typename T>
std::unique_ptr<Trajectory<T>> BsplineCurve<T>::Clone() const {
  return std::make_unique<BsplineCurve<T>>(*this);
}

template <typename T>
std::unique_ptr<Trajectory<T>> BsplineCurve<T>::MakeDerivative(
    int derivative_order) const {
  return Derivative(derivative_order).Clone();
}

template <typename T>
MatrixX<T> BsplineCurve<T>::InitialValue() const {
  return value(start_time());
}

template <typename T>
void BsplineCurve<T>::InsertKnot(
    const std::vector<double>& additional_knots) {
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
  basis_ = BsplineBasis<double>(order(), knots);
}

template <typename T>
BsplineCurve<T> BsplineCurve<T>::Derivative(int derivative_order) const {
  if (derivative_order == 0) {
    return *this;
  } else if (derivative_order > 1) {
    return Derivative(1).Derivative(derivative_order - 1);
  } else {
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
    return BsplineCurve(BsplineBasis<double>(order() - 1, derivative_knots),
                        derivative_control_points);
  }
}

template <typename T>
bool BsplineCurve<T>::operator==(const BsplineCurve<T>& other) const {
  return this->basis() == other.basis() &&
         this->control_points() == other.control_points();
}

template <typename T>
math::BsplineCurve<T> BsplineCurve<T>::CopyBlock(int start_row, int start_col,
                                                 int block_rows,
                                                 int block_cols) const {
  std::vector<drake::MatrixX<T>> new_control_points{};
  new_control_points.reserve(num_control_points());
  std::transform(
      control_points().begin(), control_points().end(),
      std::back_inserter(new_control_points),
      [&](const drake::MatrixX<T>& control_point) -> drake::MatrixX<T> {
        return control_point.block(start_row, start_col, block_rows,
                                   block_cols);
      });
  return {basis(), new_control_points};
}

/// Creates a math::BsplineCurve consisting of the first
/// `n` elements of `original`.
/// @returns the newly created math::BsplineCurve.
/// @pre original.cols() == 1
template <typename T>
math::BsplineCurve<T> BsplineCurve<T>::CopyHead(int n) const {
  DRAKE_ASSERT(cols() == 1);
  return CopyBlock(0, 0, n, 1);
}

template class BsplineCurve<double>;
template class BsplineCurve<Expression>;
template BsplineCurve<Expression>::BsplineCurve(
    const BsplineBasis<double>& basis, const std::vector<MatrixX<Variable>>&);
}  // namespace math
}  // namespace drake
