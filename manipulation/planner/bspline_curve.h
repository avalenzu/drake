#include "drake/manipulation/planner/bspline_basis.h"

#include <memory>
#include <vector>

#include "drake/common/drake_optional.h"

namespace drake {
namespace manipulation {
namespace planner {
template <typename T>
class BsplineCurve {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(BsplineCurve);

  BsplineCurve(const BsplineBasis& basis,
               const std::vector<MatrixX<T>>& control_points);

  template <typename T_input>
  BsplineCurve(const BsplineBasis& basis,
               const std::vector<MatrixX<T_input>>& control_points);

  const std::vector<MatrixX<T>>& control_points() const {
    return control_points_;
  }

  int num_control_points() const { return basis_.num_control_points(); };

  const MatrixX<T> value(double time) const;

  const std::vector<double>& knots() const { return basis_.knots(); }

  int order() const { return basis_.order(); }

  int degree() const { return order() - 1; }

  void InsertKnot(double time);

  BsplineCurve<T> Derivative() const;

 private:
  void UpdatePiecewisePolynomial();
  BsplineBasis basis_;
  std::vector<MatrixX<T>> control_points_;
  optional<PiecewisePolynomial<double>> piecwise_polynomial_{};
};
}  // namespace planner
}  // namespace manipulation
}  // namespace drake
