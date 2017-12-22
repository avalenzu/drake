#include "drake/manipulation/planner/bspline_basis.h"

#include <vector>

namespace drake {
namespace manipulation {
namespace planner {
class BsplineCurve {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(BsplineCurve);
  BsplineCurve(BsplineBasis basis, std::vector<MatrixX<double>> control_points);
  const std::vector<MatrixX<double>>& control_points() const {
    return control_points_;
  }
  const PiecewisePolynomial<double>& piecwise_polynomial() const {
    return piecwise_polynomial_;
  }
  const MatrixX<double> value(double time) const {
    return piecwise_polynomial_.value(time);
  }
  const std::vector<double>& knots() const { return basis_.knots(); }
  int order() const { return basis_.order(); }
  int degree() const { return order() - 1; }
  void InsertKnot(double time);

 private:
  BsplineBasis basis_;
  std::vector<MatrixX<double>> control_points_;
  PiecewisePolynomial<double> piecwise_polynomial_;
};
}  // namespace planner
}  // namespace manipulation
}  // namespace drake
