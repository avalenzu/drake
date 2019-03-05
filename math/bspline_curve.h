#pragma once

#include <memory>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/eigen_types.h"
#include "drake/common/trajectories/trajectory.h"
#include "drake/math/bspline_basis.h"

namespace drake {
namespace math {
template <typename T>
class BsplineCurve final : public drake::trajectories::Trajectory<T> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(BsplineCurve);

  BsplineCurve(const BsplineBasis<double>& basis,
               const std::vector<drake::MatrixX<T>>& control_points);

  template <typename T_input>
  BsplineCurve(const BsplineBasis<double>& basis,
               const std::vector<drake::MatrixX<T_input>>& control_points);

  virtual ~BsplineCurve() = default;

  // Required methods for drake::trajectories::Trajectory interface.
  std::unique_ptr<drake::trajectories::Trajectory<T>> Clone() const override;

  drake::MatrixX<T> value(double time) const override;

  std::unique_ptr<drake::trajectories::Trajectory<T>> MakeDerivative(
      int derivative_order = 1) const override;

  Eigen::Index rows() const override { return control_points()[0].rows(); }

  Eigen::Index cols() const override { return control_points()[0].cols(); }

  double start_time() const { return knots()[order() - 1]; }

  double end_time() const { return knots()[num_control_points()]; }

  // Other methods
  int num_control_points() const { return basis_.num_control_points(); }

  const std::vector<drake::MatrixX<T>>& control_points() const {
    return control_points_;
  }

  drake::MatrixX<T> InitialValue() const;

  drake::MatrixX<T> FinalValue() const;

  const std::vector<double>& knots() const { return basis_.knots(); }

  int order() const { return basis_.order(); }

  int degree() const { return order() - 1; }

  const BsplineBasis<double>& basis() const { return basis_; }

  void InsertKnot(const std::vector<double>& time);

  BsplineCurve<T> Derivative(int derivative_order = 1) const;

  bool operator==(const BsplineCurve<T>& other) const;

  math::BsplineCurve<T> CopyBlock(int start_row, int start_col, int block_rows,
                                  int block_cols) const;

  /** Creates a math::BsplineCurve consisting of the first
   * `n` elements of `original`.
   * @returns the newly created math::BsplineCurve.
   * @pre original.cols() == 1
   */
  math::BsplineCurve<T> CopyHead(int n) const;

 private:
  BsplineBasis<double> basis_;
  std::vector<drake::MatrixX<T>> control_points_;
};
}  // namespace math
}  // namespace drake
