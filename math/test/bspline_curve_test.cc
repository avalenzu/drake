#include "drake/math/bspline_curve.h"

#include <gtest/gtest.h>

#include "drake/common/symbolic.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"

using drake::CompareMatrices;
using drake::MatrixX;
using drake::VectorX;
using drake::symbolic::Expression;
using drake::symbolic::Variable;

namespace drake {
namespace math {
// Verifies that the constructors work as expected.
GTEST_TEST(BSplineBasisTests, ConstructorTest) {
  const int expected_order = 4;
  const int expected_num_control_points = 11;
  const std::vector<double> expected_knots{
      0, 0, 0, 0, 0.125, 0.25, 0.375, 0.5, 0.625, 0.75, 0.875, 1, 1, 1, 1};
  const int expected_rows = 2;
  const int expected_cols = 1;
  const double expected_start_time = 0;
  const double expected_end_time = 1;
  std::vector<MatrixX<double>> expected_control_points{};
  for (int i = 0; i < expected_num_control_points; ++i) {
    expected_control_points.push_back(
        i * MatrixX<double>::Ones(expected_rows, expected_cols));
  }
  BsplineBasis<double> bspline_basis_1{expected_order, expected_knots};
  BsplineCurve<double> curve{bspline_basis_1, expected_control_points};

  // Verify method return values.
  EXPECT_EQ(curve.rows(), expected_rows);
  EXPECT_EQ(curve.cols(), expected_cols);
  EXPECT_EQ(curve.start_time(), expected_start_time);
  EXPECT_EQ(curve.end_time(), expected_end_time);
  EXPECT_EQ(curve.control_points(), expected_control_points);
}

// Verifies that value() works as expected.
GTEST_TEST(BSplineBasisTests, ValueTest) {
  const int expected_order = 4;
  const int expected_num_control_points = 11;
  const std::vector<double> expected_knots{
      0, 0, 0, 0, 0.125, 0.25, 0.375, 0.5, 0.625, 0.75, 0.875, 1, 1, 1, 1};
  const int expected_rows = 2;
  const int expected_cols = 1;
  std::vector<MatrixX<double>> expected_control_points{};
  for (int i = 0; i < expected_num_control_points; ++i) {
    expected_control_points.push_back(
        i * MatrixX<double>::Ones(expected_rows, expected_cols));
  }
  BsplineBasis<double> bspline_basis_1{expected_order, expected_knots};
  BsplineCurve<double> curve{bspline_basis_1, expected_control_points};

  // Verify that value() returns the expected results.
  ASSERT_TRUE(CompareMatrices(expected_control_points.back(), curve.value(1)));
  EXPECT_TRUE(CompareMatrices(expected_control_points.front(), curve.value(0)));
  const int num_times = 100;
  VectorX<double> t = VectorX<double>::LinSpaced(num_times, 0, 1);
  for (int k = 0; k < num_times; ++k) {
    MatrixX<double> value = curve.value(t(k));
    for (int i = 0; i < expected_rows; ++i) {
      for (int j = 0; j < expected_cols; ++j) {
        EXPECT_EQ(value(i, j), value(0, 0));
      }
    }
  }
}

}  // namespace math
}  // namespace drake
