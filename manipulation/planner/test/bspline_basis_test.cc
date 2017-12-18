#include "drake/manipulation/planner/bspline_basis.h"

#include <gtest/gtest.h>

namespace drake {
namespace manipulation {
namespace planner {
namespace {
GTEST_TEST(BSplineBasisTests, ConstructorTest) {
  const int expected_order = 4;
  const int expected_num_control_points = 11;
  const std::vector<double> expected_knots{
      0, 0, 0, 0, 0.125, 0.25, 0.375, 0.5, 0.625, 0.75, 0.875, 1, 1, 1, 1};
  BsplineBasis bspline_basis{expected_order, expected_num_control_points};
  EXPECT_EQ(bspline_basis.order(), expected_order);
  EXPECT_EQ(bspline_basis.num_control_points(), expected_num_control_points);
  EXPECT_EQ(bspline_basis.knots(), expected_knots);
}
}  // namespace
}  // namespace planner
}  // namespace manipulation
}  // namespace drake
