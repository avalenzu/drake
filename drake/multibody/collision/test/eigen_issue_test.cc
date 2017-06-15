#include <Eigen/Dense>
#include <gtest/gtest.h>

namespace drake {
namespace {

/**
 * This is a highly distilled version of fcl::detail::boxBox2 from 
 * <fcl root>/include/fcl/narrowphase/detail/primitive_shape_algorithm/box-box-inl.h.
 * I have been unable to reduce it further and still reproduce the issue.
 */
double reproFunction(Eigen::Matrix3d R_in)
{
  const Eigen::Matrix3d R = R_in;

  Eigen::Matrix3d Q = R.cwiseAbs();
  Q(0,0) += 0;

  if(R(1,2) < 2) {
    Eigen::Vector3d n{0, 1, R(1, 2)};
    double s2 = R(1,2);
    s2 /= n.norm();
  }
  return R(1, 2);
}

GTEST_TEST(EigenIssue, reproCase) {
  Eigen::Matrix3d R{Eigen::Matrix3d::Zero()}; 

  // This succeeds, as expected
  EXPECT_EQ(reproFunction(R), R(1,2));

  // This fails - reproFunction(R) returns 0
  R(1, 2) = 0.5;
  EXPECT_EQ(reproFunction(R), R(1,2));

  // This fails - reproFunction(R) returns garbage (1.9036e+185 on my machine)
  R(1, 2) = 0.7;
  EXPECT_EQ(reproFunction(R), R(1,2));

  // This fails - reproFunction(R) returns garbage (1.9036e+185 on my machine)
  R(1, 2) = -0.7;
  EXPECT_EQ(reproFunction(R), R(1,2));
}

}  // namespace
}  // namespace drake
