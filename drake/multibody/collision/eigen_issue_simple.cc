#include <iostream>
#include <Eigen/Dense>

/**
 * This is a highly distilled version of fcl::detail::boxBox2 from 
 * <fcl root>/include/fcl/narrowphase/detail/primitive_shape_algorithm/box-box-inl.h.
 * I have been unable to reduce it further and still reproduce the issue.
 */
double reproFunction(const Eigen::Matrix3d& R_in)
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

int main() {
  Eigen::Matrix3d R;
  R = Eigen::Matrix3d::Zero(3,3); 

  // This fails - reproFunction(R) returns 0
  R(1, 2) = 0.7;
  double R12 = reproFunction(R);
  bool are_they_equal = (R12 == R(1,2));
  std::cout << "R12 == R(1,2):" << are_they_equal << std::endl;
}

