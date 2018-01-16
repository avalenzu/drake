#pragma once

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace manipulation {
namespace planner {

optional<VectorX<double>> DoDifferentialInverseKinematics(
    const RigidBodyTree<double>& robot, const KinematicsCache<double>& cache,
    const RigidBodyFrame<double>& frame_E, const Vector6<double>& V_WE,
    double dt, const VectorX<double> q_nominal, const VectorX<double>& v_last,
    const std::pair<VectorX<double>, VectorX<double>>& q_bounds,
    const optional<std::pair<VectorX<double>, VectorX<double>>>& v_bounds,
    const optional<std::pair<VectorX<double>, VectorX<double>>>& vd_bounds,
    double unconstrained_dof_v_limit,
    const Vector6<double>& gain_E = Vector6<double>::Constant(1));

class DifferentialInverseKinematics {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DifferentialInverseKinematics);

  DifferentialInverseKinematics(std::unique_ptr<RigidBodyTree<double>> robot,
                                const std::string& end_effector_frame_name);

  void set_unconstrained_degrees_of_freedom_velocity_limit(double limit) {
    unconstrained_dof_v_limit_ = limit;
  }

  void SetEndEffectorVelocityGain(const Vector6<double>& gain_E); 

  void SetJointVelocityLimits(
      const std::pair<VectorX<double>, VectorX<double>>& v_bounds);

  void SetJointAccelerationLimits(
      const std::pair<VectorX<double>, VectorX<double>>& v_bounds);

  optional<VectorX<double>> ComputeJointVelocities(
      const VectorX<double>& q, const VectorX<double>& v_last,
      const Vector6<double>& V_WE, double dt) const;

 private:
  copyable_unique_ptr<RigidBodyTree<double>> robot_{};
  std::shared_ptr<RigidBodyFrame<double>> frame_E_{};
  VectorX<double> q_nominal_;
  std::pair<VectorX<double>, VectorX<double>> q_bounds_{};
  optional<std::pair<VectorX<double>, VectorX<double>>> v_bounds_{};
  optional<std::pair<VectorX<double>, VectorX<double>>> vd_bounds_{};
  double unconstrained_dof_v_limit_{std::numeric_limits<double>::infinity()};
  Vector6<double> gain_E_{Vector6<double>::Ones()};
};

}  // namespace planner
}  // namespace manipulation
}  // namespace drake
