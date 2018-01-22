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

enum class DifferentialInverseKinematicsStatus {
  kSolutionFound,
  kNoSolutionFound,
  kStuck
};

struct DifferentialInverseKinematicsResult {
  optional<VectorX<double>> joint_velocities{};
  DifferentialInverseKinematicsStatus status{
      DifferentialInverseKinematicsStatus::kNoSolutionFound};
};

DifferentialInverseKinematicsResult DoDifferentialInverseKinematics(
    const RigidBodyTree<double>& robot, const KinematicsCache<double>& cache,
    const RigidBodyFrame<double>& frame_E, const Vector6<double>& V_WE,
    double dt, const VectorX<double> q_nominal, const VectorX<double>& v_last,
    const std::pair<VectorX<double>, VectorX<double>>& q_bounds,
    const optional<std::pair<VectorX<double>, VectorX<double>>>& v_bounds,
    const optional<std::pair<VectorX<double>, VectorX<double>>>& vd_bounds,
    double unconstrained_dof_v_limit,
    const Vector6<double>& gain_E = Vector6<double>::Constant(1));

DifferentialInverseKinematicsResult DoDifferentialInverseKinematics(
    const VectorX<double> q_current, const VectorX<double>& v_current,
    const Vector6<double>& V, const MatrixX<double>& J,
    const VectorX<double> q_nominal,
    const std::pair<VectorX<double>, VectorX<double>>& q_bounds,
    const optional<std::pair<VectorX<double>, VectorX<double>>>& v_bounds,
    const optional<std::pair<VectorX<double>, VectorX<double>>>& vd_bounds,
    double dt, double unconstrained_dof_v_limit, const Vector6<double>& gain_E);

class DifferentialInverseKinematics {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DifferentialInverseKinematics);

  DifferentialInverseKinematics(std::unique_ptr<RigidBodyTree<double>> robot,
                                const std::string& end_effector_frame_name);

  void set_unconstrained_degrees_of_freedom_velocity_limit(double limit) {
    unconstrained_dof_v_limit_ = limit;
  }
  const Vector6<double>& end_effector_velocity_gain() const { return gain_E_; }

  void SetEndEffectorVelocityGain(const Vector6<double>& gain_E);

  const std::pair<VectorX<double>, VectorX<double>> joint_position_limits()
      const {
    return q_bounds_;
  }

  void SetJointPositionLimits(
      const std::pair<VectorX<double>, VectorX<double>>& q_bounds);

  const optional<std::pair<VectorX<double>, VectorX<double>>>&
  joint_velocity_limits() const {
    return v_bounds_;
  }

  void SetJointVelocityLimits(
      const std::pair<VectorX<double>, VectorX<double>>& v_bounds);

  const optional<std::pair<VectorX<double>, VectorX<double>>>&
  joint_acceleration_limits() const {
    return vd_bounds_;
  }

  void SetJointAccelerationLimits(
      const std::pair<VectorX<double>, VectorX<double>>& v_bounds);

  void set_current_joint_position(VectorX<double> q_current) {
    DRAKE_ASSERT(q_current.size() == robot_->get_num_positions());
    q_current_ = q_current;
  }

  void set_current_joint_velocity(VectorX<double> v_current) {
    DRAKE_ASSERT(v_current.size() == robot_->get_num_velocities());
    v_current_ = v_current;
  }

  const Vector6<double>& desired_end_effector_velocity() {
    return V_WE_desired_;
  }

  void set_desired_end_effector_velocity(
      const Vector6<double>& desired_end_effector_velocity) {
    V_WE_desired_ = desired_end_effector_velocity;
  }

  double timestep() const { return dt_; }

  void set_timestep(double dt) {
    DRAKE_ASSERT(dt > 0);
    dt_ = dt;
  }

  DifferentialInverseKinematicsResult Solve() const;

  DifferentialInverseKinematicsResult ComputeJointVelocities(
      const VectorX<double>& q, const VectorX<double>& v_last,
      const Vector6<double>& V_WE, double dt);

 private:
  copyable_unique_ptr<RigidBodyTree<double>> robot_{};
  std::shared_ptr<RigidBodyFrame<double>> frame_E_{};
  VectorX<double> q_nominal_;
  std::pair<VectorX<double>, VectorX<double>> q_bounds_{};
  optional<std::pair<VectorX<double>, VectorX<double>>> v_bounds_{};
  optional<std::pair<VectorX<double>, VectorX<double>>> vd_bounds_{};
  double unconstrained_dof_v_limit_{std::numeric_limits<double>::infinity()};
  Vector6<double> gain_E_{Vector6<double>::Ones()};
  VectorX<double> q_current_;
  VectorX<double> v_current_;
  Vector6<double> V_WE_desired_;
  double dt_{1};
};

}  // namespace planner
}  // namespace manipulation
}  // namespace drake
