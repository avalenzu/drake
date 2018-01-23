
#pragma once

#include "drake/manipulation/planner/differential_inverse_kinematics.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_context.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace manipulation {
namespace planner {
/**
 * Inputs:
 *  - q_current (vector)
 *  - v_current (vector)
 *  - V_WE_desired (vector)
 * Outputs:
 *  - result (DifferentialInverseKinematicsResult)
 * Numeric Parameters:
 *  - q_nominal (vector)
 *  - joint_position_lower_bound (vector)
 *  - joint_position_upper_bound (vector)
 *  - joint_velocity_lower_bound (vector)
 *  - joint_velocity_upper_bound (vector)
 *  - joint_acceleration_lower_bound (vector)
 *  - joint_acceleration_upper_bound (vector)
 *  - gain_E (vector)
 *  - dt (scalar)
 *  - unconstrained_dof_v_limit (scalar)
 * Abstract Parameters:
 *  - robot (RigidBodyTree)
 *  - frame_E (RigidBodyFrame)
 */
template <typename T>
class DifferentialInverseKinematicsSystem
    : public drake::systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DifferentialInverseKinematicsSystem);

  DifferentialInverseKinematicsSystem(
      std::unique_ptr<RigidBodyTree<T>> robot,
      const std::string& end_effector_frame_name);

  const systems::InputPortDescriptor<T>& joint_position_input_port() const {
    return this->get_input_port(joint_position_input_port_);
  }

  const systems::InputPortDescriptor<T>& joint_velocity_input_port() const {
    return this->get_input_port(joint_velocity_input_port_);
  }

  const systems::InputPortDescriptor<T>&
  desired_end_effector_velocity_input_port() const {
    return this->get_input_port(desired_end_effector_velocity_input_port_);
  }

  const systems::OutputPort<T>& result_output_port() const {
    return this->get_output_port(result_output_port_);
  }

  const systems::BasicVector<T>& nominal_joint_position(
      const systems::Context<T>& context) const {
    return context.get_numeric_parameter(nominal_joint_position_index_);
  }

  const systems::BasicVector<T>& joint_position_lower_bound(
      const systems::Context<T>& context) const {
    return context.get_numeric_parameter(joint_position_lower_bound_index_);
  }

  const systems::BasicVector<T>& joint_position_upper_bound(
      const systems::Context<T>& context) const {
    return context.get_numeric_parameter(joint_position_upper_bound_index_);
  }

  const systems::BasicVector<T>& joint_velocity_lower_bound(
      const systems::Context<T>& context) const {
    return context.get_numeric_parameter(joint_velocity_lower_bound_index_);
  }

  const systems::BasicVector<T>& joint_velocity_upper_bound(
      const systems::Context<T>& context) const {
    return context.get_numeric_parameter(joint_velocity_upper_bound_index_);
  }

  const systems::BasicVector<T>& joint_acceleration_lower_bound(
      const systems::Context<T>& context) const {
    return context.get_numeric_parameter(joint_acceleration_lower_bound_index_);
  }

  const systems::BasicVector<T>& joint_acceleration_upper_bound(
      const systems::Context<T>& context) const {
    return context.get_numeric_parameter(joint_acceleration_upper_bound_index_);
  }

  const systems::BasicVector<T>& end_effector_velocity_gain(
      const systems::Context<T>& context) const {
    return context.get_numeric_parameter(end_effector_velocity_gain_index_);
  }

  const T& Timestep(const systems::Context<T>& context) const {
    return context.get_numeric_parameter(timestep_index_).GetAtIndex(0);
  }

  const RigidBodyTree<T>& robot() const {
    return *robot_;
  }

  const RigidBodyFrame<T>& end_effector_frame() const {
    return *end_effector_frame_;
  }

  void CalcResult(const systems::Context<T>& context,
                  DifferentialInverseKinematicsResult* output) const;

 private:
  // Input port indices
  int joint_position_input_port_{-1};
  int joint_velocity_input_port_{-1};
  int desired_end_effector_velocity_input_port_{-1};
  // Output port indices
  int result_output_port_{-1};
  // Numerical parameter indices
  int nominal_joint_position_index_{-1};
  int joint_position_lower_bound_index_{-1};
  int joint_position_upper_bound_index_{-1};
  int joint_velocity_lower_bound_index_{-1};
  int joint_velocity_upper_bound_index_{-1};
  int joint_acceleration_lower_bound_index_{-1};
  int joint_acceleration_upper_bound_index_{-1};
  int end_effector_velocity_gain_index_{-1};
  int timestep_index_{-1};
  int unconstrained_degrees_of_freedom_velocity_limit_index_{-1};
  
  std::unique_ptr<RigidBodyTree<double>> robot_{};
  std::shared_ptr<RigidBodyFrame<double>> end_effector_frame_{};
};
}  // namespace planner
}  // namespace manipulation
}  // namespace drake
