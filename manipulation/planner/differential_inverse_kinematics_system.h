
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
 *  - v_desired (vector)
 *  - status (abstract, DifferentialInverseKinematicsStatus)
 * Numeric Parameters:
 *  - q_nominal (vector)
 *  - q_lower_bound (vector)
 *  - q_upper_bound (vector)
 *  - v_lower_bound (vector)
 *  - v_upper_bound (vector)
 *  - vd_lower_bound (vector)
 *  - vd_upper_bound (vector)
 *  - gain_E (vector)
 *  - dt (scalar)
 *  - unconstrained_dof_v_limit (scalar)
 * Abstract Parameters:
 *  - robot (RigidBodyTree)
 *  - frame_E (RigidBodyFrame)
 */
class DifferentialInverseKinematicsSystem
    : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DifferentialInverseKinematicsSystem);

  DifferentialInverseKinematicsSystem(
      std::unique_ptr<RigidBodyTree<double>> robot,
      const std::string& end_effector_frame_name);

  const systems::InputPortDescriptor<double>& joint_position_input_port()
      const {
    return get_input_port(joint_position_input_port_);
  }

  const systems::InputPortDescriptor<double>& joint_velocity_input_port()
      const {
    return get_input_port(joint_velocity_input_port_);
  }

  const systems::InputPortDescriptor<double>&
  desired_end_effector_velocity_input_port() const {
    return get_input_port(desired_end_effector_velocity_input_port_);
  }

  const systems::OutputPort<double>& desired_joint_velocity_output_port()
      const {
    return get_output_port(desired_joint_velocity_output_port_);
  }

  void CopyDesiredJointVelocityOutputFromState(
      const systems::Context<double>& context,
      systems::BasicVector<double>* output) const;

  void CopyStatusOutputFromState(
      const systems::Context<double>& context,
      DifferentialInverseKinematicsStatus* output) const;

 private:
  // Input port indices
  int joint_position_input_port_{-1};
  int joint_velocity_input_port_{-1};
  int desired_end_effector_velocity_input_port_{-1};
  // Output port indices
  int desired_joint_velocity_output_port_{-1};
  int status_output_port_{-1};
  // State
  int desired_joint_velocity_state_index_{-1};
  int status_state_index_{-1};
  // Numerical parameter indices
  int q_nominal_index_{-1};
  int q_lower_bound_index_{-1};
  int q_upper_bound_index_{-1};
  int v_lower_bound_index_{-1};
  int v_upper_bound_index_{-1};
  int vd_lower_bound_index_{-1};
  int vd_upper_bound_index_{-1};
  int gain_E_index_{-1};
  int dt_index_{-1};
  int unconstrained_dof_v_limit_index_{-1};
  // Abstract parameter indices
  int robot_index{-1};
  int frame_E_index{-10};
};
}  // namespace planner
}  // namespace manipulation
}  // namespace drake
