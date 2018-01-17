#pragma once

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
 *  - status (enum)
 */
class DifferentialInverseKinematicsSystem
    : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DifferentialInverseKinematicsSystem);

  
  DifferentialInverseKinematicsSystem(std::unique_ptr<RigidBodyTree<double>> robot,
                                const std::string& end_effector_frame_name);

  const systems::InputPortDescriptor<double>& get_joint_position_input_port()
      const {
    return get_input_port(joint_position_input_port_);
  }

  const systems::InputPortDescriptor<double>& get_joint_velocity_input_port()
      const {
    return get_input_port(joint_velocity_input_port_);
  }

  const systems::InputPortDescriptor<double>&
  get_desired_end_effector_velocity_input_port() const {
    return get_input_port(desired_end_effector_velocity_input_port_);
  }

  const systems::OutputPort<double>&
  get_desired_joint_velocity_output_port() const {
    return get_output_port(desired_joint_velocity_output_port_);
  }

 private:
  int joint_position_input_port_{-1};
  int joint_velocity_input_port_{-1};
  int desired_end_effector_velocity_input_port_{-1};
  int desired_joint_velocity_output_port_{-1};
  int status_output_port_{-1};
};
}  // namespace planner
}  // namespace manipulation
}  // namespace drake
