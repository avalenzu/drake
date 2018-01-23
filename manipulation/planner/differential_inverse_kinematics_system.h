
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
class DifferentialInverseKinematicsSystem final
    : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DifferentialInverseKinematicsSystem);

  DifferentialInverseKinematicsSystem(
      std::unique_ptr<RigidBodyTree<double>> robot,
      const std::string& end_effector_frame_name);

  const systems::InputPortDescriptor<double>& joint_position_input_port()
      const {
    return this->get_input_port(joint_position_input_port_);
  }

  const systems::InputPortDescriptor<double>& joint_velocity_input_port()
      const {
    return this->get_input_port(joint_velocity_input_port_);
  }

  const systems::InputPortDescriptor<double>&
  desired_end_effector_velocity_input_port() const {
    return this->get_input_port(desired_end_effector_velocity_input_port_);
  }

  const systems::OutputPort<double>& result_output_port() const {
    return this->get_output_port(result_output_port_);
  }

  const systems::BasicVector<double>& EvaluateJointPosition(
      const systems::Context<double>& context) const {
    const systems::BasicVector<double>* joint_position =
        this->EvalVectorInput(context, joint_position_input_port_);
    DRAKE_THROW_UNLESS(joint_position);
    return *joint_position;
  }

  const systems::BasicVector<double>& EvaluateJointVelocity(
      const systems::Context<double>& context) const {
    const systems::BasicVector<double>* joint_velocity =
        this->EvalVectorInput(context, joint_velocity_input_port_);
    DRAKE_THROW_UNLESS(joint_velocity);
    return *joint_velocity;
  }

  const systems::BasicVector<double>& EvaluateDesiredEndEffectorVelocity(
      const systems::Context<double>& context) const {
    const systems::BasicVector<double>* desired_end_effector_velocity =
        this->EvalVectorInput(context,
                              desired_end_effector_velocity_input_port_);
    DRAKE_THROW_UNLESS(desired_end_effector_velocity);
    return *desired_end_effector_velocity;
  }

  const DifferentialInverseKinematicsParameters& Parameters(
      const systems::Context<double>& context) const {
    return context.get_abstract_parameter(parameters_index_)
        .GetValue<DifferentialInverseKinematicsParameters>();
  }

  const VectorX<double>& nominal_joint_position(
      const systems::Context<double>& context) const {
    return Parameters(context).nominal_joint_position();
  }

  const optional<std::pair<VectorX<double>, VectorX<double>>>&
  JointPositionLimits(const systems::Context<double>& context) const {
    return Parameters(context).joint_position_limits();
  }

  const optional<std::pair<VectorX<double>, VectorX<double>>>&
  JointVelocityLimits(const systems::Context<double>& context) const {
    return Parameters(context).joint_velocity_limits();
  }

  const optional<std::pair<VectorX<double>, VectorX<double>>>&
  JointAccelerationLimits(const systems::Context<double>& context) const {
    return Parameters(context).joint_acceleration_limits();
  }

  const Vector6<double>& EndEffectorVelocityGain(
      const systems::Context<double>& context) const {
    return Parameters(context).end_effector_velocity_gain();
  }

  const double& Timestep(const systems::Context<double>& context) const {
    return Parameters(context).timestep();
  }

  const double& UnconstrainedDegreesOfFreedomVelocityLimit(
      const systems::Context<double>& context) const {
    return Parameters(context)
        .unconstrained_degrees_of_freedom_velocity_limit();
  }

  const RigidBodyTree<double>& robot() const { return *robot_; }

  const RigidBodyFrame<double>& end_effector_frame() const {
    return *end_effector_frame_;
  }

  void CalcResult(const systems::Context<double>& context,
                  DifferentialInverseKinematicsResult* output) const;

 private:
  // Input port indices
  int joint_position_input_port_{-1};
  int joint_velocity_input_port_{-1};
  int desired_end_effector_velocity_input_port_{-1};
  // Output port indices
  int result_output_port_{-1};
  // Abstract parameter
  int parameters_index_{-1};

  std::unique_ptr<RigidBodyTree<double>> robot_{};
  std::shared_ptr<RigidBodyFrame<double>> end_effector_frame_{};
};
}  // namespace planner
}  // namespace manipulation
}  // namespace drake
