
#pragma once

#include "drake/manipulation/planner/differential_inverse_kinematics.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_context.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace manipulation {
namespace planner {
/**
 Inputs:
  - q_current (vector)
  - v_current (vector)
  - X_WE_desired (vector)
 State:
  - q_desired
  - is_initialized
 Outputs:
  - q_desired
 Abstract Parameters:
  - parameters (DifferentialInverseKinematicsParameters)
 Member variables:
  - robot (RigidBodyTree)
  - frame_E (RigidBodyFrame)


                      ┌────────────┐
               q ────▶│            │
                      │Differential│
               v ────▶│Inverse     ├────▶ q_desired
                      │Kinematics  │
    X_WE_desired ────▶│            │
                      └────────────┘



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
  desired_end_effector_pose_input_port() const {
    return this->get_input_port(desired_end_effector_pose_input_port_);
  }

  const systems::OutputPort<double>& desired_joint_position_output_port()
      const {
    return this->get_output_port(desired_joint_position_output_port_);
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

  const Isometry3<double>& EvaluateDesiredEndEffectorPose(
      const systems::Context<double>& context) const {
    const systems::AbstractValue * desired_end_effector_pose =
        this->EvalAbstractInput(context,
                              desired_end_effector_pose_input_port_);
    DRAKE_THROW_UNLESS(desired_end_effector_pose);
    return desired_end_effector_pose->GetValue<Isometry3<double>>();
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

  /**
   * Sets the initial state of the controller to @p initial_joint_position.
   * This function needs to be explicitly called before any simulation.
   * Otherwise this aborts in CalcOutput().
   */
  void Initialize(const VectorX<double>& q0,
                  systems::Context<double>* context) const;

 protected:
  virtual void DoCalcTimeDerivatives(
      const systems::Context<double>& context,
      systems::ContinuousState<double>* derivatives) const override;

 private:
  void CopyDesiredJointPosition(const systems::Context<double>& context,
                                systems::BasicVector<double>* output) const;

  // Input port indices
  int joint_position_input_port_{-1};
  int joint_velocity_input_port_{-1};
  int desired_end_effector_pose_input_port_{-1};
  // State indices
  int is_initialized_state_{-1};
  // Output port indices
  int desired_joint_position_output_port_{-1};
  // Abstract parameter
  int parameters_index_{-1};

  std::unique_ptr<RigidBodyTree<double>> robot_{};
  std::shared_ptr<RigidBodyFrame<double>> end_effector_frame_{};
};
}  // namespace planner
}  // namespace manipulation
}  // namespace drake
