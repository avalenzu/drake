#include "drake/manipulation/planner/differential_inverse_kinematics_system.h"

#include <limits>

namespace drake {
namespace manipulation {
namespace planner {

using systems::BasicVector;
using systems::kVectorValued;
using systems::Parameters;

DifferentialInverseKinematicsSystem::DifferentialInverseKinematicsSystem(
    std::unique_ptr<RigidBodyTree<double>> robot,
    const std::string& end_effector_frame_name)
    : robot_(std::move(robot)) {
  end_effector_frame_ = robot_->findFrame(end_effector_frame_name);
  const int num_positions = robot_->get_num_positions();
  const int num_velocities = robot_->get_num_velocities();
  // Input ports
  joint_position_input_port_ =
      this->DeclareInputPort(kVectorValued, num_positions).get_index();
  joint_velocity_input_port_ =
      this->DeclareInputPort(kVectorValued, num_velocities).get_index();
  constraint_input_port_ = this->DeclareAbstractInputPort().get_index();
  // State
  this->DeclareContinuousState(num_positions);
  is_initialized_state_ =
      this->DeclareAbstractState(systems::Value<bool>::Make(false));
  // Ouput ports
  desired_joint_position_output_port_ =
      this->DeclareVectorOutputPort(
              BasicVector<double>(robot_->getZeroConfiguration()),
              &DifferentialInverseKinematicsSystem::CopyDesiredJointPosition)
          .get_index();
  // Parameters
  const VectorX<double> infinity_vector_num_velocities =
      VectorX<double>::Constant(num_velocities,
                                -std::numeric_limits<double>::infinity());
  DifferentialInverseKinematicsParameters parameters{
      robot_->get_num_positions(), robot_->get_num_velocities()};
  parameters.SetJointPositionLimits(
      {robot_->joint_limit_min, robot_->joint_limit_max});
  parameters_index_ = this->DeclareAbstractParameter(
      systems::Value<DifferentialInverseKinematicsParameters>(parameters));
}

void DifferentialInverseKinematicsSystem::DoCalcTimeDerivatives(
    const systems::Context<double>& context,
    systems::ContinuousState<double>* derivatives) const {
  const VectorX<double>& joint_position =
      this->EvaluateJointPosition(context).get_value();
  const VectorX<double>& joint_velocity =
      this->EvaluateJointVelocity(context).get_value();
  const VectorX<double>& constraint_velocity =
      this->EvaluateConstraintVelocity(context);
  const MatrixX<double>& constraint_jacobian =
      this->EvaluateConstraintJacobian(context);
  KinematicsCache<double> cache = robot_->doKinematics(joint_position);
  DifferentialInverseKinematicsResult result = DoDifferentialInverseKinematics(
      joint_position, joint_velocity, constraint_velocity, constraint_jacobian,
      Parameters(context));
  if (joint_position.size() == joint_velocity.size()) {
    derivatives->SetFromVector(result.joint_velocities.value_or(
        VectorX<double>::Zero(joint_position.size())));
  } else {
    derivatives->SetFromVector(robot_->transformVelocityToQDot(
        cache,
        result.joint_velocities.value_or(
            VectorX<double>::Zero(joint_position.size()))));
  }
}

void DifferentialInverseKinematicsSystem::Initialize(
    const VectorX<double>& q0, systems::Context<double>* context) const {
  context->get_mutable_continuous_state().SetFromVector(q0);
  context->get_mutable_abstract_state<bool>(is_initialized_state_) = true;
}

void DifferentialInverseKinematicsSystem::CopyDesiredJointPosition(
    const systems::Context<double>& context,
    BasicVector<double>* output) const {
  DRAKE_THROW_UNLESS(context.get_abstract_state<bool>(is_initialized_state_));
  output->SetFrom(context.get_continuous_state().get_vector());
}
}  // namespace planner
}  // namespace manipulation
}  // namespace drake
