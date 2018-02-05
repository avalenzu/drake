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
    const std::string& end_effector_frame_name, double dt)
    : robot_(std::move(robot)), dt_(dt) {
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
  this->DeclareDiscreteState(num_positions + num_velocities);
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
  parameters.set_timestep(dt_);
  parameters_index_ = this->DeclareAbstractParameter(
      systems::Value<DifferentialInverseKinematicsParameters>(parameters));
  this->DeclarePeriodicDiscreteUpdate(dt_);
}

void DifferentialInverseKinematicsSystem::DoCalcDiscreteVariableUpdates(
    const systems::Context<double>& context,
    const std::vector<const systems::DiscreteUpdateEvent<double>*>& events,
    systems::DiscreteValues<double>* discrete_state) const {
  LeafSystem<double>::DoCalcDiscreteVariableUpdates(context, events,
                                                    discrete_state);
  BasicVector<double>& joint_state = discrete_state->get_mutable_vector();
  const VectorX<double>& joint_position =
      joint_state.get_value().head(num_positions());
  const VectorX<double>& joint_velocity =
      joint_state.get_value().tail(num_velocities());
  const VectorX<double>& constraint_velocity =
      this->EvaluateConstraintVelocity(context);
  const MatrixX<double>& constraint_jacobian =
      this->EvaluateConstraintJacobian(context);
  KinematicsCache<double> cache = robot_->doKinematics(joint_position);
  DifferentialInverseKinematicsResult result = DoDifferentialInverseKinematics(
      joint_position, joint_velocity, constraint_velocity, constraint_jacobian,
      Parameters(context));
  VectorX<double> new_joint_state{num_positions() + num_velocities()};
  new_joint_state.tail(num_velocities()) = result.joint_velocities.value_or(
      VectorX<double>::Zero(joint_position.size()));
  if (joint_position.size() == joint_velocity.size()) {
    new_joint_state.head(num_positions()) =
        joint_position + dt_ * new_joint_state.tail(num_velocities());
  } else {
    new_joint_state.head(num_positions()) =
        joint_position +
        dt_ *
            robot_->transformVelocityToQDot(
                cache, new_joint_state.tail(num_velocities()));
  }
  drake::log()->debug("x_new = {}", new_joint_state.transpose());
  joint_state.SetFromVector(new_joint_state);
}

void DifferentialInverseKinematicsSystem::Initialize(
    const VectorX<double>& q0, systems::Context<double>* context) const {
  context->get_mutable_discrete_state().get_mutable_vector().SetFromVector(
      (VectorX<double>(num_positions() + num_velocities()) << q0,
       VectorX<double>::Zero(num_velocities()))
          .finished());
  ;
  context->get_mutable_abstract_state<bool>(is_initialized_state_) = true;
}

void DifferentialInverseKinematicsSystem::CopyDesiredJointPosition(
    const systems::Context<double>& context,
    BasicVector<double>* output) const {
  DRAKE_THROW_UNLESS(context.get_abstract_state<bool>(is_initialized_state_));
  output->SetFromVector(
      context.get_discrete_state_vector().get_value().head(num_positions()));
}
}  // namespace planner
}  // namespace manipulation
}  // namespace drake
