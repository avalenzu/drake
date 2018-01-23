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
    : robot_(std::move(robot)),
      end_effector_frame_(robot_->findFrame(end_effector_frame_name)) {
  const int num_positions = robot_->get_num_positions();
  const int num_velocities = robot_->get_num_velocities();
  // Input ports
  joint_position_input_port_ =
      this->DeclareInputPort(kVectorValued, num_positions).get_index();
  joint_velocity_input_port_ =
      this->DeclareInputPort(kVectorValued, num_velocities).get_index();
  desired_end_effector_velocity_input_port_ =
      this->DeclareInputPort(kVectorValued, 6).get_index();
  // Ouput ports
  result_output_port_ =
      this->DeclareAbstractOutputPort(
              DifferentialInverseKinematicsResult(),
              &DifferentialInverseKinematicsSystem::CalcResult)
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

void DifferentialInverseKinematicsSystem::CalcResult(
    const systems::Context<double>& context,
    DifferentialInverseKinematicsResult* output) const {
  const VectorX<double>& joint_position =
      this->EvaluateJointPosition(context).get_value();
  const VectorX<double>& joint_velocity =
      this->EvaluateJointVelocity(context).get_value();
  const VectorX<double>& desired_end_effector_velocity =
      this->EvaluateDesiredEndEffectorVelocity(context).get_value();
  *output = DoDifferentialInverseKinematics(
      *robot_, joint_position, joint_velocity, *end_effector_frame_,
      desired_end_effector_velocity, Parameters(context));
}

}  // namespace planner
}  // namespace manipulation
}  // namespace drake
