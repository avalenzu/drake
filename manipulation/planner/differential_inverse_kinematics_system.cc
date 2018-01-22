#include "drake/manipulation/planner/differential_inverse_kinematics_system.h"

namespace drake {
namespace manipulation {
namespace planner {

using systems::BasicVector;
using systems::kVectorValued;
using systems::Parameters;

DifferentialInverseKinematicsSystem::DifferentialInverseKinematicsSystem(
    std::unique_ptr<RigidBodyTree<double>> robot,
    const std::string& end_effector_frame_name) {
  const int num_positions = robot->get_num_positions();
  const int num_velocities = robot->get_num_velocities();
  // Input ports
  joint_position_input_port_ =
      DeclareInputPort(kVectorValued, num_positions).get_index();
  joint_velocity_input_port_ =
      DeclareInputPort(kVectorValued, num_velocities).get_index();
  desired_end_effector_velocity_input_port_ =
      DeclareInputPort(kVectorValued, 6).get_index();
  // Ouput ports
  desired_joint_velocity_output_port_ =
      DeclareVectorOutputPort(
          BasicVector<double>(num_velocities),
          &DifferentialInverseKinematicsSystem::CopyDesiredJointVelocityOutputFromState)
          .get_index();
  status_output_port_ =
      DeclareAbstractOutputPort(
          DifferentialInverseKinematicsStatus::kNoSolutionFound,
          &DifferentialInverseKinematicsSystem::CopyStatusOutputFromState)
          .get_index();
}

}  // namespace planner
}  // namespace manipulation
}  // namespace drake
