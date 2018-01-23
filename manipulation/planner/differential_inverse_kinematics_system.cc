#include "drake/manipulation/planner/differential_inverse_kinematics_system.h"

#include <limits>

namespace drake {
namespace manipulation {
namespace planner {

using systems::BasicVector;
using systems::kVectorValued;
using systems::Parameters;

template <typename T>
DifferentialInverseKinematicsSystem<T>::DifferentialInverseKinematicsSystem(
    std::unique_ptr<RigidBodyTree<T>> robot,
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
  nominal_joint_position_index_ = this->DeclareNumericParameter(
      BasicVector<T>(robot_->getZeroConfiguration()));

  nominal_joint_position_index_ = this->DeclareNumericParameter(
      BasicVector<T>(robot_->getZeroConfiguration()));

  joint_position_lower_bound_index_ =
      this->DeclareNumericParameter(BasicVector<T>(robot_->joint_limit_min));

  joint_position_upper_bound_index_ =
      this->DeclareNumericParameter(BasicVector<T>(robot_->joint_limit_max));

  joint_velocity_lower_bound_index_ =
      this->DeclareNumericParameter(BasicVector<T>(VectorX<T>::Constant(
          num_velocities, -std::numeric_limits<T>::infinity())));

  joint_velocity_upper_bound_index_ =
      this->DeclareNumericParameter(BasicVector<T>(VectorX<T>::Constant(
          num_velocities, std::numeric_limits<T>::infinity())));

  joint_acceleration_lower_bound_index_ =
      this->DeclareNumericParameter(BasicVector<T>(VectorX<T>::Constant(
          num_velocities, -std::numeric_limits<T>::infinity())));

  joint_acceleration_upper_bound_index_ =
      this->DeclareNumericParameter(BasicVector<T>(VectorX<T>::Constant(
          num_velocities, std::numeric_limits<T>::infinity())));

  end_effector_velocity_gain_index_ =
      this->DeclareNumericParameter(BasicVector<T>(VectorX<T>::Ones(6)));

  timestep_index_ =
      this->DeclareNumericParameter(BasicVector<T>(VectorX<T>::Ones(1)));

  unconstrained_degrees_of_freedom_velocity_limit_index_ =
      this->DeclareNumericParameter(BasicVector<T>(
          VectorX<T>::Constant(1, std::numeric_limits<T>::infinity())));
}

template <typename T>
void DifferentialInverseKinematicsSystem<T>::CalcResult(
    const systems::Context<T>& context,
    DifferentialInverseKinematicsResult* output) const {
  // VectorX<T>* q_current =
  // context.EvalVectorInput(context, joint_position_input_port_);
}

// Explicitly instantiates on the most common scalar types.
template class DifferentialInverseKinematicsSystem<double>;

}  // namespace planner
}  // namespace manipulation
}  // namespace drake
