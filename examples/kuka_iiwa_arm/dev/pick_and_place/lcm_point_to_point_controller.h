#pragma once

#include <string>

#include "drake/examples/kuka_iiwa_arm/pick_and_place/pick_and_place_configuration.h"
#include "drake/manipulation/planner/differential_inverse_kinematics.h"
#include "drake/manipulation/planner/differential_inverse_kinematics_system.h"
#include "drake/systems/framework/diagram.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
class LcmPointToPointController : public systems::Diagram<double> {
 public:
  LcmPointToPointController(
      const pick_and_place::PlannerConfiguration& configuration);

  const systems::InputPortDescriptor<double>& iiwa_status_input_port() const {
    return get_input_port(iiwa_status_input_port_);
  }

  const systems::InputPortDescriptor<double>&
  plan_input_port() const {
    return get_input_port(plan_input_port_);
  }

  const systems::OutputPort<double>& iiwa_command_output_port() const {
    return get_output_port(iiwa_command_output_port_);
  }

  /**
   * Sets the initial state of the controller to @p initial_joint_position.
   * This function needs to be explicitly called before any simulation.
   * Otherwise this aborts in CalcOutput().
   */
  void Initialize(const VectorX<double>& initial_joint_position,
                  systems::Context<double>* context) const;

  int num_joints() const {
    return differential_inverse_kinematics_->num_positions();
  }

  manipulation::planner::DifferentialInverseKinematicsParameters&
  MutableParameters(systems::Context<double>* context) const {
    return differential_inverse_kinematics_->MutableParameters(
        &this->GetMutableSubsystemContext(*differential_inverse_kinematics_,
                                          context));
  }

 private:
  // Input ports.
  int iiwa_status_input_port_{-1};
  int plan_input_port_{-1};

  // Ouptut ports.
  int iiwa_command_output_port_{-1};

  // Subsystems.
  manipulation::planner::DifferentialInverseKinematicsSystem*
      differential_inverse_kinematics_{};
};
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
