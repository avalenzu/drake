#pragma once

#include "drake/examples/kuka_iiwa_arm/iiwa_world/iiwa_wsg_diagram_factory.h"
#include "drake/examples/kuka_iiwa_arm/pick_and_place/pick_and_place_configuration.h"
#include "drake/systems/framework/diagram.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace monolithic_pick_and_place {
class PickAndPlacePlant : public systems::Diagram<double> {
 public:
  PickAndPlacePlant(
      const pick_and_place::SimulatedPlantConfiguration& plant_configuration,
      const pick_and_place::OptitrackConfiguration& optitrack_configuration);

  const systems::InputPortDescriptor<double>& get_input_port_iiwa_command(
      int index) const {
    return this->get_input_port(input_port_iiwa_command_.at(index));
  }

  const systems::InputPortDescriptor<double>& get_input_port_wsg_command(
      int index) const {
    return this->get_input_port(input_port_wsg_command_.at(index));
  }

  const systems::OutputPort<double>& get_output_port_iiwa_status(
      int index) const {
    return this->get_output_port(output_port_iiwa_status_.at(index));
  }

  const systems::OutputPort<double>& get_output_port_iiwa_robot_state(
      int index) const {
    return this->get_output_port(output_port_iiwa_robot_state_.at(index));
  }

  const systems::OutputPort<double>& get_output_port_wsg_status(
      int index) const {
    return this->get_output_port(output_port_wsg_status_.at(index));
  }

  const systems::OutputPort<double>& get_output_port_optitrack_frame() const {
    return this->get_output_port(output_port_optitrack_frame_);
  }

 private:
  std::vector<int> input_port_iiwa_command_;
  std::vector<int> input_port_wsg_command_;
  std::vector<int> output_port_iiwa_status_;
  std::vector<int> output_port_wsg_status_;
  std::vector<int> output_port_iiwa_robot_state_;
  int output_port_optitrack_frame_{-1};
};
}  // namespace monolithic_pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
