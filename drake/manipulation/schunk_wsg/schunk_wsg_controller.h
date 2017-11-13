#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/diagram.h"

namespace drake {
namespace manipulation {
namespace schunk_wsg {

/// This class implements a controller for a Schunk WSG gripper.  It
/// has two input ports which receive lcmt_schunk_wsg_command messages
/// and the current state, and an output port which emits the target
/// force for the actuated finger.  The internal implementation
/// consists of a PID controller (which controls the target position
/// from the command message) combined with a saturation block (which
/// applies the force control from the command message).
template <typename T>
class SchunkWsgController : public systems::Diagram<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SchunkWsgController)
  SchunkWsgController();

  const systems::InputPortDescriptor<T>& get_gripper_force_input_port()
      const {
    return this->get_input_port(gripper_force_input_port_);
  }

  const systems::InputPortDescriptor<T>& get_state_input_port() const {
    return this->get_input_port(state_input_port_);
  }

  /// Returns the output port on which the sum is presented.
  const systems::OutputPort<T>& get_output_port() const {
    return systems::Diagram<T>::get_output_port(0);
  }

 private:
  int gripper_force_input_port_{};
  int state_input_port_{};
};

}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake
