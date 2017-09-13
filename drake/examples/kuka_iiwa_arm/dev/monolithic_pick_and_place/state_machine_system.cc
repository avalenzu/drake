#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/state_machine_system.h"

#include <utility>
#include <vector>

#include "bot_core/robot_state_t.hpp"
#include "drake/multibody/parsers/urdf_parser.h"
#include "robotlocomotion/robot_plan_t.hpp"

using bot_core::robot_state_t;

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {
/* index of iiwastate */
const int kStateIndex = 0;

robotlocomotion::robot_plan_t MakeDefaultIiwaPlan() {
  robotlocomotion::robot_plan_t default_plan{};
  default_plan.utime = 0;
  default_plan.num_states = 0;
  return default_plan;
}

lcmt_schunk_wsg_command MakeDefaultWsgCommand() {
  lcmt_schunk_wsg_command default_command{};
  default_command.utime = 0;
  default_command.target_position_mm = 110;  // maximum aperture
  default_command.force = 0;
  return default_command;
}
}  // namespace

using pick_and_place::PickAndPlaceStateMachine;

namespace monolithic_pick_and_place {

struct PickAndPlaceStateMachineSystem::InternalState {
  InternalState(const std::string& iiwa_model_path,
                const std::string& end_effector_name, int num_tables,
                const Vector3<double>& object_dimensions,
                double collision_avoidance_threshold)
      : world_state(iiwa_model_path, end_effector_name, num_tables,
                    object_dimensions),
        state_machine(iiwa_model_path, num_tables > 1),
        last_iiwa_plan(MakeDefaultIiwaPlan()),
        last_wsg_command(MakeDefaultWsgCommand()) {
    state_machine.set_collision_avoidance_threshold(
        collision_avoidance_threshold);
  }

  ~InternalState() {}

  pick_and_place::WorldState world_state;
  PickAndPlaceStateMachine state_machine;
  robotlocomotion::robot_plan_t last_iiwa_plan;
  lcmt_schunk_wsg_command last_wsg_command;
};

PickAndPlaceStateMachineSystem::PickAndPlaceStateMachineSystem(
    const std::string& iiwa_model_path, const std::string& end_effector_name,
    const Isometry3<double>& iiwa_base, int num_tables,
    const Vector3<double>& box_dimensions, double collision_avoidance_threshold,
    bool ignore_tall_tables, const double period_sec)
    : iiwa_model_path_(iiwa_model_path),
      end_effector_name_(end_effector_name),
      iiwa_base_(iiwa_base),
      num_tables_(num_tables),
      box_dimensions_(box_dimensions),
      collision_avoidance_threshold_(collision_avoidance_threshold),
      ignore_tall_tables_(ignore_tall_tables) {
  input_port_iiwa_state_ = this->DeclareAbstractInputPort().get_index();
  input_port_box_state_ = this->DeclareAbstractInputPort().get_index();
  input_port_wsg_status_ = this->DeclareAbstractInputPort().get_index();
  input_port_table_state_.resize(num_tables_);
  for (int i = 0; i < num_tables_; ++i) {
    input_port_table_state_[i] = this->DeclareAbstractInputPort().get_index();
  }

  output_port_iiwa_plan_ =
      this->DeclareAbstractOutputPort(
              MakeDefaultIiwaPlan(),
              &PickAndPlaceStateMachineSystem::CalcIiwaPlan)
          .get_index();

  output_port_wsg_command_ =
      this->DeclareAbstractOutputPort(
              MakeDefaultWsgCommand(),
              &PickAndPlaceStateMachineSystem::CalcWsgCommand)
          .get_index();

  this->DeclarePeriodicUnrestrictedUpdate(period_sec, 0);

  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      iiwa_model_path_, multibody::joints::kFixed, &iiwa_tree_);
}

std::unique_ptr<systems::AbstractValues>
PickAndPlaceStateMachineSystem::AllocateAbstractState() const {
  std::vector<std::unique_ptr<systems::AbstractValue>> abstract_vals;
  abstract_vals.push_back(
      std::unique_ptr<systems::AbstractValue>(new systems::Value<InternalState>(
          InternalState(iiwa_model_path_, end_effector_name_, num_tables_,
                        box_dimensions_, collision_avoidance_threshold_))));
  return std::make_unique<systems::AbstractValues>(std::move(abstract_vals));
}

void PickAndPlaceStateMachineSystem::SetDefaultState(
    const systems::Context<double>&,
    systems::State<double>* state) const {
  InternalState& internal_state =
      state->get_mutable_abstract_state<InternalState>(kStateIndex);
  internal_state =
      InternalState(iiwa_model_path_, end_effector_name_, num_tables_,
                    box_dimensions_, collision_avoidance_threshold_);
}

void PickAndPlaceStateMachineSystem::CalcIiwaPlan(
    const systems::Context<double>& context,
    robotlocomotion::robot_plan_t* iiwa_plan) const {
  /* Call actions based on state machine logic */
  const InternalState& internal_state =
      context.get_abstract_state<InternalState>(0);
  *iiwa_plan = internal_state.last_iiwa_plan;
}


void PickAndPlaceStateMachineSystem::CalcWsgCommand(
    const systems::Context<double>& context,
    lcmt_schunk_wsg_command* wsg_command) const {
  /* Call actions based on state machine logic */
  const InternalState& internal_state =
      context.get_abstract_state<InternalState>(0);
  *wsg_command = internal_state.last_wsg_command;
}

void PickAndPlaceStateMachineSystem::DoCalcUnrestrictedUpdate(
    const systems::Context<double>& context,
    const std::vector<const systems::UnrestrictedUpdateEvent<double>*>&,
    systems::State<double>* state) const {
  // Extract Internal state.
  InternalState& internal_state =
      state->get_mutable_abstract_state<InternalState>(kStateIndex);

  /* Update world state from inputs. */
  const robot_state_t& iiwa_state =
      this->EvalAbstractInput(context, input_port_iiwa_state_)
          ->GetValue<robot_state_t>();
  const robot_state_t& box_state =
      this->EvalAbstractInput(context, input_port_box_state_)
          ->GetValue<robot_state_t>();
  const lcmt_schunk_wsg_status& wsg_status =
      this->EvalAbstractInput(context, input_port_wsg_status_)
          ->GetValue<lcmt_schunk_wsg_status>();

  internal_state.world_state.HandleIiwaStatus(iiwa_state);
  internal_state.world_state.HandleWsgStatus(wsg_status);
  internal_state.world_state.HandleObjectStatus(box_state);
  for (int i = 0; i < num_tables_; ++i) {
    const Isometry3<double>& table_state =
        this->EvalAbstractInput(context, input_port_table_state_[i])
            ->GetValue<Isometry3<double>>();
    internal_state.world_state.HandleTableStatus(i, table_state);
  }

  PickAndPlaceStateMachine::IiwaPublishCallback iiwa_callback =
      ([&](const robotlocomotion::robot_plan_t* plan) {
        internal_state.last_iiwa_plan = *plan;
      });

  PickAndPlaceStateMachine::WsgPublishCallback wsg_callback =
      ([&](const lcmt_schunk_wsg_command* msg) {
        internal_state.last_wsg_command = *msg;
      });
  internal_state.state_machine.Update(internal_state.world_state, iiwa_callback,
                                      wsg_callback, iiwa_tree_,
                                      ignore_tall_tables_);
}

pick_and_place::PickAndPlaceState PickAndPlaceStateMachineSystem::state(
    const systems::Context<double>& context) const {
  const InternalState& internal_state =
      context.get_abstract_state<InternalState>(0);
  return internal_state.state_machine.state();
}

const pick_and_place::WorldState& PickAndPlaceStateMachineSystem::world_state(
    const systems::Context<double>& context) const {
  const InternalState& internal_state =
      context.get_abstract_state<InternalState>(0);
  return internal_state.world_state;
}

}  // namespace monolithic_pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
