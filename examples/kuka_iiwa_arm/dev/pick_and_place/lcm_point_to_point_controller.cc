#include "drake/examples/kuka_iiwa_arm/dev/pick_and_place/lcm_point_to_point_controller.h"

#include "drake/examples/kuka_iiwa_arm/iiwa_lcm.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/demultiplexer.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

using manipulation::planner::DifferentialInverseKinematicsSystem;
using systems::DiagramBuilder;

LcmPointToPointController::LcmPointToPointController(
    const std::string& model_path, const std::string& end_effector_frame_name) {
  std::unique_ptr<RigidBodyTree<double>> tree;
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      model_path, multibody::joints::kFixed, tree.get());

  DiagramBuilder<double> builder;

  // Add block to convert iiwa status to position + velocity vector.
  auto status_receiver = builder.AddSystem<IiwaStatusReceiver>(num_joints_);

  // Add differential inverse kinematics block.
  differential_inverse_kinematics_ =
      builder.AddSystem<DifferentialInverseKinematicsSystem>(
          std::move(tree), end_effector_frame_name);

  // Add a block to convert the desired position vector to iiwa command.
  auto command_sender = builder.AddSystem<IiwaCommandSender>(num_joints_);
  command_sender->set_name("command_sender");

  // Add a demux block to split apart the position + velocity vector.
  auto state_demux =
      builder.AddSystem<systems::Demultiplexer>(num_joints_ * 2, num_joints_);
  state_demux->set_name("state_demux");

  // Export the inputs.
  iiwa_status_input_port_ =
      builder.ExportInput(status_receiver->get_input_port(0));
  desired_end_effector_pose_input_port_ = builder.ExportInput(
      differential_inverse_kinematics_->desired_end_effector_pose_input_port());

  // Export the output.
  iiwa_command_output_port_ =
      builder.ExportOutput(command_sender->get_output_port(0));

  // Connect the subsystems.
  builder.Connect(status_receiver->get_measured_position_output_port(),
                  state_demux->get_input_port(0));
  builder.Connect(
      state_demux->get_output_port(0),
      differential_inverse_kinematics_->joint_position_input_port());
  builder.Connect(
      state_demux->get_output_port(1),
      differential_inverse_kinematics_->joint_velocity_input_port());
  builder.Connect(
      differential_inverse_kinematics_->desired_joint_position_output_port(),
      command_sender->get_position_input_port());

  // Build the system.
  builder.BuildInto(this);
}

void LcmPointToPointController::Initialize(
    const VectorX<double>& initial_joint_position,
    systems::Context<double>* context) const {
  differential_inverse_kinematics_->Initialize(initial_joint_position, context);
}

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
