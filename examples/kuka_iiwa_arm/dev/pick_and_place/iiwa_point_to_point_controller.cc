/// @file
///
/// Implements a controller for a KUKA iiwa arm.

#include <memory>

#include <gflags/gflags.h>
#include "robotlocomotion/robot_plan_t.hpp"

#include "drake/common/find_resource.h"
#include "drake/common/text_logging.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/examples/kuka_iiwa_arm/dev/pick_and_place/lcm_point_to_point_controller.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/examples/kuka_iiwa_arm/pick_and_place/pick_and_place_configuration.h"
#include "drake/examples/kuka_iiwa_arm/pick_and_place/pick_and_place_configuration_parsing.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
//#include "drake/systems/analysis/simulator.h"
#include "drake/solvers/mosek_solver.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_driven_loop.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/demultiplexer.h"

using robotlocomotion::robot_plan_t;

DEFINE_double(dt, 0.01, "Timestep for controller");
DEFINE_string(lcm_status_channel, "IIWA_STATUS",
              "Channel on which to listen for lcmt_iiwa_status messages.");
DEFINE_string(lcm_command_channel, "IIWA_COMMAND",
              "Channel on which to publish lcmt_iiwa_command messages.");
DEFINE_string(lcm_plan_channel, "COMMITTED_ROBOT_PLAN",
              "Channel on which to listen for robot_plan_t messages.");
DEFINE_string(configuration_file,
              "drake/examples/kuka_iiwa_arm/pick_and_place/configuration/"
              "yellow_posts.pick_and_place_configuration",
              "Path to the configuration file.");

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

// Create a system which has an integrator on the interpolated
// reference position for received plans.
int DoMain() {
  const std::string kLcmStatusChannel = FLAGS_lcm_status_channel;
  const std::string kLcmCommandChannel = FLAGS_lcm_command_channel;
  const std::string kLcmPlanChannel = FLAGS_lcm_plan_channel;

  // Parse the configuration file.
  const pick_and_place::PlannerConfiguration planner_configuration =
      pick_and_place::ParsePlannerConfigurationOrThrow(
          FLAGS_configuration_file);

  lcm::DrakeLcm lcm;
  systems::DiagramBuilder<double> builder;

  drake::log()->debug("Status channel: {}", kLcmStatusChannel);
  drake::log()->debug("Command channel: {}", kLcmCommandChannel);
  drake::log()->debug("Plan channel: {}", kLcmPlanChannel);

  auto point_to_point_controller =
      builder.AddSystem<LcmPointToPointController>(planner_configuration);
  const int kNumJoints = point_to_point_controller->num_joints();

  auto plan_sub =
      builder.AddSystem(systems::lcm::LcmSubscriberSystem::Make<robot_plan_t>(
          kLcmPlanChannel, &lcm));
  plan_sub->set_name("plan_sub");

  auto status_sub = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<lcmt_iiwa_status>(
          kLcmStatusChannel, &lcm));
  status_sub->set_name("status_sub");

  auto command_pub = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<lcmt_iiwa_command>(
          kLcmCommandChannel, &lcm));
  command_pub->set_name("command_pub");

  // Connect subscribers to input ports.
  builder.Connect(plan_sub->get_output_port(0),
                  point_to_point_controller->plan_input_port());
  builder.Connect(status_sub->get_output_port(0),
                  point_to_point_controller->iiwa_status_input_port());

  // Connect publisher to output port.
  builder.Connect(point_to_point_controller->iiwa_command_output_port(),
                  command_pub->get_input_port(0));

  auto diagram = builder.Build();

  drake::log()->info("controller started");

  systems::lcm::LcmDrivenLoop loop(
      *diagram, *status_sub, nullptr, &lcm,
      std::make_unique<
          systems::lcm::UtimeMessageToSeconds<lcmt_iiwa_status>>());
  // loop.set_publish_on_every_received_message(false);

  // Waits for the first message.
  const systems::AbstractValue& first_msg = loop.WaitForMessage();
  double msg_time =
      loop.get_message_to_time_converter().GetTimeInSeconds(first_msg);
  const lcmt_iiwa_status& first_status = first_msg.GetValue<lcmt_iiwa_status>();
  DRAKE_DEMAND(kNumJoints == first_status.num_joints);
  VectorX<double> q0(kNumJoints);
  for (int i = 0; i < kNumJoints; i++)
    q0[i] = first_status.joint_position_measured[i];

  systems::Context<double>& diagram_context = loop.get_mutable_context();
  systems::Context<double>& status_sub_context =
      diagram->GetMutableSubsystemContext(*status_sub, &diagram_context);
  status_sub->SetDefaultContext(&status_sub_context);

  // Explicit initialization.
  diagram_context.set_time(msg_time);
  auto& point_to_point_controller_context = diagram->GetMutableSubsystemContext(
      *point_to_point_controller, &diagram_context);
  VectorX<double> max_joint_velocities = 0.9 * get_iiwa_max_joint_velocities();
  VectorX<double> min_joint_velocities = -max_joint_velocities;
  point_to_point_controller
      ->MutableParameters(&point_to_point_controller_context)
      .set_timestep(FLAGS_dt);
  point_to_point_controller
      ->MutableParameters(&point_to_point_controller_context)
      .SetJointVelocityLimits({min_joint_velocities, max_joint_velocities});
  VectorX<double> comfortable_joint_position{VectorX<double>::Zero(kNumJoints)};
  comfortable_joint_position(1) = -M_PI_4;
  comfortable_joint_position(3) = -M_PI_2;
  point_to_point_controller
      ->MutableParameters(&point_to_point_controller_context)
      .set_nominal_joint_position(comfortable_joint_position);

  //auto mosek_licence = solvers::MosekSolver::AcquireLicense();
  point_to_point_controller->Initialize(q0, &point_to_point_controller_context);
  loop.RunToSecondsAssumingInitialized();
  return 0;
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();
  return drake::examples::kuka_iiwa_arm::DoMain();
}
