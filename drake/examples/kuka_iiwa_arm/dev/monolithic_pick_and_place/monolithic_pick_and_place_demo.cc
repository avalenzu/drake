#include <memory>
#include <string>
#include <vector>

#include <gflags/gflags.h>
#include "bot_core/robot_state_t.hpp"
#include "robotlocomotion/robot_plan_t.hpp"

#include "drake/common/find_resource.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/pick_and_place_configuration_parsing.h"
#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/pick_and_place_planner.h"
#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/pick_and_place_plant.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_lcm.h"
#include "drake/examples/kuka_iiwa_arm/lcm_plan_interpolator.h"
#include "drake/examples/kuka_iiwa_arm/pick_and_place/pick_and_place_configuration.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_contact_results_for_viz.hpp"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/rigid_body_plant/contact_results_to_lcm.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/primitives/constant_vector_source.h"

DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(),
              "Number of seconds to simulate.");
DEFINE_double(dt, 5e-4, "Integration step size");
DEFINE_double(realtime_rate, 1.0,
              "Rate at which to run the simulation, "
              "relative to realtime");
DEFINE_bool(quick, false,
            "Run only a brief simulation and return success "
            "without executing the entire task");
DEFINE_string(configuration_file,
              "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/"
              "configuration/default.pick_and_place_configuration",
              "Path to the configuration file.");

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace monolithic_pick_and_place {
namespace {

using manipulation::planner::InterpolatorType;
using systems::Simulator;
using systems::RungeKutta2Integrator;

const std::string kIiwaUrdf =
    "drake/manipulation/models/iiwa_description/urdf/"
    "iiwa14_polytope_collision.urdf";

int DoMain(void) {
  // Parse configuration file
  const pick_and_place::SimulatedPlantConfiguration plant_configuration =
      ParseSimulatedPlantConfigurationOrThrow(FLAGS_configuration_file);
  const pick_and_place::OptitrackConfiguration optitrack_configuration =
      ParseOptitrackConfigurationOrThrow(FLAGS_configuration_file);

  lcm::DrakeLcm lcm;
  systems::DiagramBuilder<double> builder;

  // Add the plant.
  auto plant = builder.AddSystem<PickAndPlacePlant>(plant_configuration,
                                                    optitrack_configuration);

  // Add blocks to publish contact results.
  auto contact_viz =
      builder.AddSystem<systems::ContactResultsToLcmSystem<double>>(
          plant->get_tree());
  auto contact_results_publisher = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<lcmt_contact_results_for_viz>(
          "CONTACT_RESULTS", &lcm));
  builder.Connect(plant->get_output_port_contact_results(),
                  contact_viz->get_input_port(0));
  builder.Connect(contact_viz->get_output_port(0),
                  contact_results_publisher->get_input_port(0));
  contact_results_publisher->set_publish_period(kIiwaLcmStatusPeriod);

  // Add visualizer.
  auto drake_visualizer =
      builder.AddSystem<systems::DrakeVisualizer>(plant->get_tree(), &lcm);
  drake_visualizer->set_publish_period(kIiwaLcmStatusPeriod);

  builder.Connect(plant->get_output_port_plant_state(),
                  drake_visualizer->get_input_port(0));

  // Loop over arms to add planners and plan interpolators.
  const int kNumIiwa(plant_configuration.robot_base_poses.size());
  std::vector<PickAndPlacePlanner*> planners;
  for (int i = 0; i < kNumIiwa; ++i) {
    // Parse planner configuration.
    const pick_and_place::PlannerConfiguration planner_configuration =
        ParsePlannerConfigurationOrThrow(FLAGS_configuration_file, RobotBaseIndex(i), TargetIndex(i));
    // Add planner.
    auto planner =
        builder.AddSystem<PickAndPlacePlanner>(planner_configuration);
    planners.push_back(planner);
    // Add plan interpolator.
    auto plan_interpolator = builder.AddSystem<LcmPlanInterpolator>(FindResourceOrThrow(planner_configuration.model_path), InterpolatorType::Cubic);
    // Connect planner, plan interpolator, and plant.
    // Plant --> Planner
    builder.Connect(plant->get_output_port_wsg_status(i), planner->get_input_port_wsg_status());
    builder.Connect(plant->get_output_port_iiwa_robot_state(i), planner->get_input_port_iiwa_state());
    builder.Connect(plant->get_output_port_optitrack_frame(), planner->get_input_port_optitrack_message());
    // Plant --> plan interpolator
    builder.Connect(plant->get_output_port_iiwa_status(i), plan_interpolator->get_input_port_iiwa_status());
    // Planner --> Plant
    builder.Connect(planner->get_output_port_wsg_command(), plant->get_input_port_wsg_command(i));
    // Planner --> Plan Interpolator
    builder.Connect(planner->get_output_port_iiwa_plan(), plan_interpolator->get_input_port_iiwa_plan());
    // plan interpolator --> plant
    builder.Connect(plan_interpolator->get_output_port_iiwa_command(), plant->get_input_port_iiwa_command(i));
  }

  auto sys = builder.Build();
  Simulator<double> simulator(*sys);
  simulator.set_target_realtime_rate(FLAGS_realtime_rate);
  simulator.reset_integrator<RungeKutta2Integrator<double>>(*sys,
      FLAGS_dt, simulator.get_mutable_context());
  simulator.get_mutable_integrator()->set_maximum_step_size(FLAGS_dt);
  simulator.get_mutable_integrator()->set_fixed_step_mode(true);

  lcm.StartReceiveThread();
  simulator.set_publish_every_time_step(false);
  simulator.Initialize();

  // Step the simulator in some small increment.  Between steps, check
  // to see if the state machine thinks we're done, and if so that the
  // object is near the target.
  const double simulation_step = 0.1;
  bool done{false};
  while (!done) {
    simulator.StepTo(simulator.get_context().get_time() + simulation_step);
    if (FLAGS_quick) {
      // We've run a single step, just get out now since we won't have
      // reached our destination.
      return 0;
    }
    done = true;
    for (const auto& planner : planners) {
      done = done &&
             planner->state(
                 sys->GetSubsystemContext(*planner, simulator.get_context())) ==
                 pick_and_place::PickAndPlaceState::kDone;
    }
  }

  //const pick_and_place::WorldState& world_state = state_machine->world_state(
      //sys->GetSubsystemContext(*state_machine, simulator.get_context()));
  //const Isometry3<double>& object_pose = world_state.get_object_pose();
  //const Vector6<double>& object_velocity = world_state.get_object_velocity();
  //Isometry3<double> goal = place_locations.back();
  //goal.translation()(2) += kTableTopZInWorld;
  //Eigen::Vector3d object_rpy = math::rotmat2rpy(object_pose.linear());
  //Eigen::Vector3d goal_rpy = math::rotmat2rpy(goal.linear());

  //drake::log()->info("Pose: {} {}", object_pose.translation().transpose(),
                     //object_rpy.transpose());
  //drake::log()->info("Velocity: {}", object_velocity.transpose());
  //drake::log()->info("Goal: {} {}", goal.translation().transpose(),
                     //goal_rpy.transpose());

  //const double position_tolerance = 0.02;
  //Eigen::Vector3d position_error =
      //object_pose.translation() - goal.translation();
  //drake::log()->info("Position error: {}", position_error.transpose());
  //DRAKE_DEMAND(std::abs(position_error(0)) < position_tolerance);
  //DRAKE_DEMAND(std::abs(position_error(1)) < position_tolerance);
  //DRAKE_DEMAND(std::abs(position_error(2)) < position_tolerance);

  //const double angle_tolerance = 0.0873;  // 5 degrees
  //Eigen::Vector3d rpy_error = object_rpy - goal_rpy;
  //drake::log()->info("RPY error: {}", rpy_error.transpose());
  //DRAKE_DEMAND(std::abs(rpy_error(0)) < angle_tolerance);
  //DRAKE_DEMAND(std::abs(rpy_error(1)) < angle_tolerance);
  //DRAKE_DEMAND(std::abs(rpy_error(2)) < angle_tolerance);

  //const double linear_velocity_tolerance = 0.1;
  //DRAKE_DEMAND(std::abs(object_velocity(0)) < linear_velocity_tolerance);
  //DRAKE_DEMAND(std::abs(object_velocity(1)) < linear_velocity_tolerance);
  //DRAKE_DEMAND(std::abs(object_velocity(2)) < linear_velocity_tolerance);

  //const double angular_velocity_tolerance = 0.1;
  //DRAKE_DEMAND(std::abs(object_velocity(3)) < angular_velocity_tolerance);
  //DRAKE_DEMAND(std::abs(object_velocity(4)) < angular_velocity_tolerance);
  //DRAKE_DEMAND(std::abs(object_velocity(5)) < angular_velocity_tolerance);

  return 0;
}

}  // namespace
}  // namespace monolithic_pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();
  return drake::examples::kuka_iiwa_arm::monolithic_pick_and_place::DoMain();
}
