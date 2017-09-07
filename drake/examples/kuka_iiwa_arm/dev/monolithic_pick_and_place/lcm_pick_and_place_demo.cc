#include <memory>
#include <string>
#include <vector>

#include <gflags/gflags.h>
#include "bot_core/robot_state_t.hpp"
#include "optitrack/optitrack_frame_t.hpp"
#include "robotlocomotion/robot_plan_t.hpp"

#include "drake/common/find_resource.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/state_machine_system.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_schunk_wsg_command.hpp"
#include "drake/lcmt_schunk_wsg_status.hpp"
#include "drake/lcmtypes/drake/lcmt_schunk_wsg_command.hpp"
#include "drake/lcmtypes/drake/lcmt_viewer_geometry_data.hpp"
#include "drake/lcmtypes/drake/lcmt_viewer_link_data.hpp"
#include "drake/manipulation/perception/optitrack_pose_extractor.h"
#include "drake/multibody/shapes/geometry.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_driven_loop.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/util/lcmUtil.h"

DEFINE_int32(target, 0, "ID of the target to pick.");
DEFINE_int32(end_position, 2, "Position index to end at");
DEFINE_bool(use_optitrack, false, "Use object positions from optitrack data");

using robotlocomotion::robot_plan_t;
using DrakeShapes::Geometry;

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace monolithic_pick_and_place {
namespace {

class OptitrackTranslatorSystem : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(OptitrackTranslatorSystem);

  OptitrackTranslatorSystem(
      const std::vector<lcmt_viewer_geometry_data> known_objects)
      : known_objects_(known_objects) {
    input_port_pose_.resize(known_objects.size());
    output_port_pose_.resize(known_objects.size());
    for (int i = 0; i < num_known_objects(); ++i) {
      input_port_pose_[i] =
          this->DeclareAbstractInputPort(systems::Value<Isometry3<double>>(
                                             Isometry3<double>::Identity()))
              .get_index();
      output_port_pose_[i] = this->DeclareAbstractOutputPort(
                                     bot_core::robot_state_t(),
                                     &OptitrackTranslatorSystem::OutputPose)
                                 .get_index();
    }
    output_port_world_state_ =
        this->DeclareAbstractOutputPort(lcmt_viewer_link_data(),
                                        &OptitrackTranslatorSystem::OutputWorld)
            .get_index();
  }

  int num_known_objects() const { return known_objects_.size(); }

  const systems::InputPortDescriptor<double>& get_input_port_pose(
      int index) const {
    DRAKE_DEMAND(index < static_cast<int>(input_port_pose_.size()));
    return get_input_port(input_port_pose_[index]);
  }

  const systems::OutputPort<double>& get_output_port_pose(
      int index) const {
    DRAKE_DEMAND(index < static_cast<int>(output_port_pose_.size()));
    return get_output_port(output_port_pose_[index]);
  }

  const systems::OutputPort<double>& get_output_port_world() const {
    return get_output_port(output_port_world_state_);
  }

 private:

  void OutputPose(const systems::Context<double>& context,
                  bot_core::robot_state_t* out) const {
    const Isometry3<double>& in = this->EvalAbstractInput(
        context, 0)->template GetValue<Isometry3<double>>();

    out->utime = 0;
    EncodePose(in, out->pose);
  }

  void OutputWorld(const systems::Context<double>& context,
                  lcmt_viewer_link_data* out) const {
    out->geom = known_objects_;
    for (int i = 0; i < num_known_objects(); ++i) {
      const Isometry3<double>& in = this->EvalAbstractInput(
          context, i)->template GetValue<Isometry3<double>>();
      Quaternion<double> quat{in.rotation()};
      Vector3<double> pos{in.translation()};
      out->geom[i].position[0] = pos.x();
      out->geom[i].position[1] = pos.y();
      out->geom[i].position[2] = pos.z();
      out->geom[i].quaternion[0] = quat.w();
      out->geom[i].quaternion[1] = quat.x();
      out->geom[i].quaternion[2] = quat.y();
      out->geom[i].quaternion[3] = quat.z();
    }
  }

  std::vector<int> input_port_pose_;
  int output_port_world_state_;
  std::vector<int> output_port_pose_;
  const std::vector<lcmt_viewer_geometry_data> known_objects_;
};

class RobotStateSplicer : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RobotStateSplicer);

  RobotStateSplicer() {
    input_port_joint_state_ =
        DeclareAbstractInputPort(
            systems::Value<bot_core::robot_state_t>(bot_core::robot_state_t()))
            .get_index();
    input_port_base_state_ =
        DeclareAbstractInputPort(
            systems::Value<bot_core::robot_state_t>(bot_core::robot_state_t()))
            .get_index();
    DeclareAbstractOutputPort(bot_core::robot_state_t(),
                              &RobotStateSplicer::SpliceStates)
        .get_index();
  }

  const systems::InputPortDescriptor<double>& get_input_port_joint_state() const {
    return get_input_port(input_port_joint_state_);
  }

  const systems::InputPortDescriptor<double>& get_input_port_base_state() const {
    return get_input_port(input_port_base_state_);
  }
 private:
  void SpliceStates(const systems::Context<double>& context,
                    bot_core::robot_state_t* out) const {
    const bot_core::robot_state_t& joint_state =
        this->EvalAbstractInput(context, input_port_joint_state_)
            ->GetValue<bot_core::robot_state_t>();
    const bot_core::robot_state_t& base_state =
        this->EvalAbstractInput(context, input_port_base_state_)
            ->GetValue<bot_core::robot_state_t>();
    *out = joint_state;
    out->pose = base_state.pose;
    out->twist = base_state.twist;
  }

  int input_port_joint_state_{-1};
  int input_port_base_state_{-1};
};

const char kIiwaUrdf[] =
    "drake/manipulation/models/iiwa_description/urdf/"
    "iiwa14_polytope_collision.urdf";
const char kIiwaEndEffectorName[] = "iiwa_link_ee";

struct Target {
  std::string model_name;
  Eigen::Vector3d dimensions;
  int object_id;
};

Target GetTarget() {
  Target targets[] = {
    {"block_for_pick_and_place.urdf", Eigen::Vector3d(0.06, 0.06, 0.2), 1},
    {"black_box.urdf", Eigen::Vector3d(0.055, 0.165, 0.18), 2},
    {"simple_cuboid.urdf", Eigen::Vector3d(0.06, 0.06, 0.06), 3},
    {"simple_cylinder.urdf", Eigen::Vector3d(0.065, 0.065, 0.13), 4},
    // These are hacky dimensions for the big robot toy.
    {"simple_cuboid.urdf", Eigen::Vector3d(0.06, 0.06, 0.18), 5}
  };

  const int num_targets = 5;
  if ((FLAGS_target >= num_targets) || (FLAGS_target < 0)) {
    throw std::runtime_error("Invalid target ID");
  }
  return targets[FLAGS_target];
}

int kIiwaBaseOptitrackId = 0;

int DoMain(void) {
  // Locations for the posts from physical pick and place tests with
  // the iiwa+WSG.
  std::vector<Eigen::Vector3d> post_locations;
  // TODO(sam.creasey) this should be 1.10 in the Y direction.
  post_locations.push_back(Eigen::Vector3d(0.00, 0.9, 0.05));  // position A
  post_locations.push_back(Eigen::Vector3d(0.80, 0.36, 0.05));  // position B
  post_locations.push_back(Eigen::Vector3d(0.30, -0.9, 0.012));  // position D
  post_locations.push_back(Eigen::Vector3d(-0.1, -1.0, 0.012));  // position E
  post_locations.push_back(Eigen::Vector3d(-0.47, -0.8, -0.0));  // position F

  // Position of the pick and place location on the table, relative to
  // the base of the arm.  In the original test, the position was
  // specified as 0.90m forward of the arm.  We change that to 0.86
  // here as the previous test grasped the target with the tip of the
  // fingers in the middle while this test places the fingertip
  // further forward.  The position is right at the edge of what we
  // can plan to, so this 4cm change does matter.
  // const Eigen::Vector3d table_position(0.86, -0.36, -0.07);  // position C
  const Eigen::Vector3d table_position(0.80, -0.36, 0.25);  // position C

  // The offset from the top of the table to the top of the post, used for
  // calculating the place locations in iiwa relative coordinates.
  const Eigen::Vector3d post_height_offset(0, 0, 0.27);

  // TODO(sam.creasey) select only one of these
  std::vector<Isometry3<double>> place_locations;
  Isometry3<double> place_location;
  place_location.translation() = post_locations[0] + post_height_offset;
  place_location.linear() = Matrix3<double>(
      AngleAxis<double>(M_PI / 2., Vector3<double>::UnitZ()));
  place_locations.push_back(place_location);

  place_location.translation() = post_locations[1] + post_height_offset;
  place_location.linear().setIdentity();
  place_locations.push_back(place_location);

  place_location.translation() = table_position;
  place_location.linear().setIdentity();
  place_locations.push_back(place_location);

  place_location.translation() = post_locations[2] + post_height_offset;
  place_location.linear() = Matrix3<double>(
      AngleAxis<double>(-M_PI / 2., Vector3<double>::UnitZ()));
  place_locations.push_back(place_location);

  place_location.translation() = post_locations[3] + post_height_offset;
  place_location.linear() = Matrix3<double>(
      AngleAxis<double>(-M_PI / 2., Vector3<double>::UnitZ()));
  place_locations.push_back(place_location);

  place_location.translation() = post_locations[4] + post_height_offset;
  place_location.linear() = Matrix3<double>(
      AngleAxis<double>(-M_PI / 2., Vector3<double>::UnitZ()));
  //  place_locations.push_back(place_location);

  Target target = GetTarget();
  Eigen::Vector3d half_target_height(0, 0, target.dimensions(2) * 0.5);

  for (size_t i = 0; i < place_locations.size(); i++) {
    place_locations[i].translation() += half_target_height;
  }

  lcm::DrakeLcm lcm;
  systems::DiagramBuilder<double> builder;

  const Eigen::Vector3d robot_base(0, 0, 0);
  Isometry3<double> iiwa_base = Isometry3<double>::Identity();
  iiwa_base.translation() = robot_base;

  if (FLAGS_end_position >= 0) {
    if (FLAGS_end_position >= static_cast<int>(place_locations.size())) {
      throw std::runtime_error("Invalid end position specified.");
    }
    std::vector<Isometry3<double>> new_place_locations;
    new_place_locations.push_back(place_locations[FLAGS_end_position]);
    place_locations.swap(new_place_locations);
  }

  auto state_machine =
      builder.AddSystem<PickAndPlaceStateMachineSystem>(
          FindResourceOrThrow(kIiwaUrdf), kIiwaEndEffectorName,
          iiwa_base, place_locations);

  auto iiwa_status_sub = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<bot_core::robot_state_t>(
          "EST_ROBOT_STATE", &lcm));

  auto wsg_status_sub = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<lcmt_schunk_wsg_status>(
          "SCHUNK_WSG_STATUS", &lcm));
  builder.Connect(wsg_status_sub->get_output_port(0),
                  state_machine->get_input_port_wsg_status());

  systems::lcm::LcmSubscriberSystem* object_state_sub = nullptr;
  systems::lcm::LcmSubscriberSystem* optitrack_sub = nullptr;
  if (!FLAGS_use_optitrack) {
    object_state_sub = builder.AddSystem(
        systems::lcm::LcmSubscriberSystem::Make<bot_core::robot_state_t>(
            "OBJECT_STATE_EST", &lcm));
    builder.Connect(object_state_sub->get_output_port(0),
                    state_machine->get_input_port_box_state());
    builder.Connect(iiwa_status_sub->get_output_port(0),
        state_machine->get_input_port_iiwa_state());
  } else {
    optitrack_sub = builder.AddSystem(
        systems::lcm::LcmSubscriberSystem::Make<optitrack::optitrack_frame_t>(
            "OPTITRACK_FRAMES", &lcm));

    Eigen::Isometry3d X_WO = Eigen::Isometry3d::Identity();
    Eigen::Matrix3d rot_mat;
    rot_mat.col(0) = Eigen::Vector3d::UnitY();
    rot_mat.col(1) = Eigen::Vector3d::UnitZ();
    rot_mat.col(2) = Eigen::Vector3d::UnitX();
    X_WO.linear() = rot_mat;
    Eigen::Vector3d translator;
    translator = Eigen::Vector3d::Zero();
    // translator<< 0.0, 0.0, 0;
    translator << -0.342, -0.017, 0.152;
    X_WO.translate(translator);

    auto optitrack_obj_pose_extractor =
        builder.AddSystem<manipulation::perception::OptitrackPoseExtractor>(
            target.object_id, X_WO, 1./120.);
    optitrack_obj_pose_extractor->set_name("Optitrack object pose extractor");
    builder.Connect(optitrack_sub->get_output_port(0),
                    optitrack_obj_pose_extractor->get_input_port(0));

    auto optitrack_iiwa_pose_extractor =
        builder.AddSystem<manipulation::perception::OptitrackPoseExtractor>(
            kIiwaBaseOptitrackId, X_WO, 1./120.);
    optitrack_iiwa_pose_extractor->set_name(
        "Optitrack IIWA base pose extractor");
    builder.Connect(optitrack_sub->get_output_port(0),
                    optitrack_iiwa_pose_extractor->get_input_port(0));


    std::vector<lcmt_viewer_geometry_data> known_geometry;
    known_geometry.emplace_back();
    // Placeholder for iiwa base This should be changed.
    known_geometry.back().type = lcmt_viewer_geometry_data::BOX;
    known_geometry.back().num_float_data = 3;
    known_geometry.back().float_data.push_back(target.dimensions.x());
    known_geometry.back().float_data.push_back(target.dimensions.y());
    known_geometry.back().float_data.push_back(target.dimensions.z());

    known_geometry.emplace_back();
    known_geometry.back().type = lcmt_viewer_geometry_data::BOX;
    known_geometry.back().num_float_data = 3;
    known_geometry.back().float_data.push_back(target.dimensions.x());
    known_geometry.back().float_data.push_back(target.dimensions.y());
    known_geometry.back().float_data.push_back(target.dimensions.z());

    auto optitrack_translator = builder.AddSystem<OptitrackTranslatorSystem>(known_geometry);
    builder.Connect(optitrack_iiwa_pose_extractor->get_measured_pose_output_port(),
                    optitrack_translator->get_input_port_pose(0));

    builder.Connect(optitrack_obj_pose_extractor->get_measured_pose_output_port(),
                    optitrack_translator->get_input_port_pose(1));

    builder.Connect(optitrack_translator->get_output_port_pose(1),
                    state_machine->get_input_port_box_state());

    builder.Connect(optitrack_translator->get_output_port_world(),
                    state_machine->get_input_port_env_state());

    auto iiwa_state_splicer = builder.AddSystem<RobotStateSplicer>();
    builder.Connect(iiwa_status_sub->get_output_port(0),
        iiwa_state_splicer->get_input_port_joint_state());
    builder.Connect(optitrack_translator->get_output_port_pose(0),
        iiwa_state_splicer->get_input_port_base_state());
    builder.Connect(iiwa_state_splicer->get_output_port(0),
        state_machine->get_input_port_iiwa_state());
  }

  auto iiwa_plan_sender = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<robot_plan_t>(
          "COMMITTED_ROBOT_PLAN", &lcm));
  auto wsg_command_sender = builder.AddSystem(
    systems::lcm::LcmPublisherSystem::Make<lcmt_schunk_wsg_command>(
        "SCHUNK_WSG_COMMAND", &lcm));

  builder.Connect(state_machine->get_output_port_iiwa_plan(),
                  iiwa_plan_sender->get_input_port(0));
  builder.Connect(state_machine->get_output_port_wsg_command(),
                  wsg_command_sender->get_input_port(0));

  auto sys = builder.Build();

  systems::lcm::LcmDrivenLoop loop(
      *sys, *iiwa_status_sub, nullptr, &lcm,
      std::make_unique<systems::lcm::UtimeMessageToSeconds<
      bot_core::robot_state_t>>());
  // Wait for the first object state message before doing anything else (if we're not using the optitrack).
  if (object_state_sub) {
    object_state_sub->WaitForMessage(0);
  } else if (optitrack_sub) {
    optitrack_sub->WaitForMessage(0);
  }
  //sys->GetSubsystemContext(*object_status_sub,
  //loop.get_mutable_context());


  // Waits for the first message.
  const systems::AbstractValue& first_msg = loop.WaitForMessage();
  double msg_time =
      loop.get_message_to_time_converter().GetTimeInSeconds(first_msg);
  systems::Context<double>* diagram_context = loop.get_mutable_context();
  // Explicit initialization.
  diagram_context->set_time(msg_time);


  loop.RunToSecondsAssumingInitialized();

#if 0
  // Step the simulator in some small increment.  Between steps, check
  // to see if the state machine thinks we're done, and if so that the
  // object is near the target.
  const double simulation_step = 0.1;
  while (state_machine->state(
             sys->GetSubsystemContext(*state_machine,
                                      *loop.get_mutable_context()))
         != pick_and_place::kDone) {
    simulator.StepTo(simulator.get_context().get_time() + simulation_step);
    if (FLAGS_quick) {
      // We've run a single step, just get out now since we won't have
      // reached our destination.
      return 0;
    }
  }
#endif

#if 0
  const pick_and_place::WorldState& world_state =
      state_machine->world_state(
          sys->GetSubsystemContext(*state_machine,
                                   simulator.get_context()));
  const Isometry3<double>& object_pose = world_state.get_object_pose();
  const Vector6<double>& object_velocity = world_state.get_object_velocity();
  Isometry3<double> goal = place_locations.back();
  goal.translation()(2) += kTableTopZInWorld;
  Eigen::Vector3d object_rpy = math::rotmat2rpy(object_pose.rotation());
  Eigen::Vector3d goal_rpy = math::rotmat2rpy(goal.rotation());

  drake::log()->info("Pose: {} {}",
                     object_pose.translation().transpose(),
                     object_rpy.transpose());
  drake::log()->info("Velocity: {}", object_velocity.transpose());
  drake::log()->info("Goal: {} {}",
                     goal.translation().transpose(),
                     goal_rpy.transpose());

  const double position_tolerance = 0.02;
  Eigen::Vector3d position_error =
      object_pose.translation() - goal.translation();
  drake::log()->info("Position error: {}", position_error.transpose());
  DRAKE_DEMAND(std::abs(position_error(0)) < position_tolerance);
  DRAKE_DEMAND(std::abs(position_error(1)) < position_tolerance);
  DRAKE_DEMAND(std::abs(position_error(2)) < position_tolerance);

  const double angle_tolerance = 0.0873;  // 5 degrees
  Eigen::Vector3d rpy_error = object_rpy - goal_rpy;
  drake::log()->info("RPY error: {}", rpy_error.transpose());
  DRAKE_DEMAND(std::abs(rpy_error(0)) < angle_tolerance);
  DRAKE_DEMAND(std::abs(rpy_error(1)) < angle_tolerance);
  DRAKE_DEMAND(std::abs(rpy_error(2)) < angle_tolerance);


  const double linear_velocity_tolerance = 0.1;
  DRAKE_DEMAND(std::abs(object_velocity(0)) < linear_velocity_tolerance);
  DRAKE_DEMAND(std::abs(object_velocity(1)) < linear_velocity_tolerance);
  DRAKE_DEMAND(std::abs(object_velocity(2)) < linear_velocity_tolerance);

  const double angular_velocity_tolerance = 0.1;
  DRAKE_DEMAND(std::abs(object_velocity(3)) < angular_velocity_tolerance);
  DRAKE_DEMAND(std::abs(object_velocity(4)) < angular_velocity_tolerance);
  DRAKE_DEMAND(std::abs(object_velocity(5)) < angular_velocity_tolerance);
#endif

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
