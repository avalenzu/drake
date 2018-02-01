#include "drake/examples/kuka_iiwa_arm/dev/pick_and_place/lcm_point_to_point_controller.h"

#include "robotlocomotion/robot_plan_t.hpp"

#include "drake/examples/kuka_iiwa_arm/iiwa_lcm.h"
#include "drake/manipulation/planner/pose_interpolator.h"
#include "drake/manipulation/util/trajectory_utils.h"
#include "drake/manipulation/util/world_sim_tree_builder.h"
#include "drake/multibody/joints/fixed_joint.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/demultiplexer.h"
#include "drake/util/lcmUtil.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

using manipulation::planner::DifferentialInverseKinematicsSystem;
using manipulation::planner::PoseInterpolator;
using manipulation::PiecewiseCartesianTrajectory;
using manipulation::util::WorldSimTreeBuilder;
using systems::DiagramBuilder;
using robotlocomotion::robot_plan_t;

namespace {

const char kGraspFrameName[] = "grasp_frame";

// TODO(avalenzu): Remove this awful hack.
class PlanToEndEffectorTrajectoryConverter
    : public systems::LeafSystem<double> {
 public:
  PlanToEndEffectorTrajectoryConverter() {
    this->DeclareAbstractInputPort();
    this->DeclareAbstractOutputPort(
        PiecewiseCartesianTrajectory<double>(),
        &PlanToEndEffectorTrajectoryConverter::CalcTrajectoryOutput);
  }

 private:
  void CalcTrajectoryOutput(
      const systems::Context<double>& context,
      PiecewiseCartesianTrajectory<double>* trajectory) const {
    const robot_plan_t& plan_input =
        this->EvalAbstractInput(context, 0)->GetValue<robot_plan_t>();
    if (plan_input.plan.empty()) {
      *trajectory = PiecewiseCartesianTrajectory<double>::
          MakeCubicLinearWithEndLinearVelocity(
              {0.0, 0.0},
              {Isometry3<double>::Identity(), Isometry3<double>::Identity()},
              Vector3<double>::Zero(), Vector3<double>::Zero());
    } else {
      *trajectory = PiecewiseCartesianTrajectory<double>::
          MakeCubicLinearWithEndLinearVelocity(
              {0.0,
               1e6 * (plan_input.plan.back().utime -
                      plan_input.plan.front().utime)},
              {DecodePose(plan_input.plan.front().pose),
               DecodePose(plan_input.plan.back().pose)},
              Vector3<double>::Zero(), Vector3<double>::Zero());
    }
  }
};

std::unique_ptr<RigidBodyTree<double>> BuildTree(
    const pick_and_place::PlannerConfiguration& configuration,
    bool add_grasp_frame = false, int num_arms = 1) {
  WorldSimTreeBuilder<double> tree_builder;
  tree_builder.StoreModel("iiwa", configuration.absolute_model_path());
  std::vector<int> arm_instance_ids(num_arms, 0);
  auto previous_log_level = drake::log()->level();
  drake::log()->set_level(spdlog::level::warn);
  for (int i = 0; i < num_arms; ++i) {
    arm_instance_ids[i] =
        tree_builder.AddFixedModelInstance("iiwa", Vector3<double>::Zero());
  }

  std::unique_ptr<RigidBodyTree<double>> robot{tree_builder.Build()};
  if (add_grasp_frame) {
    // Add the grasp frame as a RigidBodyFrame constraints.
    // TODO(avalenzu): Add a planning model for the gripper that includes the
    // grasp frame as a named frame.
    // The gripper (and therfore the grasp frame) is rotated relative to the end
    // effector link.
    const double grasp_frame_angular_offset{-M_PI / 8};
    // The grasp frame is located between the fingertips of the gripper, which
    // puts it grasp_frame_translational_offset from the origin of the
    // end-effector link.
    const double grasp_frame_translational_offset{0.19};
    // Define the pose of the grasp frame (G) relative to the end effector (E).
    Isometry3<double> X_EG{Isometry3<double>::Identity()};
    X_EG.rotate(Eigen::AngleAxisd(grasp_frame_angular_offset,
                                  Eigen::Vector3d::UnitX()));
    X_EG.translation().x() = grasp_frame_translational_offset;
    robot->addFrame(std::make_shared<RigidBodyFrame<double>>(
        kGraspFrameName, robot->FindBody(configuration.end_effector_name),
        X_EG));
    robot->compile();
  }

  // The iiwa driver limits joint angle commands to one degree less
  // than the min/max of each joint's range to avoid triggering
  // exceptions in the controller when the limit is reached (for
  // example, if a joint's range is +/- 120 degrees, the commanded
  // joint positions sent to the hardware will be capped to a minimum
  // of -119 and a maximum of 119 degrees).  Update the tree we're
  // using for planning to reflect this limit.
  const double kOneDegreeInRadians = M_PI / 180.;
  robot->joint_limit_min += Eigen::VectorXd::Constant(
      robot->joint_limit_min.size(), kOneDegreeInRadians);
  robot->joint_limit_max -= Eigen::VectorXd::Constant(
      robot->joint_limit_min.size(), kOneDegreeInRadians);

  drake::log()->set_level(previous_log_level);
  return robot;
}
}  // namespace

LcmPointToPointController::LcmPointToPointController(
    const pick_and_place::PlannerConfiguration& configuration) {
  DiagramBuilder<double> builder;

  // Add differential inverse kinematics block.
  differential_inverse_kinematics_ =
      builder.AddSystem<DifferentialInverseKinematicsSystem>(
          BuildTree(configuration, true), kGraspFrameName);

  // Add block to convert iiwa status to position + velocity vector.
  auto status_receiver = builder.AddSystem<IiwaStatusReceiver>(num_joints());

  // Add a demux block to split apart the position + velocity vector.
  auto state_demux =
      builder.AddSystem<systems::Demultiplexer>(num_joints() * 2, num_joints());
  state_demux->set_name("state_demux");

  // Add a block to convert the base pose of the robot_plan_t to a trajectory.
  auto plan_to_end_effector_trajectory_converter =
      builder.AddSystem<PlanToEndEffectorTrajectoryConverter>();

  // Add a block to interpolate the trajectory.
  auto pose_interpolator = builder.AddSystem<PoseInterpolator>();

  // Add a block to convert the desired position vector to iiwa command.
  auto command_sender = builder.AddSystem<IiwaCommandSender>(num_joints());
  command_sender->set_name("command_sender");

  // Export the inputs.
  iiwa_status_input_port_ =
      builder.ExportInput(status_receiver->get_input_port(0));
  desired_end_effector_pose_input_port_ =
      builder.ExportInput(plan_to_end_effector_trajectory_converter->get_input_port(0));

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
  builder.Connect(plan_to_end_effector_trajectory_converter->get_output_port(0),
      pose_interpolator->trajectory_input_port());
  builder.Connect(
      pose_interpolator->pose_output_port(),
      differential_inverse_kinematics_->desired_end_effector_pose_input_port());
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
