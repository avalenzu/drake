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
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/primitives/demultiplexer.h"
#include "drake/util/lcmUtil.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

using manipulation::PiecewiseCartesianTrajectory;
using manipulation::planner::DifferentialInverseKinematicsSystem;
using manipulation::planner::PoseInterpolator;
using manipulation::util::WorldSimTreeBuilder;
using robotlocomotion::robot_plan_t;
using systems::DiagramBuilder;

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
      Isometry3<double> X = Isometry3<double>::Identity();
      X.rotate(AngleAxis<double>(M_PI_4, Vector3<double>::UnitY()));
      X.translation() << 0.5, 0.0, 0.5;
      *trajectory = PiecewiseCartesianTrajectory<double>::
          MakeCubicLinearWithEndLinearVelocity({0.0, 1.0}, {X, X},
                                               Vector3<double>::Zero(),
                                               Vector3<double>::Zero());
    } else {
      *trajectory = PiecewiseCartesianTrajectory<double>::
          MakeCubicLinearWithEndLinearVelocity(
              {0.0,
               1e-6 * (plan_input.plan.back().utime -
                       plan_input.plan.front().utime)},
              {DecodePose(plan_input.plan.front().pose),
               DecodePose(plan_input.plan.back().pose)},
              Vector3<double>::Zero(), Vector3<double>::Zero());
      drake::log()->debug(
          "t0 = {}, tf = {}",
          trajectory->get_position_trajectory().get_start_time(),
          trajectory->get_position_trajectory().get_end_time());
    }
  }
};

/**
 * X_W1 = X_WErr * X_W0 <=> X_WErr = X_W1 * X_W0.inv()
 * p_err = pose1.translation() - pos0.translation()
 * R_err = pose1.linear() * pose0.linear().transpose().
 */
Vector6<double> ComputePoseDiffInWorldFrame(const Isometry3<double>& pose0,
                                            const Isometry3<double>& pose1) {
  Vector6<double> diff = Vector6<double>::Zero();

  // Linear.
  diff.tail<3>() = (pose1.translation() - pose0.translation());

  // Angular.
  AngleAxis<double> rot_err(pose1.linear() * pose0.linear().transpose());
  diff.head<3>() = rot_err.axis() * rot_err.angle();

  return diff;
}

class FrameSpatialVelocityConstraint : public systems::LeafSystem<double> {
 public:
  FrameSpatialVelocityConstraint(
      std::unique_ptr<RigidBodyTree<double>> robot,
      const std::string& end_effector_frame_name,
      double update_interval = kDefaultPlanUpdateInterval)
      : robot_(std::move(robot)), update_interval_(update_interval) {
    end_effector_frame_ = robot_->findFrame(end_effector_frame_name);
    const int num_positions = robot_->get_num_positions();
    const int num_velocities = robot_->get_num_velocities();
    // Input ports
    joint_position_input_port_ =
        this->DeclareInputPort(systems::kVectorValued, num_positions)
            .get_index();
    joint_velocity_input_port_ =
        this->DeclareInputPort(systems::kVectorValued, num_velocities)
            .get_index();
    plan_input_port_ = this->DeclareAbstractInputPort().get_index();
    // State
    trajectory_state_index_ = this->DeclareAbstractState(
        systems::Value<PiecewiseCartesianTrajectory<double>>::Make(
            PiecewiseCartesianTrajectory<double>()));
    last_encoded_msg_index_ = this->DeclareAbstractState(
        systems::Value<std::vector<char>>::Make(std::vector<char>()));
    this->DeclareDiscreteState(1);
    start_time_index_ = 0;
    // Output ports
    std::pair<VectorX<double>, MatrixX<double>> initial_constraint{
        VectorX<double>::Zero(6), MatrixX<double>::Zero(6, num_velocities)};
    this->DeclareAbstractOutputPort(
        initial_constraint,
        &FrameSpatialVelocityConstraint::CalcConstraintOutput);
    this->DeclarePeriodicUnrestrictedUpdate(update_interval_, 0);
  }

  const systems::InputPortDescriptor<double>& joint_position_input_port()
      const {
    return this->get_input_port(joint_position_input_port_);
  }

  const systems::InputPortDescriptor<double>& joint_velocity_input_port()
      const {
    return this->get_input_port(joint_velocity_input_port_);
  }

  const systems::InputPortDescriptor<double>& plan_input_port() const {
    return this->get_input_port(plan_input_port_);
  }

  const systems::OutputPort<double>& constraint_output_port() const {
    return this->get_output_port(0);
  }

 protected:
  void DoCalcUnrestrictedUpdate(
      const systems::Context<double>& context,
      const std::vector<const systems::UnrestrictedUpdateEvent<double>*>&,
      systems::State<double>* state) const {
    const robot_plan_t& plan_input =
        this->EvalAbstractInput(context, plan_input_port_)
            ->GetValue<robot_plan_t>();
    if (!plan_input.plan.empty()) {
      std::vector<char>& last_encoded_msg =
          state->get_mutable_abstract_state<std::vector<char>>(
              last_encoded_msg_index_);
      std::vector<char> encoded_msg(plan_input.getEncodedSize());
      plan_input.encode(encoded_msg.data(), 0, encoded_msg.size());
      if (encoded_msg != last_encoded_msg) {
        // Extract the state and input trajectories.
        PiecewiseCartesianTrajectory<double>& trajectory_state =
            state->get_mutable_abstract_state<
                PiecewiseCartesianTrajectory<double>>(trajectory_state_index_);
        trajectory_state = PiecewiseCartesianTrajectory<double>::
            MakeCubicLinearWithEndLinearVelocity(
                {0.0,
                 1e-6 * (plan_input.plan.back().utime -
                         plan_input.plan.front().utime)},
                {DecodePose(plan_input.plan.front().pose),
                 DecodePose(plan_input.plan.back().pose)},
                Vector3<double>::Zero(), Vector3<double>::Zero());
        // t = 0 in the input trajectory corresponds to the current time.
        state->get_mutable_discrete_state()
            .get_mutable_vector()
            .get_mutable_value()(start_time_index_) = context.get_time();
        last_encoded_msg = encoded_msg;
      }
    }
  }

 private:
  const systems::BasicVector<double>& EvaluateJointPosition(
      const systems::Context<double>& context) const {
    const systems::BasicVector<double>* joint_position =
        this->EvalVectorInput(context, joint_position_input_port_);
    DRAKE_THROW_UNLESS(joint_position);
    return *joint_position;
  }

  void CalcConstraintOutput(
      const systems::Context<double>& context,
      std::pair<VectorX<double>, MatrixX<double>>* constraint) const {
    const VectorX<double>& joint_position =
        this->EvaluateJointPosition(context).get_value();
    const auto& trajectory =
        context.get_abstract_state<PiecewiseCartesianTrajectory<double>>(
            trajectory_state_index_);
    const auto start_time =
        context.get_discrete_state().get_vector().GetAtIndex(start_time_index_);

    KinematicsCache<double> cache = robot_->doKinematics(joint_position);
    Isometry3<double> X_WE =
        robot_->CalcFramePoseInWorldFrame(cache, *end_effector_frame_);
    Vector6<double> V_WE = Vector6<double>::Zero();;
    if (trajectory.empty()) {
      V_WE = Vector6<double>::Zero();
      drake::log()->debug("t = {}, Traj: N, V = {}",
                          context.get_time() - start_time,
                          V_WE.tail(3).transpose());
    } else {
      Isometry3<double> X_WE_desired =
          trajectory.get_pose(context.get_time() - start_time);
      V_WE += trajectory.get_velocity(context.get_time() - start_time);
      V_WE += 1e-1*ComputePoseDiffInWorldFrame(X_WE, X_WE_desired) / update_interval_;
      drake::log()->debug("t = {}, Traj: Y, V = {}",
                          context.get_time() - start_time,
                          V_WE.tail(3).transpose());
    }

    drake::Matrix6<double> R_EW = drake::Matrix6<double>::Zero();
    R_EW.block<3, 3>(0, 0) = X_WE.linear().transpose();
    R_EW.block<3, 3>(3, 3) = R_EW.block<3, 3>(0, 0);

    // Rotate the velocity into E frame.
    constraint->first = R_EW * V_WE;
    constraint->second = R_EW *
                         robot_->CalcFrameSpatialVelocityJacobianInWorldFrame(
                             cache, *end_effector_frame_);
  }

  static constexpr double kDefaultPlanUpdateInterval = 0.01;

  // Input port indices
  int joint_position_input_port_{-1};
  int joint_velocity_input_port_{-1};
  int plan_input_port_{-1};
  // State indices
  int trajectory_state_index_{-1};
  int last_encoded_msg_index_{-1};
  int start_time_index_{-1};

  std::unique_ptr<RigidBodyTree<double>> robot_;
  std::shared_ptr<RigidBodyFrame<double>> end_effector_frame_{};
  double update_interval_{kDefaultPlanUpdateInterval};
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

  // Add a block to convert the base pose of the robot_plan_t to a constraint.
  auto frame_spatial_velocity_constraint =
      builder.AddSystem<FrameSpatialVelocityConstraint>(
          BuildTree(configuration, true), kGraspFrameName, 0.005);

  // Add a block to convert the desired position vector to iiwa command.
  auto command_sender = builder.AddSystem<IiwaCommandSender>(num_joints());
  command_sender->set_name("command_sender");

  // Export the inputs.
  iiwa_status_input_port_ =
      builder.ExportInput(status_receiver->get_input_port(0));
  plan_input_port_ =
      builder.ExportInput(frame_spatial_velocity_constraint->plan_input_port());

  // Export the output.
  iiwa_command_output_port_ =
      builder.ExportOutput(command_sender->get_output_port(0));

  // Connect the subsystems.
  builder.Connect(status_receiver->get_measured_position_output_port(),
                  state_demux->get_input_port(0));
  builder.Connect(
      state_demux->get_output_port(0),
      frame_spatial_velocity_constraint->joint_position_input_port());
  builder.Connect(
      state_demux->get_output_port(1),
      frame_spatial_velocity_constraint->joint_velocity_input_port());
  builder.Connect(
      state_demux->get_output_port(0),
      differential_inverse_kinematics_->joint_position_input_port());
  builder.Connect(
      state_demux->get_output_port(1),
      differential_inverse_kinematics_->joint_velocity_input_port());
  builder.Connect(frame_spatial_velocity_constraint->constraint_output_port(),
                  differential_inverse_kinematics_->constraint_input_port());
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
