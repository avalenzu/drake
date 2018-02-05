#include "drake/examples/kuka_iiwa_arm/pick_and_place/pick_and_place_state_machine.h"

#include <algorithm>
#include <limits>
#include <random>
#include <string>

#include <spdlog/fmt/ostr.h>

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"
#include "drake/common/trajectories/piecewise_quaternion.h"
#include "drake/manipulation/util/world_sim_tree_builder.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/joints/fixed_joint.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_ik.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace pick_and_place {
namespace {

using manipulation::util::WorldSimTreeBuilder;

const char kGraspFrameName[] = "grasp_frame";

struct PostureInterpolationRequest {
  // Initial configuration
  MatrixX<double> q_initial;
  // Final configuration
  MatrixX<double> q_final;
  // Knots
  std::vector<double> times;
  // Maximum allowable deviation from straight line end-effector path at knot
  // points
  double position_tolerance;
  // Maximum allowable angular deviation at knot points
  double orientation_tolerance;
  // If true, interpolate in joint space if the planner fails to find an
  // interpolation that provides a
  // straight-line end-effector path.
  bool fall_back_to_joint_space_interpolation;
  double max_joint_position_change;
};

struct PostureInterpolationResult {
  // Configuration trajectory
  PiecewisePolynomial<double> q_traj;
  // Success
  bool success;
};

void OpenGripper(const WorldState& env_state, double grip_force,
                 WsgAction* wsg_act, lcmt_schunk_wsg_command* msg) {
  wsg_act->OpenGripper(env_state, grip_force, msg);
}

void CloseGripper(const WorldState& env_state, double grip_force,
                  WsgAction* wsg_act, lcmt_schunk_wsg_command* msg) {
  wsg_act->CloseGripper(env_state, grip_force, msg);
}

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
    // Add the grasp frame as a RigidBody. This allows it to be used in IK
    // constraints.
    // TODO(avalenzu): Add a planning model for the gripper that includes the
    // grasp frame as a named frame.
    auto grasp_frame = std::make_unique<RigidBody<double>>();
    grasp_frame->set_name(kGraspFrameName);
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
    // Rigidly affix the grasp frame RigidBody to the end effector RigidBody.
    std::string grasp_frame_joint_name = kGraspFrameName;
    grasp_frame_joint_name += "_joint";
    auto grasp_frame_fixed_joint =
        std::make_unique<FixedJoint>(grasp_frame_joint_name, X_EG);
    grasp_frame->add_joint(robot->FindBody(configuration.end_effector_name),
                           std::move(grasp_frame_fixed_joint));
    robot->add_rigid_body(std::move(grasp_frame));
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

optional<std::pair<Isometry3<double>, Isometry3<double>>>
ComputeInitialAndFinalObjectPoses(const WorldState& env_state) {
  // W -- World frame, coincides with robot base frame.
  // S -- fixed Sensor frame. All poses returned by methods of env_state are
  //      expressed relative to this frame.
  // O -- Object frame
  // T -- Table frame

  // Since env_state.get_iiwa_base() returns the pose of the robot's base
  // relative to fixed sensor frame, inverting the returned transform yields the
  // world pose of the fixed sensor frame.
  const Isometry3<double> X_WS{env_state.get_iiwa_base().inverse()};
  const Isometry3<double> X_WO_initial = X_WS * env_state.get_object_pose();

  // Check that the object is oriented correctly.
  if (X_WO_initial.linear()(2, 2) < std::cos(20 * M_PI / 180)) {
    drake::log()->warn(
        "Improper object orientation relative to robot base. Please reset "
        "object and/or check Optitrack markers.");
    return nullopt;
  }

  // Find the destination table.
  const std::vector<Isometry3<double>>& table_poses =
      env_state.get_table_poses();
  int destination_table_index = -1;
  const double max_reach = 1.1;
  double min_angle = std::numeric_limits<double>::infinity();
  for (int i = 0; i < static_cast<int>(table_poses.size()); ++i) {
    const Isometry3<double> X_WT = X_WS * table_poses[i];
    Vector3<double> r_WT_in_xy_plane{X_WT.translation()};
    r_WT_in_xy_plane.z() = 0;
    if (r_WT_in_xy_plane.norm() < max_reach) {
      Vector3<double> r_WO_in_xy_plane{X_WO_initial.translation()};
      r_WO_in_xy_plane.z() = 0;
      const Vector3<double> dir_WO_in_xy_plane{r_WO_in_xy_plane.normalized()};
      double x = r_WT_in_xy_plane.dot(-dir_WO_in_xy_plane);
      double y = (r_WT_in_xy_plane - x * (-dir_WO_in_xy_plane))
                     .dot(Vector3<double>::UnitZ().cross(-dir_WO_in_xy_plane));
      double angle = std::atan2(y, x) + M_PI;
      if (angle > 20 * M_PI / 180 && angle < min_angle) {
        destination_table_index = i;
        min_angle = angle;
      }
    }
  }

  if (destination_table_index < 0) {
    drake::log()->warn("Cannot find a suitable destination table.");
    return nullopt;
  }

  // Pose of destination table in world
  const Isometry3<double> X_WT = X_WS * table_poses.at(destination_table_index);
  const Vector3<double> r_WT = X_WT.translation();

  Vector3<double> dir_TO_final = -X_WT.linear().inverse() * r_WT;
  dir_TO_final.z() = 0;
  dir_TO_final.normalize();
  Vector3<double> r_TO_final = Vector3<double>::Zero();
  r_TO_final.z() += 0.5 * env_state.get_object_dimensions().z();
  Matrix3<double> R_TO_final{Matrix3<double>::Identity()};
  R_TO_final.col(0) = -dir_TO_final;
  R_TO_final.col(2) = Vector3<double>::UnitZ();
  R_TO_final.col(1) = R_TO_final.col(2).cross(R_TO_final.col(0));
  Isometry3<double> X_TO_final;
  X_TO_final.translation() = r_TO_final;
  X_TO_final.linear() = R_TO_final;
  const Isometry3<double> X_WO_final = X_WT * X_TO_final;
  return std::make_pair(X_WO_initial, X_WO_final);
}

optional<std::map<PickAndPlaceState, Isometry3<double>>> ComputeDesiredPoses(
    const WorldState& env_state, double yaw_offset, double pitch_offset) {
  //       (ApproachPickPregrasp,                         (ApproachPlacePregrasp
  //        LiftFromPick ),                                LiftFromPlace)
  //       +--------------------------------------------------------+
  //       |                                                        |
  //       |                                                        |
  //       + (ApproachPick)                         (ApproachPlace) +
  //
  // W  - World frame, coincides with kuka base frame.
  // O  - Object frame
  // Oi - Object frame (initial)
  // Of - Object frame (final)
  // T  - Table frame
  // E  - End-effector frame
  // G  - Grasp frame

  // Position the gripper 30cm above the object before grasp.
  const double pregrasp_offset = 0.3;

  if (auto X_WO_initial_and_final =
          ComputeInitialAndFinalObjectPoses(env_state)) {
    std::map<PickAndPlaceState, Isometry3<double>> X_WG_desired;
    Isometry3<double>& X_WOi = X_WO_initial_and_final->first;
    Isometry3<double>& X_WOf = X_WO_initial_and_final->second;

    X_WOi.rotate(AngleAxis<double>(yaw_offset, Vector3<double>::UnitZ()));

    // A conservative estimate of the fingers' length.
    const double finger_length = 0.07;

    // The grasp frame (G) should be at the center of the object if possible,
    // but no further than finger_length*cos(pitch_offset) from the back edge of
    // the object.
    Isometry3<double> X_OG{Isometry3<double>::Identity()};
    X_OG.rotate(AngleAxis<double>(pitch_offset, Vector3<double>::UnitY()));
    X_OG.translation().x() =
        std::min<double>(0,
                         -0.5 * env_state.get_object_dimensions().x() +
                             finger_length * std::cos(pitch_offset));
    // Set ApproachPick pose.
    Isometry3<double> X_OiO{Isometry3<double>::Identity()};
    X_WG_desired.emplace(PickAndPlaceState::kApproachPick,
                         X_WOi * X_OiO * X_OG);
    // Set ApproachPickPregrasp pose.
    Isometry3<double> X_GGoffset{Isometry3<double>::Identity()};
    X_OiO.setIdentity();
    const double approach_angle = 70.0 * M_PI / 180.0;
    X_OiO.translation()[0] = -cos(approach_angle) * pregrasp_offset;
    X_OiO.translation()[2] = sin(approach_angle) * pregrasp_offset;
    X_WG_desired.emplace(PickAndPlaceState::kApproachPickPregrasp,
                         X_WOi * X_OiO * X_OG * X_GGoffset);
    // Set LiftFromPick pose.
    X_OiO.setIdentity();
    X_OiO.translation()[2] = pregrasp_offset;
    X_WG_desired.emplace(PickAndPlaceState::kLiftFromPick,
                         X_WOi * X_OiO * X_OG);
    // Set ApproachPlace pose.
    Isometry3<double> X_OfO{Isometry3<double>::Identity()};
    X_WG_desired.emplace(PickAndPlaceState::kApproachPlace,
                         X_WOf * X_OfO * X_OG);
    // Set ApproachPlacePregrasp pose.
    X_OfO.setIdentity();
    X_OfO.translation()[2] = pregrasp_offset;
    X_WG_desired.emplace(PickAndPlaceState::kApproachPlacePregrasp,
                         X_WOf * X_OfO * X_OG);
    // Set LiftFromPlace pose.
    X_OfO.setIdentity();
    X_OfO.translation()[0] = -cos(approach_angle) * pregrasp_offset;
    X_OfO.translation()[2] = sin(approach_angle) * pregrasp_offset;
    X_WG_desired.emplace(PickAndPlaceState::kLiftFromPlace,
                         X_WOf * X_OfO * X_OG);
    return X_WG_desired;
  } else {
    return nullopt;
  }
}
}  // namespace

std::ostream& operator<<(std::ostream& os, const PickAndPlaceState value) {
  switch (value) {
    case (PickAndPlaceState::kOpenGripper):
      return os << "kOpenGripper";
    case (PickAndPlaceState::kPlan):
      return os << "kPlan";
    case (PickAndPlaceState::kApproachPickPregrasp):
      return os << "kApproachPickPregrasp";
    case (PickAndPlaceState::kApproachPick):
      return os << "kApproachPick";
    case (PickAndPlaceState::kGrasp):
      return os << "kGrasp";
    case (PickAndPlaceState::kLiftFromPick):
      return os << "kLiftFromPick";
    case (PickAndPlaceState::kApproachPlacePregrasp):
      return os << "kApproachPlacePregrasp";
    case (PickAndPlaceState::kApproachPlace):
      return os << "kApproachPlace";
    case (PickAndPlaceState::kPlace):
      return os << "kPlace";
    case (PickAndPlaceState::kLiftFromPlace):
      return os << "kLiftFromPlace";
    case (PickAndPlaceState::kReset):
      return os << "kReset";
    case (PickAndPlaceState::kDone):
      return os << "kDone";
    default:
      DRAKE_ABORT();
  }
}

PickAndPlaceStateMachine::PickAndPlaceStateMachine(
    const pick_and_place::PlannerConfiguration& configuration, bool single_move)
    : single_move_(single_move),
      state_(PickAndPlaceState::kOpenGripper),
      // Position and rotation tolerances.  These were hand-tuned by
      // adjusting to tighter bounds until IK stopped reliably giving
      // results.
      tight_pos_tol_(0.001, 0.001, 0.001),
      tight_rot_tol_(0.05),
      loose_pos_tol_(0.1, 0.1, 0.1),
      loose_rot_tol_(30 * M_PI / 180),
      configuration_(configuration) {
  std::unique_ptr<RigidBodyTree<double>> robot{BuildTree(configuration_)};
  const int num_positions = robot->get_num_positions();
  joint_names_.resize(num_positions);
  for (int i = 0; i < num_positions; ++i) {
    joint_names_[i] = robot->get_position_name(i);
  }
}

PickAndPlaceStateMachine::~PickAndPlaceStateMachine() {}

void PickAndPlaceStateMachine::Update(const WorldState& env_state,
                                      const IiwaPublishCallback& iiwa_callback,
                                      const WsgPublishCallback& wsg_callback) {
  IKResults ik_res;
  robotlocomotion::robot_plan_t stopped_plan{};
  stopped_plan.num_states = 0;

  PickAndPlaceState next_state{state_};
  auto schunk_action = OpenGripper;
  switch (state_) {
    case PickAndPlaceState::kOpenGripper: {
      next_state = PickAndPlaceState::kPlan;
      break;
    }
    case PickAndPlaceState::kApproachPickPregrasp: {
      next_state = PickAndPlaceState::kApproachPick;
      break;
    }
    case PickAndPlaceState::kApproachPick: {
      next_state = PickAndPlaceState::kGrasp;
      break;
    }
    case PickAndPlaceState::kGrasp: {
      schunk_action = CloseGripper;
      next_state = PickAndPlaceState::kLiftFromPick;
      break;
    }
    case PickAndPlaceState::kLiftFromPick: {
      next_state = PickAndPlaceState::kApproachPlacePregrasp;
      break;
    }
    case PickAndPlaceState::kApproachPlacePregrasp: {
      next_state = PickAndPlaceState::kApproachPlace;
      break;
    }
    case PickAndPlaceState::kApproachPlace: {
      next_state = PickAndPlaceState::kPlace;
      break;
    }
    case PickAndPlaceState::kPlace: {
      next_state = PickAndPlaceState::kLiftFromPlace;
      break;
    }
    case PickAndPlaceState::kLiftFromPlace: {
      next_state = PickAndPlaceState::kReset;
      break;
    }
    default:  // No action needed for other cases.
      break;
  }

  switch (state_) {
    // IIWA arm movements
    case PickAndPlaceState::kApproachPick:
    case PickAndPlaceState::kLiftFromPick:
    case PickAndPlaceState::kApproachPlace:
    case PickAndPlaceState::kLiftFromPlace:
    case PickAndPlaceState::kApproachPickPregrasp:
    case PickAndPlaceState::kApproachPlacePregrasp: {
      if (!iiwa_move_.ActionStarted()) {
        DRAKE_THROW_UNLESS(static_cast<bool>(X_WG_desired_));
        robotlocomotion::robot_plan_t plan{};
        auto robot = BuildTree(configuration_, true /*add_grasp_frame*/);
        auto kinematics_cache = robot->doKinematics(env_state.get_iiwa_q());
        const Isometry3<double> X_WG = robot->CalcBodyPoseInWorldFrame(
            kinematics_cache, *robot->FindBody(kGraspFrameName));
        if (X_WG_desired_->at(state_).isApprox(X_WG, 1e-2)) {
          state_ = next_state;
        } else {
          iiwa_move_.MoveCartesian(env_state, {X_WG, X_WG_desired_->at(state_)},
                                   {0.0, 1.0}, &plan);
          iiwa_callback(&plan);

          drake::log()->info("{} at {}", state_, env_state.get_iiwa_time());
        }
      }
      if (iiwa_move_.ActionFinished(env_state)) {
        // If the object has moved since kPlan, we need to replan.
        state_ = next_state;
        switch (state_) {
          case PickAndPlaceState::kApproachPick:
          case PickAndPlaceState::kApproachPickPregrasp: {
            if (!env_state.get_object_pose().translation().isApprox(
                    expected_object_pose_.translation(), 0.05)) {
              drake::log()->info("Target moved! Re-planning ...");
              state_ = PickAndPlaceState::kPlan;
            }
            break;
          }
          default:  // No action needed for other cases
            break;
        }
        iiwa_move_.Reset();
      }
      break;
    }
    // Schunk gripper actions
    case PickAndPlaceState::kOpenGripper: {
      if (!wsg_act_.ActionStarted()) {
        const Isometry3<double>& obj_pose = env_state.get_object_pose();
        drake::log()->info("Object at: {} {}",
                           obj_pose.translation().transpose(),
                           math::rotmat2rpy(obj_pose.rotation()).transpose());
        const Isometry3<double>& iiwa_pose = env_state.get_iiwa_base();
        drake::log()->info("Base at: {} {}",
                           iiwa_pose.translation().transpose(),
                           math::rotmat2rpy(iiwa_pose.rotation()).transpose());
      }
    }  // Intentionally fall through.
    case PickAndPlaceState::kGrasp:
    case PickAndPlaceState::kPlace: {
      if (!wsg_act_.ActionStarted()) {
        lcmt_schunk_wsg_command msg;
        schunk_action(env_state, configuration_.grip_force, &wsg_act_, &msg);
        wsg_callback(&msg);

        drake::log()->info("{} at {}", state_, env_state.get_iiwa_time());
      }
      if (wsg_act_.ActionFinished(env_state)) {
        state_ = next_state;
        wsg_act_.Reset();
      }
      break;
    }
    case PickAndPlaceState::kPlan: {
      drake::log()->info("{} at {}", state_, env_state.get_iiwa_time());
      // Compute all the desired configurations.
      expected_object_pose_ = env_state.get_object_pose();
      std::unique_ptr<RigidBodyTree<double>> robot{
          BuildTree(configuration_, true /*add_grasp_frame*/)};

      VectorX<double> q_initial{env_state.get_iiwa_q()};
      const double yaw_offset{0.0};
      const double pitch_offset{M_PI / 6};
      X_WG_desired_ = ComputeDesiredPoses(env_state, yaw_offset, pitch_offset);
      state_ = PickAndPlaceState::kApproachPickPregrasp;
      break;
    }
    case PickAndPlaceState::kReset: {
      if (single_move_) {
        state_ = PickAndPlaceState::kDone;
        iiwa_callback(&stopped_plan);
        drake::log()->info("{} at {}", state_, env_state.get_iiwa_time());
      } else {
        state_ = PickAndPlaceState::kOpenGripper;
      }
      break;
    }
    case PickAndPlaceState::kDone: {
      break;
    }
  }
}

}  // namespace pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
