#include "drake/examples/kuka_iiwa_arm/pick_and_place/pick_and_place_state_machine.h"

#include <algorithm>
#include <limits>
#include <random>
#include <string>

#include <spdlog/fmt/ostr.h>

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"
#include "drake/common/trajectories/piecewise_quaternion.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
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

void OpenGripper(const WorldState& env_state, WsgAction* wsg_act,
                 lcmt_schunk_wsg_command* msg) {
  wsg_act->OpenGripper(env_state, msg);
}

void CloseGripper(const WorldState& env_state, WsgAction* wsg_act,
                  lcmt_schunk_wsg_command* msg) {
  wsg_act->CloseGripper(env_state, msg);
}

std::unique_ptr<RigidBodyTree<double>> BuildTree(
    const std::string& model_path) {
  WorldSimTreeBuilder<double> tree_builder;
  tree_builder.StoreModel("iiwa", model_path);
  tree_builder.AddFixedModelInstance("iiwa", Vector3<double>::Zero());
  return tree_builder.Build();
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

  // Check that the object is oriented correctly
  if (X_WO_initial.linear()(2, 2) < std::cos(20 * M_PI / 180)) {
    drake::log()->warn(
        "Improper object orientation relative to robot base. Please reset "
        "object and/or check Optitrack markers.");
    return nullopt;
  }

  // Find the destination table
  const std::vector<Isometry3<double>>& table_poses =
      env_state.get_table_poses();
  int destination_table_index = -1;
  const double kMaxReach = 1.1;
  double min_angle = std::numeric_limits<double>::infinity();
  for (int i = 0; i < static_cast<int>(table_poses.size()); ++i) {
    const Isometry3<double> X_WT = X_WS * table_poses[i];
    Vector3<double> r_WT_in_xy_plane{X_WT.translation()};
    r_WT_in_xy_plane.z() = 0;
    if (r_WT_in_xy_plane.norm() < kMaxReach) {
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
/**
              LiftFromPick 2───────────────────────────3 ApproachPlacePregrasp
                           │                           │
   ApproachPickPregrasp 0  │                           │  5 LiftFromPlace
                         ╲ │                           │ ╱
                          ╲│                           │╱
              ApproachPick 1                           4 ApproachPlacePregrasp
**/
optional<std::map<PickAndPlaceState, Isometry3<double>>> ComputeDesiredPoses(
    const WorldState& env_state, double yaw_offset, double pitch_offset) {
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
        std::min<double>(0, -0.5 * env_state.get_object_dimensions().x() +
                                finger_length * std::cos(pitch_offset));
    // Set ApproachPick pose
    Isometry3<double> X_OiO{Isometry3<double>::Identity()};
    X_WG_desired.emplace(PickAndPlaceState::kApproachPick,
                         X_WOi * X_OiO * X_OG);
    // Set ApproachPickPregrasp pose
    Isometry3<double> X_GGoffset{Isometry3<double>::Identity()};
    X_OiO.setIdentity();
    const double approach_angle = 70.0 * M_PI / 180.0;
    X_OiO.translation()[0] = -cos(approach_angle) * pregrasp_offset;
    X_OiO.translation()[2] = sin(approach_angle) * pregrasp_offset;
    X_WG_desired.emplace(PickAndPlaceState::kApproachPickPregrasp,
                         X_WOi * X_OiO * X_OG * X_GGoffset);
    // Set LiftFromPick pose
    X_OiO.setIdentity();
    X_OiO.translation()[2] = pregrasp_offset;
    X_WG_desired.emplace(PickAndPlaceState::kLiftFromPick,
                         X_WOi * X_OiO * X_OG);
    // Set ApproachPlace pose
    Isometry3<double> X_OfO{Isometry3<double>::Identity()};
    X_WG_desired.emplace(PickAndPlaceState::kApproachPlace,
                         X_WOf * X_OfO * X_OG);
    // Set ApproachPlacePregrasp pose
    X_OfO.setIdentity();
    X_OfO.translation()[2] = pregrasp_offset;
    X_WG_desired.emplace(PickAndPlaceState::kApproachPlacePregrasp,
                         X_WOf * X_OfO * X_OG);
    // Set LiftFromPlace pose
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

optional<std::map<PickAndPlaceState, PiecewisePolynomial<double>>>
ComputeTrajectories(const WorldState& env_state,
                    const PiecewisePolynomial<double>& q_traj_seed,
                    const double orientation_tolerance,
                    const Vector3<double>& position_tolerance,
                    RigidBodyTree<double>* robot) {
  //  Create vectors to hold the constraint objects
  std::vector<std::unique_ptr<WorldPositionConstraint>> position_constraints;
  std::vector<std::unique_ptr<WorldQuatConstraint>> orientation_constraints;
  std::vector<std::unique_ptr<PostureChangeConstraint>>
      posture_change_constraints;
  std::vector<std::unique_ptr<Point2PointDistanceConstraint>>
      point_to_point_constraints;
  std::vector<std::unique_ptr<WorldGazeDirConstraint>> gaze_dir_constraints;
  std::vector<std::vector<RigidBodyConstraint*>> constraint_arrays;
  std::vector<double> yaw_offsets{M_PI, 0.0};
  std::vector<double> pitch_offsets{M_PI / 8};
  int num_positions = robot->get_num_positions();
  const VectorX<int> joint_indices =
      VectorX<int>::LinSpaced(num_positions, 0, num_positions - 1);

  int grasp_frame_index = robot->FindBodyIndex(kGraspFrameName);
  int world_idx = robot->FindBodyIndex("world");
  Vector3<double> end_effector_points{0, 0, 0};

  std::vector<PickAndPlaceState> states{
      PickAndPlaceState::kApproachPickPregrasp,
      PickAndPlaceState::kApproachPick,
      PickAndPlaceState::kLiftFromPick,
      PickAndPlaceState::kApproachPlacePregrasp,
      PickAndPlaceState::kApproachPlace,
      PickAndPlaceState::kLiftFromPlace};
  const double kShortDuration = 1;
  const double kLongDuration = 3;
  const double kFewKnots = 3;
  const double kManyKnots = 5;
  std::map<PickAndPlaceState, double> per_state_durations{
      {PickAndPlaceState::kApproachPickPregrasp, kLongDuration},
      {PickAndPlaceState::kApproachPick, kShortDuration},
      {PickAndPlaceState::kLiftFromPick, kShortDuration},
      {PickAndPlaceState::kApproachPlacePregrasp, kLongDuration},
      {PickAndPlaceState::kApproachPlace, kShortDuration},
      {PickAndPlaceState::kLiftFromPlace, kShortDuration}};
  std::map<PickAndPlaceState, double> per_state_number_of_knots{
      {PickAndPlaceState::kApproachPickPregrasp, kManyKnots},
      {PickAndPlaceState::kApproachPick, kFewKnots},
      {PickAndPlaceState::kLiftFromPick, kFewKnots},
      {PickAndPlaceState::kApproachPlacePregrasp, kManyKnots},
      {PickAndPlaceState::kApproachPlace, kFewKnots},
      {PickAndPlaceState::kLiftFromPlace, kFewKnots}};
  const int num_states = states.size();
  int kNumKnots = 1;
  for (int i = 0; i < num_states; ++i) {
    kNumKnots += per_state_number_of_knots.at(states[i]);
  }

  // Compute the initial end-effector pose
  const VectorX<double> q_initial{q_traj_seed.value(0)};
  auto kinematics_cache = robot->CreateKinematicsCache();
  kinematics_cache.initialize(q_initial);
  robot->doKinematics(kinematics_cache);

  VectorX<double> t{kNumKnots};
  double duration{0};
  t(0) = 0;
  int index{1};
  for (int i = 0; i < num_states; ++i) {
    int num_knots_per_state = per_state_number_of_knots.at(states[i]);
    for (int j = 1; j <= num_knots_per_state; ++j) {
      duration += per_state_durations.at(states[i]) / num_knots_per_state;
      t(index++) = duration;
    }
  }
  // Set up an inverse kinematics trajectory problem with one knot for each
  // state
  MatrixX<double> q_nom =
      MatrixX<double>::Zero(robot->get_num_positions(), kNumKnots);

  for (double pitch_offset : pitch_offsets) {
    for (double yaw_offset : yaw_offsets) {
      if (auto X_WG_desired =
              ComputeDesiredPoses(env_state, yaw_offset, pitch_offset)) {
        constraint_arrays.emplace_back();

        Isometry3<double> X_WG_initial = robot->relativeTransform(
            kinematics_cache, world_idx, grasp_frame_index);
        int start_knot = 0;
        for (int i = 0; i < num_states; ++i) {
          const PickAndPlaceState state{states[i]};
          int num_knots_per_state = per_state_number_of_knots.at(states[i]);
          const int& end_knot = start_knot + num_knots_per_state;
          const double& start_time = t(start_knot);
          const double& end_time = t(end_knot);
          const Vector2<double> intermediate_tspan{start_time, end_time};
          const Vector2<double> final_tspan{end_time, end_time};

          // Extract desired position and orientation of end effector at the
          // given state.
          const Isometry3<double>& X_WG_final = X_WG_desired->at(state);
          const Vector3<double>& r_WG_final = X_WG_final.translation();
          const Quaternion<double>& quat_WG_final{X_WG_final.rotation()};
          const Vector3<double>& r_WG_initial = X_WG_initial.translation();

          // Constrain the end-effector position and orientation at the end of
          // this state.
          position_constraints.emplace_back(new WorldPositionConstraint(
              robot, grasp_frame_index, end_effector_points,
              r_WG_final - position_tolerance, r_WG_final + position_tolerance,
              final_tspan));
          constraint_arrays.back().push_back(position_constraints.back().get());
          orientation_constraints.emplace_back(new WorldQuatConstraint(
              robot, grasp_frame_index,
              Eigen::Vector4d(quat_WG_final.w(), quat_WG_final.x(),
                              quat_WG_final.y(), quat_WG_final.z()),
              orientation_tolerance, final_tspan));
          constraint_arrays.back().push_back(
              orientation_constraints.back().get());

          double intermediate_orientation_tolerance = orientation_tolerance;
          Vector3<double> intermediate_position_tolerance = position_tolerance;
          if (state == PickAndPlaceState::kApproachPlacePregrasp ||
              (state == PickAndPlaceState::kApproachPickPregrasp &&
               (r_WG_final - r_WG_initial).norm() > 0.2)) {
            intermediate_orientation_tolerance = 30 * M_PI / 180.0;
            intermediate_position_tolerance = Vector3<double>::Constant(0.1);
          }

          // Constrain the position of the grasp frame at intermediate points.
          // The desired position interpolates between the intial and final
          // positions unless it comes within base_avoidance_threshold of the
          // origin, in which case it is projected out to the surface of the
          // positive-z hemi-sphere centered at the origin with radius
          // base_avoidance_threshold.
          auto r_WG_traj = PiecewisePolynomial<double>::Pchip(
              {start_time, end_time}, {r_WG_initial, r_WG_final}, true);
          constexpr double base_avoidance_threshold{0.6};
          for (int j = start_knot + 1; j < end_knot; ++j) {
            Vector3<double> r_WG_intermediate = r_WG_traj.value(t(j));
            if (r_WG_intermediate.norm() < base_avoidance_threshold) {
              r_WG_intermediate.normalize();
              r_WG_intermediate *= base_avoidance_threshold;
              r_WG_intermediate.z() = std::abs<double>(r_WG_intermediate.z());
            }
            position_constraints.emplace_back(new WorldPositionConstraint(
                robot, grasp_frame_index, end_effector_points,
                r_WG_intermediate - intermediate_position_tolerance,
                r_WG_intermediate + intermediate_position_tolerance,
                {t(j), t(j)}));
            constraint_arrays.back().push_back(
                position_constraints.back().get());
          }

          // If the intial and final end-effector orientations are close to each
          // other (to within the orientation tolerance), fix the orientation
          // for all via points. Otherwise, only allow the end-effector to
          // rotate about the axis defining the rotation between the initial and
          // final orientations.
          // Find axis-angle representation of the rotation from X_WG_initial to
          // X_WG_final.
          Isometry3<double> X_WG_delta = X_WG_final.inverse() * X_WG_initial;
          Eigen::AngleAxis<double> aaxis{X_WG_delta.linear()};
          Vector3<double> axis_E{aaxis.axis()};
          Vector3<double> dir_W{X_WG_initial.linear() * axis_E};
          if (std::abs(aaxis.angle()) < orientation_tolerance) {
            orientation_constraints.emplace_back(new WorldQuatConstraint(
                robot, grasp_frame_index,
                Eigen::Vector4d(quat_WG_final.w(), quat_WG_final.x(),
                                quat_WG_final.y(), quat_WG_final.z()),
                intermediate_orientation_tolerance, intermediate_tspan));
            constraint_arrays.back().push_back(
                orientation_constraints.back().get());
          } else {
            gaze_dir_constraints.emplace_back(new WorldGazeDirConstraint(
                robot, grasp_frame_index, axis_E, dir_W,
                intermediate_orientation_tolerance, intermediate_tspan));
            constraint_arrays.back().push_back(
                gaze_dir_constraints.back().get());
          }

          // For each pair of adjacent knots, add a constraint on the change in
          // joint positions.
          for (int j = start_knot + 1; j <= end_knot; ++j) {
            drake::log()->trace("j = {}", j);
            const double dt{t(j) - t(j - 1)};
            VectorX<double> ub_change = 0.9 * dt * kIiwaMaxJointVelocities;
            if ((r_WG_final - r_WG_initial).norm() <
                position_tolerance.norm()) {
              ub_change *= 0.1;
            }
            VectorX<double> lb_change = -ub_change;
            const Vector2<double> segment_tspan{t(j - 1), t(j)};
            posture_change_constraints.emplace_back(new PostureChangeConstraint(
                robot, joint_indices, lb_change, ub_change, segment_tspan));
            constraint_arrays.back().push_back(
                posture_change_constraints.back().get());
          }

          // Reset X_WG_initial for the next iteration.
          X_WG_initial = X_WG_final;
          start_knot += num_knots_per_state;
        }
      }
    }
  }

  if (constraint_arrays.empty()) return nullopt;

  // Solve the IK problem.
  IKResults ik_res;
  IKoptions ikoptions(robot);
  ikoptions.setFixInitialState(true);
  ikoptions.setMajorIterationsLimit(5e2);
  // ikoptions.setIterationsLimit(1e5);
  ikoptions.setQ(MatrixX<double>::Zero(num_positions, num_positions));
  ikoptions.setQa(MatrixX<double>::Identity(num_positions, num_positions));
  // ikoptions.setQv(MatrixX<double>::Identity(num_positions,
  // num_positions));
  bool success = false;
  for (const auto& constraint_array : constraint_arrays) {
    MatrixX<double> q_knots_seed{robot->get_num_positions(), kNumKnots};
    for (int j = 0; j < kNumKnots; ++j) {
      q_knots_seed.col(j) = q_traj_seed.value(t(j) / duration);
    }
    ik_res = inverseKinTrajSimple(robot, t, q_knots_seed, q_nom,
                                  constraint_array, ikoptions);
    success = ik_res.info[0] == 1;
    // success = true;
    if (success) {
      break;
    }
  }
  if (!success) {
    return nullopt;
  }
  std::map<PickAndPlaceState, PiecewisePolynomial<double>>
      interpolation_result_map;
  std::vector<double> times;
  for (int i = 0; i < kNumKnots; ++i) times.push_back(t(i));
  int start_knot = 0;
  for (int i = 0; i < num_states; ++i) {
    int num_knots_per_state = per_state_number_of_knots.at(states[i]);
    const int& end_knot = start_knot + num_knots_per_state;
    std::vector<double> breaks;
    std::vector<MatrixX<double>> knots;
    drake::log()->trace("Trajectory for {}:", states[i]);
    for (int j = start_knot; j <= end_knot; ++j) {
      breaks.push_back(t(j) - t(start_knot));
      knots.push_back(ik_res.q_sol[j]);
      drake::log()->trace("t = {}, q = {}:", breaks.back(),
                          knots.back().transpose());
    }
    const auto knot_dot_zero = VectorX<double>::Zero(num_positions);
    interpolation_result_map.emplace(
        states[i], PiecewisePolynomial<double>::Cubic(
                       breaks, knots, knot_dot_zero, knot_dot_zero));
    start_knot = end_knot;
  }
  return interpolation_result_map;
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
      configuration_(configuration) {}

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
    } break;
    case PickAndPlaceState::kApproachPickPregrasp: {
      next_state = PickAndPlaceState::kApproachPick;
    } break;
    case PickAndPlaceState::kApproachPick: {
      next_state = PickAndPlaceState::kGrasp;
    } break;
    case PickAndPlaceState::kGrasp: {
      schunk_action = CloseGripper;
      next_state = PickAndPlaceState::kLiftFromPick;
    } break;
    case PickAndPlaceState::kLiftFromPick: {
      next_state = PickAndPlaceState::kApproachPlacePregrasp;
    } break;
    case PickAndPlaceState::kApproachPlacePregrasp: {
      next_state = PickAndPlaceState::kApproachPlace;
    } break;
    case PickAndPlaceState::kApproachPlace: {
      next_state = PickAndPlaceState::kPlace;
    } break;
    case PickAndPlaceState::kPlace: {
      next_state = PickAndPlaceState::kLiftFromPlace;
    } break;
    case PickAndPlaceState::kLiftFromPlace: {
      next_state = PickAndPlaceState::kReset;
    } break;
    default:  // No action needed for other cases
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
        DRAKE_THROW_UNLESS(bool(interpolation_result_map_));
        robotlocomotion::robot_plan_t plan{};
        std::vector<VectorX<double>> q;
        PiecewisePolynomial<double> q_traj =
            interpolation_result_map_->at(state_);
        const VectorX<double> q_0 =
            q_traj.value(q_traj.getSegmentTimes().front());
        const VectorX<double> q_f =
            q_traj.value(q_traj.getSegmentTimes().back());
        const double kExtraShortDuration{0.5};
        if ((q_f - q_0).array().abs().maxCoeff() < 10 * M_PI / 180) {
          drake::log()->debug("Very short move!");
          q_traj = PiecewisePolynomial<double>::Pchip({0, kExtraShortDuration},
                                                      {q_0, q_f}, true);
        }
        // Upsample the trajectory.
        const int num_plan_points_per_segment{1};
        const double& t_0 = q_traj.getSegmentTimes().front();
        const double& t_f = q_traj.getSegmentTimes().back();
        const int num_times{
            num_plan_points_per_segment * q_traj.getNumberOfSegments() + 1};
        std::vector<double> times(num_times);
        for (int i = 0; i < num_times; ++i) {
          times[i] = i * (t_f - t_0) / static_cast<double>(num_times - 1);
        }
        q.reserve(times.size());
        for (auto& t : times) {
          q.push_back(q_traj.value(t));
        }
        std::unique_ptr<RigidBodyTree<double>> robot{
            BuildTree(configuration_.model_path)};

        iiwa_move_.MoveJoints(env_state, *robot, times, q, &plan);
        iiwa_callback(&plan);

        drake::log()->info("{} at {}", state_, env_state.get_iiwa_time());
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
              interpolation_result_map_->clear();
              state_ = PickAndPlaceState::kPlan;
            }
          } break;
          default:  // No action needed for other cases
            break;
        }
        iiwa_move_.Reset();
      }
    } break;
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
    }  // Intentionally fall through
    case PickAndPlaceState::kGrasp:
    case PickAndPlaceState::kPlace: {
      if (!wsg_act_.ActionStarted()) {
        lcmt_schunk_wsg_command msg;
        schunk_action(env_state, &wsg_act_, &msg);
        wsg_callback(&msg);

        drake::log()->info("{} at {}", state_, env_state.get_iiwa_time());
      }
      if (wsg_act_.ActionFinished(env_state)) {
        state_ = next_state;
        wsg_act_.Reset();
      }
    } break;
    case PickAndPlaceState::kPlan: {
      drake::log()->info("{} at {}", state_, env_state.get_iiwa_time());
      // Compute all the desired configurations
      expected_object_pose_ = env_state.get_object_pose();
      std::unique_ptr<RigidBodyTree<double>> robot{
          BuildTree(configuration_.model_path)};

      // Add the grasp frame as a RigidBody. This allows it to be used in IK
      // constraints.
      // TODO(avalenzu): Add a planning model for the gripper that includes
      // the grasp frame as a named frame.
      auto grasp_frame = std::make_unique<RigidBody<double>>();
      grasp_frame->set_name(kGraspFrameName);
      // The gripper (and therfore the grasp frame) is rotated relative to the
      // end effector link.
      const double grasp_frame_angular_offset{-M_PI / 8};
      // The grasp frame is located between the fingertips of the gripper
      // which is grasp_frame_translational_offset from the origin of the
      // end-effector link.
      const double grasp_frame_translational_offset{0.19};
      Isometry3<double> X_EG{Isometry3<double>::Identity()};
      X_EG.rotate(Eigen::AngleAxisd(grasp_frame_angular_offset,
                                    Eigen::Vector3d::UnitX()));
      X_EG.translation().x() = grasp_frame_translational_offset;
      std::string grasp_frame_joint_name = kGraspFrameName;
      grasp_frame_joint_name += "_joint";
      auto grasp_frame_fixed_joint =
          std::make_unique<FixedJoint>(grasp_frame_joint_name, X_EG);
      grasp_frame->add_joint(robot->FindBody(configuration_.end_effector_name),
                             std::move(grasp_frame_fixed_joint));
      robot->add_rigid_body(std::move(grasp_frame));
      robot->compile();

      VectorX<double> q_initial{env_state.get_iiwa_q()};
      double duration = 1.0;
      interpolation_result_map_ = ComputeTrajectories(
          env_state,
          q_traj_seed_.value_or(PiecewisePolynomial<double>::ZeroOrderHold(
              {0.0, duration}, {q_initial, q_initial})),
          tight_rot_tol_, tight_pos_tol_, robot.get());
      if (interpolation_result_map_) {
        // Proceed to execution
        state_ = PickAndPlaceState::kApproachPickPregrasp;
        planning_failure_count_ = 0;
        q_traj_seed_ = nullopt;
      } else {
        // Otherwise, re-plan on next call to Update.
        drake::log()->warn("Attempt {} failed", planning_failure_count_);
        // Set a random seed for the next call to ComputeTrajectories.
        VectorX<double> q_seed = robot->getRandomConfiguration(rand_generator_);
        std::vector<MatrixX<double>> q_knots;
        std::vector<double> t_breaks;
        q_knots.push_back(q_initial);
        t_breaks.push_back(0);
        const int num_seed_knots = 2;
        for (int i = 1; i < num_seed_knots; ++i) {
          q_knots.push_back(robot->getRandomConfiguration(rand_generator_));
          t_breaks.push_back(i * duration /
                             static_cast<double>(num_seed_knots - 1));
        }
        q_traj_seed_.emplace(
            PiecewisePolynomial<double>::FirstOrderHold(t_breaks, q_knots));
        ++planning_failure_count_;
      }
    } break;
    case PickAndPlaceState::kReset: {
      if (single_move_) {
        state_ = PickAndPlaceState::kDone;
        iiwa_callback(&stopped_plan);
        drake::log()->info("{} at {}", state_, env_state.get_iiwa_time());
      } else {
        state_ = PickAndPlaceState::kOpenGripper;
      }
    } break;
    case PickAndPlaceState::kDone: {
    } break;
  }
}

}  // namespace pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
