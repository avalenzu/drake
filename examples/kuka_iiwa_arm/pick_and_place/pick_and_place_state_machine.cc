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
#include "drake/manipulation/planner/kinematic_trajectory_optimization.h"
#include "drake/manipulation/util/world_sim_tree_builder.h"
#include "drake/math/orthonormal_basis.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/constraint_wrappers.h"
#include "drake/multibody/joints/fixed_joint.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_ik.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace pick_and_place {
namespace {

using manipulation::planner::BsplineBasis;
using manipulation::planner::BsplineCurve;
using manipulation::planner::KinematicTrajectoryOptimization;
using manipulation::util::WorldSimTreeBuilder;
using systems::plants::KinematicsCacheHelper;
using systems::plants::SingleTimeKinematicConstraintWrapper;

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

optional<std::map<PickAndPlaceState, PiecewisePolynomial<double>>>
ComputeTrajectories(const WorldState& env_state,
                    const PiecewisePolynomial<double>& q_traj_seed,
                    const double orientation_tolerance,
                    const Vector3<double>& position_tolerance,
                    RigidBodyTree<double>* robot) {
  //  Create vectors to hold the constraint objects
  std::vector<std::unique_ptr<Point2LineSegDistConstraint>>
      point_to_line_seg_constraints;
  std::vector<std::unique_ptr<WorldPositionConstraint>> position_constraints;
  std::vector<std::unique_ptr<WorldQuatConstraint>> orientation_constraints;
  std::vector<std::unique_ptr<PostureChangeConstraint>>
      posture_change_constraints;
  std::vector<std::unique_ptr<Point2PointDistanceConstraint>>
      point_to_point_constraints;
  std::vector<std::unique_ptr<WorldGazeDirConstraint>> gaze_dir_constraints;
  std::vector<std::unique_ptr<WorldPositionInFrameConstraint>>
      position_in_frame_constraints;
  std::vector<std::vector<RigidBodyConstraint*>> constraint_arrays;

  std::vector<double> yaw_offsets{0.0};
  std::vector<double> pitch_offsets{M_PI / 8};
  int num_positions = robot->get_num_positions();
  const VectorX<int> joint_indices =
      VectorX<int>::LinSpaced(num_positions, 0, num_positions - 1);

  int grasp_frame_index = robot->FindBodyIndex(kGraspFrameName);
  // int world_idx = robot->FindBodyIndex("world");
  Vector3<double> end_effector_points{0, 0, 0};

  std::vector<PickAndPlaceState> states{PickAndPlaceState::kApproachPick,
                                        PickAndPlaceState::kApproachPlace,
                                        PickAndPlaceState::kLiftFromPlace};
  const double short_duration = 1;
  const double long_duration = 2;
  std::map<PickAndPlaceState, double> per_state_durations{
      {PickAndPlaceState::kApproachPick, long_duration + short_duration},
      {PickAndPlaceState::kApproachPlace, long_duration + 2 * short_duration},
      {PickAndPlaceState::kLiftFromPlace, short_duration}};
  const int num_states = states.size();
  const int num_joints{robot->get_num_positions()};

  // Compute the initial end-effector pose
  const VectorX<double> q_initial{q_traj_seed.value(0)};
  auto kinematics_cache = robot->CreateKinematicsCache();
  kinematics_cache.initialize(q_initial);
  robot->doKinematics(kinematics_cache);

  VectorX<double> t{num_states + 1};
  double duration{0};
  t(0) = 0;
  for (int i = 0; i < num_states; ++i) {
    duration += per_state_durations.at(states[i]);
    t(i + 1) = duration;
  }

  // Construct a vector of KinematicTrajectoryOptimization objects.
  const int spline_order{5};
  int num_control_points{(2 * spline_order + 1)};
  // int num_control_points{120};
  std::vector<KinematicTrajectoryOptimization> programs;
  std::vector<std::unique_ptr<KinematicsCacheHelper<double>>> cache_helpers;

  // Set up an inverse kinematics trajectory problem with one knot for each
  // state
  std::vector<MatrixX<double>> seed_control_points(num_control_points);
  for (int i = 0; i < num_control_points; ++i) {
    seed_control_points[i] =
        q_traj_seed.value(static_cast<double>(i) / num_control_points);
  }

  double plan_time_pre_pick = long_duration / duration;
  double plan_time_pick = (long_duration + short_duration) / duration;
  double plan_time_pick_lift = plan_time_pick + short_duration / duration;
  double plan_time_pre_place =
      plan_time_pick + (short_duration + long_duration) / duration;
  double plan_time_place =
      plan_time_pick + (2 * short_duration + long_duration) / duration;
  double plan_time_place_lift = plan_time_place + short_duration / duration;

  for (double pitch_offset : pitch_offsets) {
    for (double yaw_offset : yaw_offsets) {
      if (auto X_WG_desired =
              ComputeDesiredPoses(env_state, yaw_offset, pitch_offset)) {
        constraint_arrays.emplace_back();
        programs.emplace_back(
            BsplineCurve<double>(BsplineBasis(spline_order, num_control_points),
                                 seed_control_points));
        KinematicTrajectoryOptimization& prog = programs.back();
        prog.set_min_knot_resolution(1e-3);
        prog.set_num_evaluation_points(100);
        prog.set_initial_num_evaluation_points(2);

        // Add costs
        double position_weight{-1};
        double velocity_weight{1};
        double acceleration_weight{-1e-3};
        double jerk_weight{-1};
        auto position_squared_norm =
            prog.position().transpose() * prog.position();
        auto velocity_squared_norm =
            prog.velocity().transpose() * prog.velocity();
        auto acceleration_squared_norm =
            prog.acceleration().transpose() * prog.acceleration();
        auto jerk_squared_norm = prog.jerk().transpose() * prog.jerk();
        symbolic::Expression cost{0};
        if (position_weight > 0) {
          drake::log()->info("Position weight: {}", position_weight);
          cost += position_weight * position_squared_norm(0);
        }
        if (velocity_weight > 0) {
          drake::log()->info("Velocity weight: {}", velocity_weight);
          cost += velocity_weight * velocity_squared_norm(0);
        }
        if (acceleration_weight > 0) {
          drake::log()->info("Acceleration weight: {}", acceleration_weight);
          cost += acceleration_weight * acceleration_squared_norm(0);
        }
        if (jerk_weight > 0) {
          drake::log()->info("Jerk weight: {}", jerk_weight);
          cost += jerk_weight * jerk_squared_norm(0);
        }
        prog.AddQuadraticCost(cost);

        // Add joint limits
        prog.AddLinearConstraint(prog.position() >= robot->joint_limit_min,
                                 {{0.0, 1.0}});
        prog.AddLinearConstraint(prog.position() <= robot->joint_limit_max,
                                 {{0.0, 1.0}});

        // Add velocity limits.
        prog.AddLinearConstraint(
            prog.velocity() <= 0.9 * duration * get_iiwa_max_joint_velocities(),
            {{0.0, 1.0}});
        prog.AddLinearConstraint(
            prog.velocity() >=
                -0.9 * duration * get_iiwa_max_joint_velocities(),
            {{0.0, 1.0}});

        // Constrain initial configuration.
        VectorX<double> zero_vector{VectorX<double>::Zero(num_joints)};
        prog.AddLinearConstraint(prog.position() == q_initial, {{0.0, 0.0}});

        // Constrain initial derivatives
        prog.AddLinearConstraint(prog.velocity() == zero_vector, {{0.0, 0.0}});
        prog.AddLinearConstraint(prog.acceleration() == zero_vector,
                                 {{0.0, 0.0}});
        prog.AddLinearConstraint(prog.jerk() == zero_vector, {{0.0, 0.0}});

        // Constrain final derivatives
        prog.AddLinearConstraint(prog.velocity() == zero_vector, {{1.0, 1.0}});
        prog.AddLinearConstraint(prog.acceleration() == zero_vector,
                                 {{1.0, 1.0}});
        prog.AddLinearConstraint(prog.jerk() == zero_vector, {{1.0, 1.0}});

        // Full stop at pick.
        prog.AddLinearConstraint(prog.velocity() == zero_vector,
                                 {{plan_time_pick, plan_time_pick}});
        prog.AddLinearConstraint(prog.acceleration() == zero_vector,
                                 {{plan_time_pick, plan_time_pick}});
        prog.AddLinearConstraint(prog.jerk() == zero_vector,
                                 {{plan_time_pick, plan_time_pick}});
        // Full stop at place.
        prog.AddLinearConstraint(prog.velocity() == zero_vector,
                                 {{plan_time_place, plan_time_place}});
        prog.AddLinearConstraint(prog.acceleration() == zero_vector,
                                 {{plan_time_place, plan_time_place}});
        prog.AddLinearConstraint(prog.jerk() == zero_vector,
                                 {{plan_time_place, plan_time_place}});

        double threshold_height = 0;
        const Isometry3<double> X_WS{env_state.get_iiwa_base().inverse()};
        for (const auto& X_ST : env_state.get_table_poses()) {
          threshold_height = std::max<double>(threshold_height,
                                              (X_WS * X_ST).translation().z());
        }
        threshold_height += 0.3;

        if (true) {
          drake::log()->debug(
              "Adding height threshold constraint for pre-pick");

          auto X_WC = Isometry3<double>::Identity();
          X_WC.translation().z() = threshold_height;

          Vector3<double> lb{-std::numeric_limits<double>::infinity(),
                             -std::numeric_limits<double>::infinity(),
                             -position_tolerance.z()};
          Vector3<double> ub{std::numeric_limits<double>::infinity(),
                             std::numeric_limits<double>::infinity(),
                             std::numeric_limits<double>::infinity()};
          position_in_frame_constraints.emplace_back(
              new WorldPositionInFrameConstraint(robot, grasp_frame_index,
                                                 end_effector_points,
                                                 X_WC.matrix(), lb, ub));
          constraint_arrays.back().push_back(
              position_in_frame_constraints.back().get());
          cache_helpers.emplace_back(
              new KinematicsCacheHelper<double>(robot->bodies));
          prog.AddGenericPositionConstraint(
              std::make_shared<SingleTimeKinematicConstraintWrapper>(
                  position_in_frame_constraints.back().get(),
                  cache_helpers.back().get()),
              {{plan_time_pre_pick, plan_time_pre_pick}});

          bool done{false};
          bool success{false};
          while (!done) {
            solvers::SolutionResult solution_result =
                prog.Solve(false /*always_update_curve*/);
            drake::log()->info("Solution result: {}", solution_result);
            if (solution_result == solvers::SolutionResult::kSolutionFound) {
              done = !prog.UpdateGenericConstraints();
              // done = true;
              success = done;
            } else {
              done = !prog.AddKnots();
              success = false;
            }
          }
          if (!success) {
            return nullopt;
          }
        }

        if (true) {
          drake::log()->debug(
              "Adding height threshold constraint for pre-place");

          auto X_WC = Isometry3<double>::Identity();
          X_WC.translation().z() = threshold_height;

          Vector3<double> lb{-std::numeric_limits<double>::infinity(),
                             -std::numeric_limits<double>::infinity(),
                             -position_tolerance.z()};
          Vector3<double> ub{std::numeric_limits<double>::infinity(),
                             std::numeric_limits<double>::infinity(),
                             std::numeric_limits<double>::infinity()};
          position_in_frame_constraints.emplace_back(
              new WorldPositionInFrameConstraint(robot, grasp_frame_index,
                                                 end_effector_points,
                                                 X_WC.matrix(), lb, ub));
          constraint_arrays.back().push_back(
              position_in_frame_constraints.back().get());
          cache_helpers.emplace_back(
              new KinematicsCacheHelper<double>(robot->bodies));
          prog.AddGenericPositionConstraint(
              std::make_shared<SingleTimeKinematicConstraintWrapper>(
                  position_in_frame_constraints.back().get(),
                  cache_helpers.back().get()),
              {{plan_time_pre_place, plan_time_pre_place}});

          bool done{false};
          bool success{false};
          while (!done) {
            solvers::SolutionResult solution_result =
                prog.Solve(false /*always_update_curve*/);
            drake::log()->info("Solution result: {}", solution_result);
            if (solution_result == solvers::SolutionResult::kSolutionFound) {
              done = !prog.UpdateGenericConstraints();
              // done = true;
              success = done;
            } else {
              done = !prog.AddKnots();
              success = false;
            }
          }
          if (!success) {
            return nullopt;
          }
        }

        // Add a full stop at the pick pose
        const Isometry3<double> X_WG_pick =
            X_WG_desired->at(PickAndPlaceState::kApproachPick);
        const Vector3<double>& r_WG_pick = X_WG_pick.translation();
        const Quaternion<double>& quat_WG_pick{X_WG_pick.rotation()};

        {
          drake::log()->debug("Adding position constraint for pick.");

          // The grasp frame should hit the target pose at the end of aproach.
          position_constraints.emplace_back(new WorldPositionConstraint(
              robot, grasp_frame_index, end_effector_points,
              r_WG_pick - position_tolerance, r_WG_pick + position_tolerance,
              {plan_time_pick, plan_time_pick}));
          constraint_arrays.back().push_back(position_constraints.back().get());
          cache_helpers.emplace_back(
              new KinematicsCacheHelper<double>(robot->bodies));
          prog.AddGenericPositionConstraint(
              std::make_shared<SingleTimeKinematicConstraintWrapper>(
                  position_constraints.back().get(),
                  cache_helpers.back().get()),
              {{plan_time_pick, plan_time_pick}});

          bool done{false};
          bool success{false};
          while (!done) {
            solvers::SolutionResult solution_result =
                prog.Solve(false /*always_update_curve*/);
            drake::log()->info("Solution result: {}", solution_result);
            if (solution_result == solvers::SolutionResult::kSolutionFound) {
              done = !prog.UpdateGenericConstraints();
              // done = true;
              success = done;
            } else {
              done = !prog.AddKnots();
              success = false;
            }
          }
          if (!success) {
            return nullopt;
          }
        }

        // Constrain final approach and lift
        {
          drake::log()->debug(
              "Adding position constraint for place approach/lift");
          auto X_WC = X_WG_pick;
          X_WC.rotate(
              AngleAxis<double>(-pitch_offset, Vector3<double>::UnitY()));

          Vector3<double> lb{-std::numeric_limits<double>::infinity(),
                             -position_tolerance.y(), -position_tolerance.z()};
          Vector3<double> ub{position_tolerance.x(), position_tolerance.y(),
                             std::numeric_limits<double>::infinity()};
          position_in_frame_constraints.emplace_back(
              new WorldPositionInFrameConstraint(robot, grasp_frame_index,
                                                 end_effector_points,
                                                 X_WC.matrix(), lb, ub));
          constraint_arrays.back().push_back(
              position_in_frame_constraints.back().get());
          cache_helpers.emplace_back(
              new KinematicsCacheHelper<double>(robot->bodies));
          prog.AddGenericPositionConstraint(
              std::make_shared<SingleTimeKinematicConstraintWrapper>(
                  position_in_frame_constraints.back().get(),
                  cache_helpers.back().get()),
              {{plan_time_pre_pick, plan_time_pick_lift}});

          bool done{false};
          bool success{false};
          while (!done) {
            solvers::SolutionResult solution_result =
                prog.Solve(false /*always_update_curve*/);
            drake::log()->info("Solution result: {}", solution_result);
            if (solution_result == solvers::SolutionResult::kSolutionFound) {
              done = !prog.UpdateGenericConstraints();
              // done = true;
              success = done;
            } else {
              done = !prog.AddKnots();
              success = false;
            }
          }
          if (!success) {
            return nullopt;
          }
        }

        // The orientation should remain fixed during the final approach and the
        // lift.
        {
          drake::log()->debug(
              "Adding orientation constraint for place approach/lift");
          orientation_constraints.emplace_back(new WorldQuatConstraint(
              robot, grasp_frame_index,
              Eigen::Vector4d(quat_WG_pick.w(), quat_WG_pick.x(),
                              quat_WG_pick.y(), quat_WG_pick.z()),
              orientation_tolerance, {plan_time_pick, plan_time_pick}));
          constraint_arrays.back().push_back(
              orientation_constraints.back().get());
          cache_helpers.emplace_back(
              new KinematicsCacheHelper<double>(robot->bodies));
          prog.AddGenericPositionConstraint(
              std::make_shared<SingleTimeKinematicConstraintWrapper>(
                  orientation_constraints.back().get(),
                  cache_helpers.back().get()),
              {{plan_time_pre_pick, plan_time_pick_lift}});

          bool done{false};
          bool success{false};
          while (!done) {
            solvers::SolutionResult solution_result =
                prog.Solve(false /*always_update_curve*/);
            drake::log()->info("Solution result: {}", solution_result);
            if (solution_result == solvers::SolutionResult::kSolutionFound) {
              done = !prog.UpdateGenericConstraints();
              // done = true;
              success = done;
            } else {
              done = !prog.AddKnots();
              success = false;
            }
          }
          if (!success) {
            return nullopt;
          }
        }

        // The grasp frame should stay above a safety threshold during the
        // move.
        if (true) {
          drake::log()->debug(
              "Adding height threshold constraint for pick-to-place move.");

          auto X_WC = Isometry3<double>::Identity();
          X_WC.translation().z() = threshold_height;

          Vector3<double> lb{-std::numeric_limits<double>::infinity(),
                             -std::numeric_limits<double>::infinity(),
                             -position_tolerance.z()};
          Vector3<double> ub{std::numeric_limits<double>::infinity(),
                             std::numeric_limits<double>::infinity(),
                             std::numeric_limits<double>::infinity()};
          position_in_frame_constraints.emplace_back(
              new WorldPositionInFrameConstraint(robot, grasp_frame_index,
                                                 end_effector_points,
                                                 X_WC.matrix(), lb, ub));
          constraint_arrays.back().push_back(
              position_in_frame_constraints.back().get());
          cache_helpers.emplace_back(
              new KinematicsCacheHelper<double>(robot->bodies));
          prog.AddGenericPositionConstraint(
              std::make_shared<SingleTimeKinematicConstraintWrapper>(
                  position_in_frame_constraints.back().get(),
                  cache_helpers.back().get()),
              {{plan_time_pick_lift, plan_time_pre_place}});

          bool done{false};
          bool success{false};
          while (!done) {
            solvers::SolutionResult solution_result =
                prog.Solve(false /*always_update_curve*/);
            drake::log()->info("Solution result: {}", solution_result);
            if (solution_result == solvers::SolutionResult::kSolutionFound) {
              done = !prog.UpdateGenericConstraints();
              // done = true;
              success = done;
            } else {
              done = !prog.AddKnots();
              success = false;
            }
          }
          if (!success) {
            return nullopt;
          }
        }

        const Isometry3<double> X_WG_place =
            X_WG_desired->at(PickAndPlaceState::kApproachPlace);
        const Vector3<double>& r_WG_place = X_WG_place.translation();
        const Quaternion<double>& quat_WG_place{X_WG_place.rotation()};

        // Add a full stop at the place pose.
        {
          drake::log()->debug("Adding position constraint for place.");

          position_constraints.emplace_back(new WorldPositionConstraint(
              robot, grasp_frame_index, end_effector_points,
              r_WG_place - position_tolerance, r_WG_place + position_tolerance,
              {plan_time_place, plan_time_place}));
          constraint_arrays.back().push_back(position_constraints.back().get());
          cache_helpers.emplace_back(
              new KinematicsCacheHelper<double>(robot->bodies));
          prog.AddGenericPositionConstraint(
              std::make_shared<SingleTimeKinematicConstraintWrapper>(
                  position_constraints.back().get(),
                  cache_helpers.back().get()),
              {{plan_time_place, plan_time_place}});

          bool done{false};
          bool success{false};
          while (!done) {
            solvers::SolutionResult solution_result =
                prog.Solve(false /*always_update_curve*/);
            drake::log()->info("Solution result: {}", solution_result);
            if (solution_result == solvers::SolutionResult::kSolutionFound) {
              done = !prog.UpdateGenericConstraints();
              // done = true;
              success = done;
            } else {
              done = !prog.AddKnots();
              success = false;
            }
          }
          if (!success) {
            return nullopt;
          }
        }

        // Constrain final approach and lift
        if (true) {
          drake::log()->debug(
              "Adding position constraint for place approach/lift");
          auto X_WC = X_WG_place;
          X_WC.rotate(
              AngleAxis<double>(-pitch_offset, Vector3<double>::UnitY()));

          Vector3<double> lb{-std::numeric_limits<double>::infinity(),
                             -position_tolerance.y(), -position_tolerance.z()};
          Vector3<double> ub{position_tolerance.x(), position_tolerance.y(),
                             std::numeric_limits<double>::infinity()};
          position_in_frame_constraints.emplace_back(
              new WorldPositionInFrameConstraint(robot, grasp_frame_index,
                                                 end_effector_points,
                                                 X_WC.matrix(), lb, ub));
          constraint_arrays.back().push_back(
              position_in_frame_constraints.back().get());
          cache_helpers.emplace_back(
              new KinematicsCacheHelper<double>(robot->bodies));
          prog.AddGenericPositionConstraint(
              std::make_shared<SingleTimeKinematicConstraintWrapper>(
                  position_in_frame_constraints.back().get(),
                  cache_helpers.back().get()),
              {{plan_time_pre_place, plan_time_place_lift}});

          bool done{false};
          bool success{false};
          while (!done) {
            solvers::SolutionResult solution_result =
                prog.Solve(false /*always_update_curve*/);
            drake::log()->info("Solution result: {}", solution_result);
            if (solution_result == solvers::SolutionResult::kSolutionFound) {
              done = !prog.UpdateGenericConstraints();
              // done = true;
              success = done;
            } else {
              done = !prog.AddKnots();
              success = false;
            }
          }
          if (!success) {
            return nullopt;
          }
        }

        if (true) {
          drake::log()->debug(
              "Adding orientation constraint for place approach/lift");
          // The orientation should remain fixed during the final approach.
          orientation_constraints.emplace_back(new WorldQuatConstraint(
              robot, grasp_frame_index,
              Eigen::Vector4d(quat_WG_place.w(), quat_WG_place.x(),
                              quat_WG_place.y(), quat_WG_place.z()),
              orientation_tolerance, {plan_time_place, plan_time_place}));
          constraint_arrays.back().push_back(
              orientation_constraints.back().get());
          cache_helpers.emplace_back(
              new KinematicsCacheHelper<double>(robot->bodies));
          prog.AddGenericPositionConstraint(
              std::make_shared<SingleTimeKinematicConstraintWrapper>(
                  orientation_constraints.back().get(),
                  cache_helpers.back().get()),
              {{plan_time_pre_place, plan_time_place_lift}});

          bool done{false};
          bool success{false};
          while (!done) {
            solvers::SolutionResult solution_result =
                prog.Solve(false /*always_update_curve*/);
            drake::log()->info("Solution result: {}", solution_result);
            if (solution_result == solvers::SolutionResult::kSolutionFound) {
              done = !prog.UpdateGenericConstraints();
              // done = true;
              success = done;
            } else {
              done = !prog.AddKnots();
              success = false;
            }
          }
          if (!success) {
            return nullopt;
          }
        }

        if (true) {
          drake::log()->debug(
              "Adding height threshold constraint for lift-from-place");

          auto X_WC = Isometry3<double>::Identity();
          X_WC.translation().z() = threshold_height;

          Vector3<double> lb{-std::numeric_limits<double>::infinity(),
                             -std::numeric_limits<double>::infinity(),
                             -position_tolerance.z()};
          Vector3<double> ub{std::numeric_limits<double>::infinity(),
                             std::numeric_limits<double>::infinity(),
                             std::numeric_limits<double>::infinity()};
          position_in_frame_constraints.emplace_back(
              new WorldPositionInFrameConstraint(robot, grasp_frame_index,
                                                 end_effector_points,
                                                 X_WC.matrix(), lb, ub));
          constraint_arrays.back().push_back(
              position_in_frame_constraints.back().get());
          cache_helpers.emplace_back(
              new KinematicsCacheHelper<double>(robot->bodies));
          prog.AddGenericPositionConstraint(
              std::make_shared<SingleTimeKinematicConstraintWrapper>(
                  position_in_frame_constraints.back().get(),
                  cache_helpers.back().get()),
              {{plan_time_place_lift, plan_time_place_lift}});

          bool done{false};
          bool success{false};
          while (!done) {
            solvers::SolutionResult solution_result =
                prog.Solve(false /*always_update_curve*/);
            drake::log()->info("Solution result: {}", solution_result);
            if (solution_result == solvers::SolutionResult::kSolutionFound) {
              done = !prog.UpdateGenericConstraints();
              // done = true;
              success = done;
            } else {
              done = !prog.AddKnots();
              success = false;
            }
          }
          if (!success) {
            return nullopt;
          }
        }
      }
    }
  }

  if (constraint_arrays.empty()) return nullopt;

  PiecewisePolynomial<double> q_traj;
  bool success = false;
  for (auto& prog : programs) {
    bool done{false};
    while (!done) {
      solvers::SolutionResult solution_result =
          prog.Solve(false /*always_update_curve*/);
      drake::log()->info("Solution result: {}", solution_result);
      if (solution_result == solvers::SolutionResult::kSolutionFound) {
        done = !prog.UpdateGenericConstraints();
        // done = true;
        success = done;
      } else {
        done = !prog.AddKnots();
        success = false;
      }
    }
    if (success) {
      q_traj = prog.GetPositionSolution(duration);
      break;
    }
  }
  if (!success) {
    return nullopt;
  }
  std::map<PickAndPlaceState, PiecewisePolynomial<double>>
      interpolation_result_map;
  for (int i = 0; i < num_states; ++i) {
    std::vector<double> breaks;
    std::vector<MatrixX<double>> knots;
    drake::log()->trace("Trajectory for {}:", states[i]);
    // Upsample trajectory since we don't actually send the PiecwisePolynomial.
    VectorX<double> t_eval = VectorX<double>::LinSpaced(100, t(i), t(i + 1));
    breaks.resize(t_eval.size());
    knots.resize(t_eval.size());
    for (int j = 0; j < t_eval.size(); ++j) {
      breaks[j] = t_eval(j) - t_eval(0);
      knots[j] = q_traj.value(t_eval(j));
      drake::log()->trace("t = {}, q = {}:", breaks.back(),
                          knots.back().transpose());
    }
    const auto knot_dot_zero = VectorX<double>::Zero(num_positions);
    interpolation_result_map.emplace(
        states[i], PiecewisePolynomial<double>::Cubic(
                       breaks, knots, knot_dot_zero, knot_dot_zero));
  }
  return interpolation_result_map;
}  // namespace

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
      tight_pos_tol_(0.01, 0.01, 0.01),
      tight_rot_tol_(0.05),
      loose_pos_tol_(0.05, 0.05, 0.05),
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
    case PickAndPlaceState::kApproachPick: {
      next_state = PickAndPlaceState::kGrasp;
      break;
    }
    case PickAndPlaceState::kGrasp: {
      schunk_action = CloseGripper;
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
    case PickAndPlaceState::kApproachPlace:
    case PickAndPlaceState::kLiftFromPlace: {
      if (!iiwa_move_.ActionStarted()) {
        DRAKE_THROW_UNLESS(static_cast<bool>(interpolation_result_map_));
        robotlocomotion::robot_plan_t plan{};
        std::vector<VectorX<double>> q;
        PiecewisePolynomial<double>& q_traj =
            interpolation_result_map_->at(state_);
        const std::vector<double>& times{q_traj.getSegmentTimes()};
        q.reserve(times.size());
        for (double t : times) {
          q.push_back(q_traj.value(t));
        }

        iiwa_move_.MoveJoints(env_state, joint_names_, times, q, &plan);
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
        schunk_action(env_state, &wsg_act_, &msg);
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
      interpolation_result_map_ = ComputeTrajectories(
          env_state,
          q_traj_seed_.value_or(PiecewisePolynomial<double>::ZeroOrderHold(
              {0.0, 1.0}, {q_initial, q_initial})),
          tight_rot_tol_, tight_pos_tol_, robot.get());
      if (interpolation_result_map_) {
        // Proceed to execution.
        state_ = PickAndPlaceState::kApproachPick;
        planning_failure_count_ = 0;
      } else {
        // otherwise re-plan on next call to Update.
        drake::log()->warn("Attempt {} failed", planning_failure_count_);
        // Set a random seed for the next call to ComputeTrajectories.
        VectorX<double> q_seed = robot->getRandomConfiguration(rand_generator_);
        q_traj_seed_.emplace(PiecewisePolynomial<double>::FirstOrderHold(
            {0.0, 1.0}, {q_initial, q_seed}));
        ++planning_failure_count_;
      }
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
    default: {
      DRAKE_ABORT_MSG("Bad state!");
      break;
    }
  }
}

}  // namespace pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
