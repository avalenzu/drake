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

using manipulation::util::WorldSimTreeBuilder;
using manipulation::planner::KinematicTrajectoryOptimization;
using manipulation::planner::BsplineBasis;
using manipulation::planner::BsplineCurve;
using systems::plants::KinematicsCacheHelper;
using systems::plants::SingleTimeKinematicConstraintWrapper;

const char kGraspFrameName[] = "grasp_frame";

// Generates a joint-space trajectory for @p robot that interpolates between the
// inital and final configurations specified in @p request. If @p
// straight_line_motion is true, the end-effector position at all knot points of
// the returned trajectory will lie on the line between the end-effector
// positions at the initial and final configurations.
//PostureInterpolationResult PlanInterpolatingMotion(
    //const PostureInterpolationRequest& request, RigidBodyTree<double>* robot,
    //bool straight_line_motion = true) {
  //// Create local references to request member variables.
  //const VectorX<double>& q_initial{request.q_initial};
  //const VectorX<double>& q_final{request.q_final};
  //const std::vector<double>& times{request.times};
  //const double& position_tolerance{request.position_tolerance};
  //const double& orientation_tolerance{request.orientation_tolerance};
  //const bool& fall_back_to_joint_space_interpolation{
      //request.fall_back_to_joint_space_interpolation};
  //const double max_joint_position_change{request.max_joint_position_change};

  //const int num_knots = times.size();

  //int num_joints{robot->get_num_positions()};
  //const VectorX<int> joint_indices =
      //VectorX<int>::LinSpaced(num_joints, 0, num_joints - 1);
  //int grasp_frame_index = robot->FindBodyIndex(kGraspFrameName);
  //int world_idx = robot->FindBodyIndex("world");
  //Vector3<double> end_effector_points{0, 0, 0};

  //// Create vectors to hold the constraint objects.
  //std::vector<std::unique_ptr<PostureConstraint>> posture_constraints;
  //std::vector<std::unique_ptr<WorldQuatConstraint>> orientation_constraints;
  //std::vector<std::unique_ptr<Point2LineSegDistConstraint>>
      //point_to_line_seg_constraints;
  //std::vector<std::unique_ptr<WorldGazeDirConstraint>> gaze_dir_constraints;
  //std::vector<std::unique_ptr<PostureChangeConstraint>>
      //posture_change_constraints;
  //std::vector<std::unique_ptr<WorldPositionInFrameConstraint>>
      //position_in_frame_constraints;
  //std::vector<RigidBodyConstraint*> constraint_array;

  //// Compute the initial and final end-effector poses.
  //auto kinematics_cache = robot->CreateKinematicsCache();
  //kinematics_cache.initialize(q_initial);
  //robot->doKinematics(kinematics_cache);
  //const Isometry3<double> X_WG_initial =
      //robot->relativeTransform(kinematics_cache, world_idx, grasp_frame_index);
  //kinematics_cache.initialize(q_final);
  //robot->doKinematics(kinematics_cache);
  //const Isometry3<double> X_WG_final =
      //robot->relativeTransform(kinematics_cache, world_idx, grasp_frame_index);

  //std::pair<Vector3<double>, Vector3<double>> r_WG{X_WG_initial.translation(),
                                                   //X_WG_final.translation()};
  //std::pair<Quaternion<double>, Quaternion<double>> quat_WG{
      //Quaternion<double>(X_WG_initial.linear()),
      //Quaternion<double>(X_WG_final.linear())};

  //// Define active times for intermediate and final constraints.
  //const double& start_time = times.front();
  //const double& end_time = times.back();
  //const Vector2<double> intermediate_tspan{start_time, end_time};
  //const Vector2<double> final_tspan{end_time, end_time};

  //// Construct a KinematicTrajectoryOptimization object.
  //const int spline_order{5};
  //const int initial_num_control_points{2 * spline_order + 1};
  //manipulation::planner::KinematicTrajectoryOptimization prog{
      //robot->get_num_positions(), initial_num_control_points, spline_order};
  //prog.set_min_knot_resolution(2e-2);
  //prog.set_num_evaluation_points(30);
  //prog.set_initial_num_evaluation_points(2);

  //// Constrain initial and final configuration and derivatives.
  //VectorX<double> zero_vector{VectorX<double>::Zero(num_joints)};
  //prog.AddLinearConstraint(prog.position() == q_initial, {{0.0, 0.0}});
  //prog.AddLinearConstraint(prog.position() == q_final, {{1.0, 1.0}});
  //prog.AddLinearConstraint(prog.velocity() == zero_vector, {{0.0, 0.0}});
  //prog.AddLinearConstraint(prog.velocity() == zero_vector, {{1.0, 1.0}});
  //prog.AddLinearConstraint(prog.acceleration() == zero_vector, {{0.0, 0.0}});
  //prog.AddLinearConstraint(prog.acceleration() == zero_vector, {{1.0, 1.0}});
  //prog.AddLinearConstraint(prog.jerk() == zero_vector, {{0.0, 0.0}});
  //prog.AddLinearConstraint(prog.jerk() == zero_vector, {{1.0, 1.0}});

  //// Add velocity limits.
  //const double duration{end_time - start_time};
  //prog.AddLinearConstraint(
      //prog.velocity() <= 0.9 * duration * get_iiwa_max_joint_velocities(),
      //{{0.0, 1.0}});
  //prog.AddLinearConstraint(
      //prog.velocity() >= -0.9 * duration * get_iiwa_max_joint_velocities(),
      //{{0.0, 1.0}});
  //KinematicsCacheHelper<double> cache_helper{robot->bodies};

  //// Constrain the configuration at the final knot point to match q_final.
  //posture_constraints.emplace_back(new PostureConstraint(robot, final_tspan));
  //const VectorX<double> q_lb{q_final};
  //const VectorX<double> q_ub{q_final};
  //posture_constraints.back()->setJointLimits(joint_indices, q_lb, q_ub);
  //constraint_array.push_back(posture_constraints.back().get());

  //// Construct intermediate constraints for via points.
  //if (straight_line_motion) {
    //// We will impose a Point2LineSegDistConstraint and WorldGazeDirConstraint
    //// on the via points.
    //Eigen::Matrix<double, 3, 2> line_ends_W;
    //line_ends_W << r_WG.first, r_WG.second;

    //// double dist_lb{0.0};
    //// double dist_ub{position_tolerance};
    //// point_to_line_seg_constraints.emplace_back(new
    //// Point2LineSegDistConstraint(
    //// robot, grasp_frame_index, end_effector_points, world_idx, line_ends_W,
    //// dist_lb, dist_ub, intermediate_tspan));
    //// constraint_array.push_back(point_to_line_seg_constraints.back().get());
    //// prog.AddGenericPositionConstraint(
    //// std::make_shared<SingleTimeKinematicConstraintWrapper>(
    //// point_to_line_seg_constraints.back().get(), &cache_helper),
    ////{{0.0, 1.0}});

    //// Construct a frame whose origin is coincident with that of X_WG_initial
    //// and whose X-axis points towards the origin of X_WG_final.
    //auto X_WC = Isometry3<double>::Identity();  // Constraint frame to World
    //X_WC.linear() = math::ComputeBasisFromAxis<double>(
        //0, X_WG_final.translation() - X_WG_initial.translation());
    //X_WC.translation() = X_WG_initial.translation();

    //Vector3<double> lb{-position_tolerance, -position_tolerance,
                       //-position_tolerance};
    //Vector3<double> ub{
        //(X_WG_final.translation() - X_WG_initial.translation()).norm() +
            //position_tolerance,
        //position_tolerance, position_tolerance};
    //position_in_frame_constraints.emplace_back(
        //new WorldPositionInFrameConstraint(robot, grasp_frame_index,
                                           //end_effector_points, X_WC.matrix(),
                                           //lb, ub, intermediate_tspan));
    //constraint_array.push_back(position_in_frame_constraints.back().get());
    //prog.AddGenericPositionConstraint(
        //std::make_shared<SingleTimeKinematicConstraintWrapper>(
            //position_in_frame_constraints.back().get(), &cache_helper),
        //{{0.0, 1.0}});

    //// Find axis-angle representation of the rotation from X_WG_initial to
    //// X_WG_final.
    //Isometry3<double> X_WG_delta = X_WG_final.inverse() * X_WG_initial;
    //Eigen::AngleAxis<double> aaxis{X_WG_delta.linear()};
    //Vector3<double> axis_E{aaxis.axis()};
    //Vector3<double> dir_W{X_WG_initial.linear() * axis_E};

    //// If the intial and final end-effector orientations are close to each other
    //// (to within the orientation tolerance), fix the orientation for all via
    //// points. Otherwise, only allow the end-effector to rotate about the axis
    //// defining the rotation between the initial and final orientations.
    //if (std::abs(aaxis.angle()) < orientation_tolerance) {
      //orientation_constraints.emplace_back(new WorldQuatConstraint(
          //robot, grasp_frame_index,
          //Eigen::Vector4d(quat_WG.second.w(), quat_WG.second.x(),
                          //quat_WG.second.y(), quat_WG.second.z()),
          //orientation_tolerance, intermediate_tspan));
      //constraint_array.push_back(orientation_constraints.back().get());
      //prog.AddGenericPositionConstraint(
          //std::make_shared<SingleTimeKinematicConstraintWrapper>(
              //orientation_constraints.back().get(), &cache_helper),
          //{{0.0, 1.0}});
    //} else {
      //gaze_dir_constraints.emplace_back(new WorldGazeDirConstraint(
          //robot, grasp_frame_index, axis_E, dir_W, orientation_tolerance,
          //intermediate_tspan));
      //constraint_array.push_back(gaze_dir_constraints.back().get());
      //prog.AddGenericPositionConstraint(
          //std::make_shared<SingleTimeKinematicConstraintWrapper>(
              //gaze_dir_constraints.back().get(), &cache_helper),
          //{{0.0, 1.0}});
    //}

    //// Place limits on the change in joint angles between knots.
    //const VectorX<double> ub_change =
        //max_joint_position_change * VectorX<double>::Ones(num_joints);
    //const VectorX<double> lb_change = -ub_change;
    //for (int i = 1; i < num_knots; ++i) {
      //const Vector2<double> segment_tspan{times[i - 1], times[i]};
      //posture_change_constraints.emplace_back(new PostureChangeConstraint(
          //robot, joint_indices, lb_change, ub_change, segment_tspan));
      //constraint_array.push_back(posture_change_constraints.back().get());
    //}
  //} else {
    //// We will constrain the z-component of the end-effector position to be
    //// above the lower of the two end points at all via points.
    //Isometry3<double> X_WL{Isometry3<double>::Identity()};  // World to fLoor
    //X_WL.translation() =
        //(r_WG.first.z() < r_WG.second.z()) ? r_WG.first : r_WG.second;

    //Vector3<double> lb{-std::numeric_limits<double>::infinity(),
                       //-std::numeric_limits<double>::infinity(),
                       //-position_tolerance};
    //Vector3<double> ub{std::numeric_limits<double>::infinity(),
                       //std::numeric_limits<double>::infinity(),
                       //std::numeric_limits<double>::infinity()};

    //position_in_frame_constraints.emplace_back(
        //new WorldPositionInFrameConstraint(robot, grasp_frame_index,
                                           //end_effector_points, X_WL.matrix(),
                                           //lb, ub, intermediate_tspan));
    //constraint_array.push_back(position_in_frame_constraints.back().get());
    //prog.AddGenericPositionConstraint(
        //std::make_shared<SingleTimeKinematicConstraintWrapper>(
            //position_in_frame_constraints.back().get(), &cache_helper),
        //{{0.0, 1.0}});
  //}

  //// Set the seed for the first attempt (and the nominal value for all attempts)
  //// to be the cubic interpolation between the initial and final
  //// configurations.
  //VectorX<double> q_dot_zero{VectorX<double>::Zero(num_joints)};
  //MatrixX<double> q_knots_seed{num_joints, num_knots};
  //PiecewisePolynomial<double> q_seed_traj{PiecewisePolynomial<double>::Cubic(
      //{times.front(), times.back()}, {q_initial, q_final}, q_dot_zero,
      //q_dot_zero)};
  //for (int i = 0; i < num_knots; ++i) {
    //q_knots_seed.col(i) = q_seed_traj.value(times[i]);
  //}
  //MatrixX<double> q_knots_nom{q_knots_seed};

  //// Get the time values into the format required by inverseKinTrajSimple.
  //VectorX<double> t{num_knots};
  //for (int i = 0; i < num_knots; ++i) t(i) = times[i];

  //// Configure ik input and output structs.
  //IKoptions ikoptions(robot);
  //ikoptions.setFixInitialState(true);
  //ikoptions.setQa(MatrixX<double>::Identity(robot->get_num_positions(),
                                            //robot->get_num_positions()));
  //ikoptions.setQv(MatrixX<double>::Identity(robot->get_num_positions(),
                                            //robot->get_num_positions()));
  //ikoptions.setMajorOptimalityTolerance(straight_line_motion ? 1e-6 : 1e-4);
  //IKResults ik_res;

  //// Attempt to solve the ik traj problem multiple times. If falling back to
  //// joint-space interpolation is allowed, don't try as hard.
  //// const int num_restarts =
  ////(straight_line_motion && fall_back_to_joint_space_interpolation) ? 5 : 50;
  //// std::default_random_engine rand_generator{1234};
  //// for (int i = 0; i < num_restarts; ++i) {
  //// ik_res = inverseKinTrajSimple(robot, t, q_knots_seed, q_knots_nom,
  //// constraint_array, ikoptions);
  //// if (ik_res.info[0] == 1) {
  //// break;
  ////} else {
  //// VectorX<double> q_mid = robot->getRandomConfiguration(rand_generator);
  //// q_seed_traj = PiecewisePolynomial<double>::Cubic(
  ////{times.front(), 0.5 * (times.front() + times.back()), times.back()},
  ////{q_initial, q_mid, q_final}, q_dot_zero, q_dot_zero);
  //// for (int j = 0; j < num_knots; ++j) {
  //// q_knots_seed.col(j) = q_seed_traj.value(times[j]);
  ////}
  ////}
  ////}
  //PostureInterpolationResult result;
  //// result.success = (ik_res.info[0] == 1);
  //std::vector<MatrixX<double>> q_sol(num_knots);

  //bool done{false};
  //while (!done) {
    //solvers::SolutionResult solution_result =
        //prog.Solve(false [>always_update_curve<]);
    //drake::log()->info("Solution result: {}", solution_result);
    //if (solution_result == solvers::SolutionResult::kSolutionFound) {
      //done = !prog.UpdateGenericConstraints();
    //} else {
      //done = !prog.AddKnots();
    //}
  //}

  //// Add a cost on velocity squared
  //const int penalized_derivative_order{2};
  //auto cost_variable = prog.jerk();
  //switch (penalized_derivative_order) {
    //case 0: {
      //cost_variable = prog.position();
      //break;
    //}
    //case 1: {
      //cost_variable = prog.velocity();
      //break;
    //}
    //case 2: {
      //cost_variable = prog.acceleration();
      //break;
    //}
    //case 3: {
      //cost_variable = prog.jerk();
      //break;
    //}
    //default:
      //DRAKE_THROW_UNLESS(false);
  //}
  //double cost_weight{1.0};
  //auto squared_norm = cost_variable.transpose() * cost_variable;
  //drake::log()->info("Cost weight: {}", cost_weight);
  //prog.AddQuadraticCost(cost_weight * squared_norm(0));

  //done = false;
  //while (!done) {
    //solvers::SolutionResult solution_result =
        //prog.Solve(false [>always_update_curve<]);
    //drake::log()->info("Solution result: {}", solution_result);
    //if (solution_result == solvers::SolutionResult::kSolutionFound) {
      //done = !prog.UpdateGenericConstraints();
    //} else {
      //done = !prog.AddKnots();
    //}
    //result.success =
        //(solution_result == solvers::SolutionResult::kSolutionFound);
  //}

  //result.q_traj = prog.GetPositionSolution(duration);
  //if (result.success) {
    //// Upsample trajectory since we don't actually send the PiecwisePolynomial.
    //VectorX<double> t = VectorX<double>::LinSpaced(
        //prog.num_evaluation_points(), result.q_traj.getSegmentTimes().front(),
        //result.q_traj.getSegmentTimes().back());
    //q_sol.resize(prog.num_evaluation_points());
    //for (int i = 0; i < prog.num_evaluation_points(); ++i) {
      //q_sol[i] = result.q_traj.value(t(i));
    //}
    //// for (int i = 0; i < num_knots; ++i) {
    //// q_sol[i] = ik_res.q_sol[i];
    ////}
    //// result.q_traj = PiecewisePolynomial<double>::Cubic(times, q_sol,
    //// q_dot_zero,
    //// q_dot_zero);
  //} else if (straight_line_motion && fall_back_to_joint_space_interpolation) {
    //result = PlanInterpolatingMotion(request, robot,
                                     //false [> straight_line_motion <]);
  //} else {
    //result.q_traj = PiecewisePolynomial<double>::Cubic(
        //{times.front(), times.back()}, {q_initial, q_initial}, q_dot_zero,
        //q_dot_zero);
  //}
  //return result;
//}

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

  std::vector<double> yaw_offsets{0.0, M_PI};
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
  const double short_duration = 2;
  const double long_duration = 6;
  const double few_knots = 2;
  const double many_knots = 5;
  std::map<PickAndPlaceState, double> per_state_durations{
      {PickAndPlaceState::kApproachPickPregrasp, long_duration},
      {PickAndPlaceState::kApproachPick, short_duration},
      {PickAndPlaceState::kLiftFromPick, short_duration},
      {PickAndPlaceState::kApproachPlacePregrasp, long_duration},
      {PickAndPlaceState::kApproachPlace, short_duration},
      {PickAndPlaceState::kLiftFromPlace, short_duration}};
  std::map<PickAndPlaceState, double> per_state_number_of_knots{
      {PickAndPlaceState::kApproachPickPregrasp, many_knots},
      {PickAndPlaceState::kApproachPick, few_knots},
      {PickAndPlaceState::kLiftFromPick, few_knots},
      {PickAndPlaceState::kApproachPlacePregrasp, many_knots},
      {PickAndPlaceState::kApproachPlace, few_knots},
      {PickAndPlaceState::kLiftFromPlace, few_knots}};
  const int num_states = states.size();
  int num_knots = 1;
  const int num_joints{robot->get_num_positions()};
  for (int i = 0; i < num_states; ++i) {
    num_knots += per_state_number_of_knots.at(states[i]);
  }

  // Compute the initial end-effector pose
  const VectorX<double> q_initial{q_traj_seed.value(0)};
  auto kinematics_cache = robot->CreateKinematicsCache();
  kinematics_cache.initialize(q_initial);
  robot->doKinematics(kinematics_cache);

  VectorX<double> t{num_knots};
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

  // Construct a vector of KinematicTrajectoryOptimization objects.
  const int spline_order{5};
  const int initial_num_control_points{num_states * (2 * spline_order + 1)};
  std::vector<KinematicTrajectoryOptimization> programs;
  std::vector<std::unique_ptr<KinematicsCacheHelper<double>>> cache_helpers;

  // Set up an inverse kinematics trajectory problem with one knot for each
  // state
  MatrixX<double> q_nom =
      MatrixX<double>::Zero(robot->get_num_positions(), num_knots);

  std::vector<MatrixX<double>> seed_control_points(initial_num_control_points);
  for (int i = 0; i < initial_num_control_points; ++i) {
    seed_control_points[i] =
        q_traj_seed.value(static_cast<double>(i) / initial_num_control_points);
  }

  for (double pitch_offset : pitch_offsets) {
    for (double yaw_offset : yaw_offsets) {
      if (auto X_WG_desired =
              ComputeDesiredPoses(env_state, yaw_offset, pitch_offset)) {
        constraint_arrays.emplace_back();
        programs.emplace_back(BsplineCurve<double>(
            BsplineBasis(spline_order, initial_num_control_points),
            seed_control_points));
        KinematicTrajectoryOptimization& prog = programs.back();
        prog.set_min_knot_resolution(1e-3);
        prog.set_num_evaluation_points(100);
        prog.set_initial_num_evaluation_points(2);

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
            prog.velocity() >= -0.9 * duration * get_iiwa_max_joint_velocities(),
            {{0.0, 1.0}});

        // Constrain initial and final configuration and derivatives.
        VectorX<double> zero_vector{VectorX<double>::Zero(num_joints)};
        prog.AddLinearConstraint(prog.position() == q_initial, {{0.0, 0.0}});
        prog.AddLinearConstraint(prog.velocity() == zero_vector, {{0.0, 0.0}});
        prog.AddLinearConstraint(prog.acceleration() == zero_vector, {{0.0, 0.0}});
        prog.AddLinearConstraint(prog.jerk() == zero_vector, {{0.0, 0.0}});


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

          const std::array<double, 2> intermediate_plan_interval{
              {start_time / duration, end_time / duration}};
          const std::array<double, 2> final_plan_interval{
              {end_time / duration, end_time / duration}};

          // Extract desired position and orientation of end effector at the
          // given state.
          const Isometry3<double>& X_WG_final = X_WG_desired->at(state);
          const Vector3<double>& r_WG_final = X_WG_final.translation();
          const Quaternion<double>& quat_WG_final{X_WG_final.rotation()};
          const Vector3<double>& r_WG_initial = X_WG_initial.translation();
          //const Quaternion<double>& quat_WG_initial{X_WG_initial.rotation()};

          // Constrain the end-effector position and orientation at the end of
          // this state.
          position_constraints.emplace_back(new WorldPositionConstraint(
              robot, grasp_frame_index, end_effector_points,
              r_WG_final - position_tolerance, r_WG_final + position_tolerance,
              final_tspan));
          constraint_arrays.back().push_back(position_constraints.back().get());
          cache_helpers.emplace_back(new KinematicsCacheHelper<double>(robot->bodies));
          prog.AddGenericPositionConstraint(
              std::make_shared<SingleTimeKinematicConstraintWrapper>(
                  position_constraints.back().get(), cache_helpers.back().get()),
              final_plan_interval);

          orientation_constraints.emplace_back(new WorldQuatConstraint(
              robot, grasp_frame_index,
              Eigen::Vector4d(quat_WG_final.w(), quat_WG_final.x(),
                              quat_WG_final.y(), quat_WG_final.z()),
              orientation_tolerance, final_tspan));
          constraint_arrays.back().push_back(
              orientation_constraints.back().get());
          cache_helpers.emplace_back(new KinematicsCacheHelper<double>(robot->bodies));
          prog.AddGenericPositionConstraint(
              std::make_shared<SingleTimeKinematicConstraintWrapper>(
                  orientation_constraints.back().get(), cache_helpers.back().get()),
              final_plan_interval);

          prog.AddLinearConstraint(prog.velocity() == zero_vector, final_plan_interval);
          prog.AddLinearConstraint(prog.acceleration() == zero_vector, final_plan_interval);
          prog.AddLinearConstraint(prog.jerk() == zero_vector, final_plan_interval);

          double intermediate_orientation_tolerance = orientation_tolerance;
          Vector3<double> intermediate_position_tolerance = 10*position_tolerance;
          if (state == PickAndPlaceState::kApproachPlacePregrasp ||
              (state == PickAndPlaceState::kApproachPickPregrasp &&
               (r_WG_final - r_WG_initial).norm() > 0.2)) {
            intermediate_orientation_tolerance = 30 * M_PI / 180.0;
            intermediate_position_tolerance = Vector3<double>::Constant(0.5);
          }

          // Construct a frame whose origin is coincident with that of X_WG_initial
          // and whose X-axis points towards the origin of X_WG_final.
          auto X_WC = Isometry3<double>::Identity();  // Constraint frame to World
          X_WC.linear() = math::ComputeBasisFromAxis<double>(
              0, X_WG_final.translation() - X_WG_initial.translation());
          X_WC.translation() = X_WG_initial.translation();

          Vector3<double> lb{-intermediate_position_tolerance};
          Vector3<double> ub{intermediate_position_tolerance};
          ub.x() += (X_WG_final.translation() - X_WG_initial.translation()).norm();
          position_in_frame_constraints.emplace_back(
              new WorldPositionInFrameConstraint(robot, grasp_frame_index,
                                                 end_effector_points, X_WC.matrix(),
                                                 lb, ub, intermediate_tspan));
          constraint_arrays.back().push_back(position_in_frame_constraints.back().get());
          cache_helpers.emplace_back(new KinematicsCacheHelper<double>(robot->bodies));
          prog.AddGenericPositionConstraint(
              std::make_shared<SingleTimeKinematicConstraintWrapper>(
                  position_in_frame_constraints.back().get(), cache_helpers.back().get()),
              intermediate_plan_interval);

          Eigen::Matrix<double, 3, 2> line_ends_W;
          line_ends_W << r_WG_initial, r_WG_final;
          double dist_lb{0.0};
          double dist_ub{intermediate_position_tolerance.minCoeff()};
          point_to_line_seg_constraints.emplace_back(new
              Point2LineSegDistConstraint(
                robot, grasp_frame_index, end_effector_points, world_idx, line_ends_W,
                dist_lb, dist_ub, intermediate_tspan));
          //cache_helpers.emplace_back(new KinematicsCacheHelper<double>(robot->bodies));
          //prog.AddGenericPositionConstraint(
              //std::make_shared<SingleTimeKinematicConstraintWrapper>(
                //point_to_line_seg_constraints.back().get(), cache_helpers.back().get()),
              //intermediate_plan_interval);


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
          //Quaternion<double> quat_WG_intermediate =
              //quat_WG_initial.slerp(0.5, quat_WG_final);
          if (std::abs(aaxis.angle()) < orientation_tolerance) {
            orientation_constraints.emplace_back(new WorldQuatConstraint(
                robot, grasp_frame_index,
                Eigen::Vector4d(quat_WG_final.w(), quat_WG_final.x(),
                                quat_WG_final.y(), quat_WG_final.z()),
                intermediate_orientation_tolerance, intermediate_tspan));
            constraint_arrays.back().push_back(
                orientation_constraints.back().get());
            cache_helpers.emplace_back(new KinematicsCacheHelper<double>(robot->bodies));
            prog.AddGenericPositionConstraint(
                std::make_shared<SingleTimeKinematicConstraintWrapper>(
                    orientation_constraints.back().get(), cache_helpers.back().get()),
                intermediate_plan_interval);
          } else {
            gaze_dir_constraints.emplace_back(new WorldGazeDirConstraint(
                robot, grasp_frame_index, axis_E, dir_W, orientation_tolerance,
                intermediate_tspan));
            prog.AddGenericPositionConstraint(
                std::make_shared<SingleTimeKinematicConstraintWrapper>(
                    gaze_dir_constraints.back().get(), cache_helpers.back().get()),
                intermediate_plan_interval);
          }

          // Reset X_WG_initial for the next iteration.
          X_WG_initial = X_WG_final;
          start_knot += num_knots_per_state;
        }
      }
    }
  }

  if (constraint_arrays.empty()) return nullopt;

  bool success = false;
  PiecewisePolynomial<double> q_traj;
  for (auto& prog : programs) {
    bool done{true};
    while (!done) {
      solvers::SolutionResult solution_result =
          prog.Solve(false /*always_update_curve*/);
      drake::log()->info("Solution result: {}", solution_result);
      if (solution_result == solvers::SolutionResult::kSolutionFound) {
        //done = true;
        done = !prog.UpdateGenericConstraints();
      } else {
        done = !prog.AddKnots();
      }
    }

    // Add costs
    double position_weight{-1};
    double velocity_weight{1};
    double acceleration_weight{-1e-3};
    double jerk_weight{-1e-6};
    auto position_squared_norm = prog.position().transpose() * prog.position();
    auto velocity_squared_norm = prog.velocity().transpose() * prog.velocity();
    auto acceleration_squared_norm = prog.acceleration().transpose() * prog.acceleration();
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

    done = false;
    while (!done) {
      solvers::SolutionResult solution_result =
          prog.Solve(false /*always_update_curve*/);
      drake::log()->info("Solution result: {}", solution_result);
      if (solution_result == solvers::SolutionResult::kSolutionFound) {
        done = !prog.UpdateGenericConstraints();
        //done = true;
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
  std::vector<double> times;
  for (int i = 0; i < num_knots; ++i) times.push_back(t(i));
  int start_knot = 0;
  for (int i = 0; i < num_states; ++i) {
    int num_knots_per_state = per_state_number_of_knots.at(states[i]);
    const int& end_knot = start_knot + num_knots_per_state;
    std::vector<double> breaks;
    std::vector<MatrixX<double>> knots;
    drake::log()->trace("Trajectory for {}:", states[i]);
    // Upsample trajectory since we don't actually send the PiecwisePolynomial.
    VectorX<double> t_eval = VectorX<double>::LinSpaced(
        100, t(start_knot), t(end_knot));
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
        state_ = PickAndPlaceState::kApproachPickPregrasp;
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
  }
}

}  // namespace pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
