#include "drake/examples/kuka_iiwa_arm/pick_and_place/pick_and_place_state_machine.h"

#include <random>

#include <spdlog/fmt/ostr.h>

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"
#include "drake/common/trajectories/piecewise_quaternion.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace pick_and_place {
namespace {

using manipulation::planner::ConstraintRelaxingIk;


// Position the gripper 30cm above the object before grasp.
const double kPreGraspHeightOffset = 0.3;

// Finger is 19 cm from end-effector frame.
const double kEndEffectorToMidFingerDepth = 0.19;

PostureInterpolationResult PlanMotionAboveEndPoints(
    const PostureInterpolationRequest& request,
    const RigidBodyTree<double>& original_robot) {
  // Create local references to request member variables
  const VectorX<double>& q_0{request.q_initial};
  const VectorX<double>& q_f{request.q_final};
  const std::vector<double>& times{request.times};
  const double& position_tolerance{request.position_tolerance};

  // Create a local copy of the robot
  std::unique_ptr<RigidBodyTree<double>> robot{original_robot.Clone()};
  int kNumJoints{robot->get_num_positions()};
  const VectorX<int> joint_indices =
      VectorX<int>::LinSpaced(kNumJoints, 0, kNumJoints - 1);
  int end_effector_idx = robot->FindBodyIndex("iiwa_link_ee");
  int world_idx = robot->FindBodyIndex("world");
  Vector3<double> end_effector_points{kEndEffectorToMidFingerDepth, 0, 0};

  // Create vectors to hold the constraint objects
  std::vector<std::unique_ptr<PostureConstraint>> posture_constraints;
  std::vector<std::unique_ptr<WorldPositionInFrameConstraint>> position_in_frame_constraints;
  std::vector<RigidBodyConstraint*> constraint_array;

  const int kNumKnots = times.size();

  // Compute the initial and final end-effector poses
  auto kinematics_cache = robot->CreateKinematicsCache();
  kinematics_cache.initialize(q_0);
  robot->doKinematics(kinematics_cache);
  std::pair<Isometry3<double>, Isometry3<double>> X_WE;
  Isometry3<double> X_LE{Isometry3<double>::Identity()};
  X_LE.translation()[0] = kEndEffectorToMidFingerDepth;
  X_WE.first =
      robot->relativeTransform(kinematics_cache, world_idx, end_effector_idx) *
      X_LE;
  kinematics_cache.initialize(q_f);
  robot->doKinematics(kinematics_cache);
  X_WE.second =
      robot->relativeTransform(kinematics_cache, world_idx, end_effector_idx) *
      X_LE;

  std::pair<Vector3<double>, Vector3<double>> r_WE{X_WE.first.translation(),
                                                   X_WE.second.translation()};

  // Define active times for intermediate and final constraints
  const double& start_time = times.front();
  const double& end_time = times.back();
  const Vector2<double> intermediate_tspan{start_time, end_time};
  const Vector2<double> final_tspan{end_time, end_time};

  // Constrain the configuration at the final knot point to match q_f.
  posture_constraints.emplace_back(
      new PostureConstraint(robot.get(), final_tspan));
  const VectorX<double> q_lb{q_f};
  const VectorX<double> q_ub{q_f};
  posture_constraints.back()->setJointLimits(joint_indices, q_lb, q_ub);
  constraint_array.push_back(posture_constraints.back().get());

  // Construct intermediate constraints for via points
  // We will constrain the z-component of the end-effector position to be above
  // the lower of the two end points at all via points.
  Isometry3<double> X_WL{Isometry3<double>::Identity()}; // World to fLoor
  X_WL.translation() =
      (r_WE.first.z() < r_WE.second.z()) ? r_WE.first : r_WE.second;

  drake::log()->debug("Planning motion from {} to {}, above z = {}",
                      r_WE.first.transpose(), r_WE.second.transpose(),
                      X_WL.translation().z());

  Vector3<double> lb{-std::numeric_limits<double>::infinity(),
                     -std::numeric_limits<double>::infinity(),
                     -position_tolerance};
  Vector3<double> ub{std::numeric_limits<double>::infinity(),
                     std::numeric_limits<double>::infinity(),
                     std::numeric_limits<double>::infinity()};

  position_in_frame_constraints.emplace_back(new WorldPositionInFrameConstraint(
      robot.get(), end_effector_idx, end_effector_points, X_WL.matrix(), lb, ub,
      intermediate_tspan));
  constraint_array.push_back(position_in_frame_constraints.back().get());

  // Set the seed for the first attempt (and the nominal value for all attempts)
  // to be the cubic interpolation between the initial and final
  // configurations.
  VectorX<double> q_dot0{VectorX<double>::Zero(kNumJoints)};
  VectorX<double> q_dotf{VectorX<double>::Zero(kNumJoints)};
  MatrixX<double> q_knots_seed{kNumJoints, kNumKnots};
  PiecewisePolynomial<double> q_seed_traj{PiecewisePolynomial<double>::Cubic(
      {times.front(), times.back()}, {q_0, q_f}, q_dot0, q_dotf)};
  for (int i = 0; i < kNumKnots; ++i) {
    q_knots_seed.col(i) = q_seed_traj.value(times[i]);
  }
  MatrixX<double> q_knots_nom{q_knots_seed};

  // Get the time values into the format required by inverseKinTrajSimple
  VectorX<double> t{kNumKnots};
  for (int i = 0; i < kNumKnots; ++i) t(i) = times[i];

  // Configure ik input and output structs.
  IKoptions ikoptions(robot.get());
  ikoptions.setFixInitialState(true);
  ikoptions.setQa(MatrixX<double>::Identity(robot->get_num_positions(),
                                            robot->get_num_positions()));
  ikoptions.setQv(MatrixX<double>::Identity(robot->get_num_positions(),
                                            robot->get_num_positions()));
  ikoptions.setMajorOptimalityTolerance(1e-4);
  IKResults ik_res;

  // Attempt to solve the ik traj problem multiple times. If falling back to
  // joint-space interpolation is allowed, don't try as hard.
  const int kNumRestarts = 50;
  std::default_random_engine rand_generator{1234};
  for (int i = 0; i < kNumRestarts; ++i) {
    ik_res = inverseKinTrajSimple(robot.get(), t, q_knots_seed, q_knots_nom,
                                   constraint_array, ikoptions);
    if (ik_res.info[0] == 1) {
      break;
    } else {
      VectorX<double> q_mid = robot->getRandomConfiguration(rand_generator);
      q_seed_traj = PiecewisePolynomial<double>::Cubic(
          {times.front(), 0.5 * (times.front() + times.back()), times.back()},
          {q_0, q_mid, q_f}, q_dot0, q_dotf);
      for (int j = 0; j < kNumKnots; ++j) {
        q_knots_seed.col(j) = q_seed_traj.value(times[j]);
      }
    }
  }
  PostureInterpolationResult result;
  result.success = (ik_res.info[0] == 1);
  std::vector<MatrixX<double>> q_sol(kNumKnots);
  if (result.success) {
    for (int i = 0; i < kNumKnots; ++i) {
      q_sol[i] = ik_res.q_sol[i];
    }
    result.q_traj = PiecewisePolynomial<double>::Cubic(times, q_sol, q_dot0, q_dotf);
  } else {
    result.q_traj = PiecewisePolynomial<double>::Cubic(
            {times.front(), times.back()}, {q_0, q_0}, q_dot0, q_dotf);
  }
  return result;
}

// Generates a sequence (@p num_via_points + 1) of key frames s.t. the end
// effector moves in a straight line between @pX_WEndEffector0 and
// @p X_WEndEffector1.
PostureInterpolationResult PlanStraightLineMotion(
    const PostureInterpolationRequest& request,
    const RigidBodyTree<double>& original_robot) {
  // Create local references to request member variables
  const VectorX<double>& q_0{request.q_initial};
  const VectorX<double>& q_f{request.q_final};
  const std::vector<double>& times{request.times};
  const double& position_tolerance{request.position_tolerance};
  const double& orientation_tolerance{request.orientation_tolerance};
  const bool& fall_back_to_joint_space_interpolation{
      request.fall_back_to_joint_space_interpolation};
  const double max_joint_position_change{request.max_joint_position_change};

  const int kNumKnots = times.size();

  drake::log()->debug("Processing interpolation request:");
  drake::log()->debug("\tNumber of knots: {}", kNumKnots);
  drake::log()->debug("\tPosition tolerance: {}", position_tolerance);
  drake::log()->debug("\tOrientation tolerance: {}", orientation_tolerance);
  drake::log()->debug("\tFall back to joint space: {}",
                      fall_back_to_joint_space_interpolation);

  // Create a local copy of the robot
  std::unique_ptr<RigidBodyTree<double>> robot{original_robot.Clone()};
  int kNumJoints{robot->get_num_positions()};
  const VectorX<int> joint_indices =
      VectorX<int>::LinSpaced(kNumJoints, 0, kNumJoints - 1);
  int end_effector_idx = robot->FindBodyIndex("iiwa_link_ee");
  int world_idx = robot->FindBodyIndex("world");
  Vector3<double> end_effector_points{kEndEffectorToMidFingerDepth, 0, 0};

  // Create vectors to hold the constraint objects
  std::vector<std::unique_ptr<PostureConstraint>> posture_constraints;
  std::vector<std::unique_ptr<WorldQuatConstraint>> orientation_constraints;
  std::vector<std::unique_ptr<Point2LineSegDistConstraint>>
      point_to_line_seg_constraints;
  std::vector<std::unique_ptr<WorldGazeDirConstraint>> gaze_dir_constraints;
  std::vector<std::unique_ptr<PostureChangeConstraint>>
      posture_change_constraints;
  std::vector<RigidBodyConstraint*> constraint_array;

  // Compute the initial and final end-effector poses
  auto kinematics_cache = robot->CreateKinematicsCache();
  kinematics_cache.initialize(q_0);
  robot->doKinematics(kinematics_cache);
  std::pair<Isometry3<double>, Isometry3<double>> X_WE;
  Isometry3<double> X_LE{Isometry3<double>::Identity()};
  X_LE.translation()[0] = kEndEffectorToMidFingerDepth;
  X_WE.first =
      robot->relativeTransform(kinematics_cache, world_idx, end_effector_idx) *
      X_LE;
  kinematics_cache.initialize(q_f);
  robot->doKinematics(kinematics_cache);
  X_WE.second =
      robot->relativeTransform(kinematics_cache, world_idx, end_effector_idx) *
      X_LE;

  std::pair<Vector3<double>, Vector3<double>> r_WE{X_WE.first.translation(),
                                                   X_WE.second.translation()};
  std::pair<Quaternion<double>, Quaternion<double>> quat_WE{
      Quaternion<double>(X_WE.first.linear()),
      Quaternion<double>(X_WE.second.linear())};

  drake::log()->debug("Planning straight line from {} {} to {} {}",
                      r_WE.first.transpose(),
                      math::rotmat2rpy(X_WE.first.rotation()).transpose(),
                      r_WE.second.transpose(),
                      math::rotmat2rpy(X_WE.second.rotation()).transpose());

  // Define active times for intermediate and final constraints
  const double& start_time = times.front();
  const double& end_time = times.back();
  const Vector2<double> intermediate_tspan{start_time, end_time};
  const Vector2<double> final_tspan{end_time, end_time};

  // Constrain the configuration at the final knot point to match q_f.
  posture_constraints.emplace_back(
      new PostureConstraint(robot.get(), final_tspan));
  const VectorX<double> q_lb{q_f};
  const VectorX<double> q_ub{q_f};
  posture_constraints.back()->setJointLimits(joint_indices, q_lb, q_ub);
  constraint_array.push_back(posture_constraints.back().get());

  // Construct intermediate constraints for via points
  // We will impose a Point2LineSegDistConstraint and WorldGazeDirConstraint
  // on the via points.
  Eigen::Matrix<double, 3, 2> line_ends_W;
  line_ends_W << r_WE.first, r_WE.second;

  double dist_lb{0.0};
  double dist_ub{position_tolerance};
  point_to_line_seg_constraints.emplace_back(new Point2LineSegDistConstraint(
      robot.get(), end_effector_idx, end_effector_points, world_idx,
      line_ends_W, dist_lb, dist_ub, intermediate_tspan));
  constraint_array.push_back(point_to_line_seg_constraints.back().get());

  // Find axis-angle representation of the rotation from X_WE.first to
  // X_WE.second.
  Isometry3<double> X_second_first = X_WE.second.inverse() * X_WE.first;
  Eigen::AngleAxis<double> aaxis{X_second_first.linear()};
  Vector3<double> axis_E{aaxis.axis()};
  Vector3<double> dir_W{X_WE.first.linear() * axis_E};

  // If the intial and final end-effector orientations are close to each other
  // (to within the orientation tolerance), fix the orientation for all via
  // points. Otherwise, only allow the end-effector to rotate about the axis
  // defining the rotation between the initial and final orientations.
  if (std::abs(aaxis.angle()) < orientation_tolerance) {
    orientation_constraints.emplace_back(new WorldQuatConstraint(
        robot.get(), end_effector_idx,
        Eigen::Vector4d(quat_WE.second.w(), quat_WE.second.x(),
                        quat_WE.second.y(), quat_WE.second.z()),
        orientation_tolerance, intermediate_tspan));
    constraint_array.push_back(orientation_constraints.back().get());
  } else {
    gaze_dir_constraints.emplace_back(new WorldGazeDirConstraint(
        robot.get(), end_effector_idx, axis_E, dir_W,
        orientation_tolerance, intermediate_tspan));
    constraint_array.push_back(gaze_dir_constraints.back().get());
  }

  // Place limits on the change in joint angles between knots
  const VectorX<double> ub_change =
      max_joint_position_change * VectorX<double>::Ones(kNumJoints);
  const VectorX<double> lb_change = -ub_change;
  for (int i = 1; i < kNumKnots; ++i) {
    const Vector2<double> segment_tspan{times[i-1], times[i]};
    posture_change_constraints.emplace_back(new PostureChangeConstraint(
        robot.get(), joint_indices, lb_change, ub_change, segment_tspan));
    constraint_array.push_back(posture_change_constraints.back().get());
  }

  // Set the seed for the first attempt (and the nominal value for all attempts)
  // to be the cubic interpolation between the initial and final
  // configurations.
  VectorX<double> q_dot0{VectorX<double>::Zero(kNumJoints)};
  VectorX<double> q_dotf{VectorX<double>::Zero(kNumJoints)};
  MatrixX<double> q_knots_seed{kNumJoints, kNumKnots};
  PiecewisePolynomial<double> q_seed_traj{PiecewisePolynomial<double>::Cubic(
      {times.front(), times.back()}, {q_0, q_f}, q_dot0, q_dotf)};
  for (int i = 0; i < kNumKnots; ++i) {
    q_knots_seed.col(i) = q_seed_traj.value(times[i]);
  }
  MatrixX<double> q_knots_nom{q_knots_seed};

  // Get the time values into the format required by inverseKinTrajSimple
  VectorX<double> t{kNumKnots};
  for (int i = 0; i < kNumKnots; ++i) t(i) = times[i];

  // Configure ik input and output structs.
  IKoptions ikoptions(robot.get());
  ikoptions.setFixInitialState(true);
  ikoptions.setQa(MatrixX<double>::Identity(robot->get_num_positions(),
                                            robot->get_num_positions()));
  ikoptions.setQv(MatrixX<double>::Identity(robot->get_num_positions(),
                                            robot->get_num_positions()));
  ikoptions.setMajorOptimalityTolerance(1e-6);
  IKResults ik_res;

  // Attempt to solve the ik traj problem multiple times. If falling back to
  // joint-space interpolation is allowed, don't try as hard.
  const int kNumRestarts =
      (fall_back_to_joint_space_interpolation) ? 5 : 50;
  std::default_random_engine rand_generator{1234};
  for (int i = 0; i < kNumRestarts; ++i) {
    ik_res = inverseKinTrajSimple(robot.get(), t, q_knots_seed, q_knots_nom,
                                   constraint_array, ikoptions);
    if (ik_res.info[0] == 1) {
      break;
    } else {
      VectorX<double> q_mid = robot->getRandomConfiguration(rand_generator);
      q_seed_traj = PiecewisePolynomial<double>::Cubic(
          {times.front(), 0.5 * (times.front() + times.back()), times.back()},
          {q_0, q_mid, q_f}, q_dot0, q_dotf);
      for (int j = 0; j < kNumKnots; ++j) {
        q_knots_seed.col(j) = q_seed_traj.value(times[j]);
      }
    }
  }
  PostureInterpolationResult result;
  result.success = (ik_res.info[0] == 1);
  std::vector<MatrixX<double>> q_sol(kNumKnots);
  if (result.success) {
    for (int i = 0; i < kNumKnots; ++i) {
      q_sol[i] = ik_res.q_sol[i];
    }
    result.q_traj = PiecewisePolynomial<double>::Cubic(times, q_sol, q_dot0, q_dotf);
  } else if (fall_back_to_joint_space_interpolation) {
    result = PlanMotionAboveEndPoints(request, original_robot);
  } else {
    result.q_traj = PiecewisePolynomial<double>::Cubic(
            {times.front(), times.back()}, {q_0, q_0}, q_dot0, q_dotf);
  }
  return result;
}

void OpenGripper(const WorldState& env_state, WsgAction* wsg_act,
                 lcmt_schunk_wsg_command* msg) {
  wsg_act->OpenGripper(env_state, msg);
};

void CloseGripper(const WorldState& env_state, WsgAction* wsg_act,
                 lcmt_schunk_wsg_command* msg) {
  wsg_act->CloseGripper(env_state, msg);
};

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

void PickAndPlaceStateMachine::ComputeDesiredPoses(
    const WorldState& env_state, double yaw_offset) {
  X_WE_desired_.clear();

  //     
  //       (ApproachPickPregrasp,                         (ApproachPlacePregrasp
  //        LiftFromPick ),                                LiftFromPlace)
  //       +--------------------------------------------------------+
  //       |                                                        |
  //       |                                                        |
  //       + (ApproachPick)                         (ApproachPlace) +

  // Gripper is rotated relative to the end effector
  Isometry3<double> X_GE{Isometry3<double>::Identity()};
  X_GE.rotate(Eigen::AngleAxisd(0.39269908, Eigen::Vector3d::UnitX()));

  Isometry3<double> X_WS{env_state.get_iiwa_base().inverse()};
  Isometry3<double> X_WO_initial{X_WS*env_state.get_object_pose()};
  drake::log()->debug("r_WO_initial = [{}]",
                      X_WO_initial.translation().transpose());
  drake::log()->debug("R_WO_initial = \n{}",
                      X_WO_initial.linear());
  X_WO_initial.rotate(Eigen::AngleAxisd(yaw_offset, Eigen::Vector3d::UnitZ()));
  Isometry3<double> X_WO_final{place_locations_[next_place_location_]};
  drake::log()->debug("r_WO_final = [{}]",
                      X_WO_final.translation().transpose());
  drake::log()->debug("R_WO_final = \n{}",
                      X_WO_final.linear());

  // Set ApproachPick pose
  Isometry3<double> X_OG{Isometry3<double>::Identity()};
  X_WE_desired_.emplace(PickAndPlaceState::kApproachPick,
                        X_WO_initial * X_OG * X_GE);

  // Set ApproachPickPregrasp pose
  X_OG.setIdentity();
  X_OG.translation()[2] = kPreGraspHeightOffset;
  X_WE_desired_.emplace(PickAndPlaceState::kApproachPickPregrasp,
                        X_WO_initial * X_OG * X_GE);

  // Set LiftFromPick pose
  X_OG.setIdentity();
  X_OG.translation()[2] = kPreGraspHeightOffset;
  X_WE_desired_.emplace(PickAndPlaceState::kLiftFromPick,
                        X_WO_initial * X_OG * X_GE);

  // Set ApproachPlace pose
  X_OG.setIdentity();
  X_WE_desired_.emplace(PickAndPlaceState::kApproachPlace,
                        X_WO_final * X_OG * X_GE);

  // Set ApproachPlacePregrasp pose
  X_OG.setIdentity();
  X_OG.translation()[2] = kPreGraspHeightOffset;
  X_WE_desired_.emplace(PickAndPlaceState::kApproachPlacePregrasp,
                        X_WO_final * X_OG * X_GE);

  // Set LiftFromPlace pose
  X_OG.setIdentity();
  X_OG.translation()[2] = kPreGraspHeightOffset;
  X_WE_desired_.emplace(PickAndPlaceState::kLiftFromPlace,
                        X_WO_final * X_OG * X_GE);
}

PostureInterpolationRequest
PickAndPlaceStateMachine::CreatePostureInterpolationRequest(
    const WorldState& env_state, PickAndPlaceState state, double duration,
    bool fall_back_to_joint_space_interpolation) {
  PostureInterpolationRequest request;
  request.max_joint_position_change = 0.5*M_PI_4;
  request.q_initial = env_state.get_iiwa_q();
  request.q_final = nominal_q_map_.at(state);
  double max_delta_q{
      (request.q_final - request.q_initial).cwiseAbs().maxCoeff()};
  int num_via_points =
      std::ceil(2*max_delta_q / request.max_joint_position_change);
  double dt{duration/static_cast<double>(num_via_points)};
  request.times.resize(num_via_points + 1);
  request.times.front() = 0.0;
  for (int i = 1; i < static_cast<int>(request.times.size()) - 1; ++i) {
    request.times[i] = i*dt;
  }
  request.times.back() = duration;
  request.position_tolerance = tight_pos_tol_(0);
  request.orientation_tolerance = tight_rot_tol_;
  request.fall_back_to_joint_space_interpolation =
      fall_back_to_joint_space_interpolation;
  drake::log()->debug("Interpolation request has {} knots.",
                      request.times.size());
  return request;
}

PickAndPlaceStateMachine::PickAndPlaceStateMachine(
    const std::vector<Isometry3<double>>& place_locations, bool loop)
    : place_locations_(place_locations),
      next_place_location_(0),
      loop_(loop),
      state_(PickAndPlaceState::kOpenGripper),
      // Position and rotation tolerances.  These were hand-tuned by
      // adjusting to tighter bounds until IK stopped reliably giving
      // results.
      tight_pos_tol_(0.001, 0.001, 0.001),
      tight_rot_tol_(0.05),
      loose_pos_tol_(0.1, 0.1, 0.1),
      loose_rot_tol_(30*M_PI/180) {
  DRAKE_THROW_UNLESS(!place_locations.empty());
}

PickAndPlaceStateMachine::~PickAndPlaceStateMachine() {}

bool PickAndPlaceStateMachine::ComputeNominalConfigurations(
    const RigidBodyTree<double>& iiwa, const WorldState& env_state) {
  bool success = false;
  nominal_q_map_.clear();
  //  Create vectors to hold the constraint objects
  std::vector<std::unique_ptr<WorldPositionConstraint>> position_constraints;
  std::vector<std::unique_ptr<WorldQuatConstraint>> orientation_constraints;
  std::vector<std::unique_ptr<PostureChangeConstraint>>
    posture_change_constraints;
  std::vector<std::vector<RigidBodyConstraint*>> constraint_array;
  std::vector<double> yaw_offsets{M_PI, 0.0};
  const int kNumYawOffsets = yaw_offsets.size();
  std::unique_ptr<RigidBodyTree<double>> robot{iiwa.Clone()};
  int kNumJoints{robot->get_num_positions()};

  int end_effector_body_idx = robot->FindBodyIndex("iiwa_link_ee");
  Vector3<double> end_effector_points{kEndEffectorToMidFingerDepth, 0, 0};

  std::vector<PickAndPlaceState> states{
      PickAndPlaceState::kApproachPickPregrasp,
      PickAndPlaceState::kApproachPick,
      PickAndPlaceState::kLiftFromPick,
      PickAndPlaceState::kApproachPlacePregrasp,
      PickAndPlaceState::kApproachPlace,
      PickAndPlaceState::kLiftFromPlace};
  int kNumKnots = states.size();

  VectorX<double> q_seed_local{env_state.get_iiwa_q()};
  VectorX<double> t{VectorX<double>::LinSpaced(kNumKnots, 0, kNumKnots - 1)};
  // Set up an inverse kinematics trajectory problem with one knot for each
  // state
  MatrixX<double> q_nom =
      MatrixX<double>::Zero(robot->get_num_positions(), kNumKnots);

  for (double yaw_offset : yaw_offsets) {
    ComputeDesiredPoses(env_state, yaw_offset);
    constraint_array.emplace_back();

    for (int i = 0; i < kNumKnots; ++i) {
      const PickAndPlaceState state{states[i]};
      const Vector2<double> knot_tspan{t(i), t(i)};

      // Extract desired position and orientation of end effector at the given
      // state.
      const Isometry3<double>& X_WE = X_WE_desired_.at(state);
      const Vector3<double>& r_WE = X_WE.translation();
      const Quaternion<double>& quat_WE{X_WE.rotation()};


      // Constrain the end-effector position for all knots.
      position_constraints.emplace_back(new WorldPositionConstraint(
          robot.get(), end_effector_body_idx, end_effector_points,
          r_WE - tight_pos_tol_, r_WE + tight_pos_tol_, knot_tspan));
      constraint_array.back().push_back(position_constraints.back().get());

      // Constrain the end-effector orientation for all knots
      orientation_constraints.emplace_back(new WorldQuatConstraint(
          robot.get(), end_effector_body_idx,
          Eigen::Vector4d(quat_WE.w(), quat_WE.x(), quat_WE.y(), quat_WE.z()),
          tight_rot_tol_, knot_tspan));
      constraint_array.back().push_back(orientation_constraints.back().get());

      // For each pair of adjacent knots, add a constraint on the change in
      // joint
      // positions.
      if (i > 0) {
        const VectorX<int> joint_indices =
            VectorX<int>::LinSpaced(kNumJoints, 0, kNumJoints - 1);
        const Vector2<double> segment_tspan{t(i - 1), t(i)};
        // The move to ApproachPlacePregrasp can require large joint motions.
        const double max_joint_position_change =
            (state == PickAndPlaceState::kApproachPlacePregrasp) ? 0.75 * M_PI
                                                                 : M_PI_4;
        const VectorX<double> ub_change{max_joint_position_change *
                                        VectorX<double>::Ones(kNumJoints)};
        const VectorX<double> lb_change{-ub_change};
        posture_change_constraints.emplace_back(new PostureChangeConstraint(
            robot.get(), joint_indices, lb_change, ub_change, segment_tspan));
        constraint_array.back().push_back(
            posture_change_constraints.back().get());
      }
    }
  }

  // Solve the IK problem. Re-seed with random values if the initial seed is
  // unsuccessful.
  IKResults ik_res;
  IKoptions ikoptions(robot.get());
  ikoptions.setFixInitialState(false);
  ikoptions.setQ(MatrixX<double>::Zero(robot->get_num_positions(),
                                       robot->get_num_positions()));
  ikoptions.setQv(MatrixX<double>::Identity(robot->get_num_positions(),
                                            robot->get_num_positions()));
  const int kNumRestarts = 50;
  std::default_random_engine rand_generator{1234};
  for (int i = 0; i < kNumRestarts; ++i) {
    MatrixX<double> q_knots_seed{robot->get_num_positions(), kNumKnots};
    for (int j = 0; j < kNumKnots; ++j) {
      q_knots_seed.col(j) = q_seed_local;
    }
    ik_res =
        inverseKinTrajSimple(robot.get(), t, q_knots_seed, q_nom,
                             constraint_array[i % kNumYawOffsets], ikoptions);
    success = ik_res.info[0] == 1;
    if (success) {
      q_seed_local = ik_res.q_sol.back();
      break;
    } else {
      q_seed_local = robot->getRandomConfiguration(rand_generator);
      drake::log()->warn("Attempt {} failed with info {}", i, ik_res.info[0]);
    }
  }
  if (success) {
    for (int i = 0; i < kNumKnots; ++i) {
      drake::log()->debug("State {}: q = ({})", states[i],
                          ik_res.q_sol[i].transpose());
      nominal_q_map_.emplace(states[i], ik_res.q_sol[i]);
    }
  }
  return success;
}

bool PickAndPlaceStateMachine::ComputeTrajectories(const RigidBodyTree<double>& iiwa, const WorldState& env_state) {
  ComputeNominalConfigurations(iiwa, env_state);
  bool success{true};
  std::vector<PickAndPlaceState> states{
      PickAndPlaceState::kApproachPickPregrasp,
      PickAndPlaceState::kApproachPick,
      PickAndPlaceState::kLiftFromPick,
      PickAndPlaceState::kApproachPlacePregrasp,
      PickAndPlaceState::kApproachPlace,
      PickAndPlaceState::kLiftFromPlace};
  VectorX<double> q_0{iiwa.get_num_positions()};
  if (interpolation_result_map_.empty()) {
    q_0 << env_state.get_iiwa_q();
    drake::log()->debug("Using current configuration as q_0.");
  } else {
    drake::log()->debug("Using end of the last plan as q_0.");
    const PiecewisePolynomial<double>& q_traj_last =
        interpolation_result_map_.at(states.back()).q_traj;
    q_0 << q_traj_last.value(q_traj_last.getEndTime());
  }
  drake::log()->debug("\tq_0 = [{}]", q_0.transpose());
  drake::log()->debug("Clearing interpolation_result_map_.");
  interpolation_result_map_.clear();
  const double kShortDuration = 2;
  const double kLongDuration = 3;
  const double kExtraLongDuration = 5;
  for (PickAndPlaceState state : states) {
    drake::log()->info("Planning trajectory for {}.", state);
    const VectorX<double> q_f = nominal_q_map_.at(state);
    double duration{kShortDuration};
    double position_tolerance{tight_pos_tol_(0)};
    double orientation_tolerance{tight_rot_tol_};
    bool fall_back_to_joint_space_interpolation{false};
    switch (state) {
      case PickAndPlaceState::kApproachPickPregrasp:
      case PickAndPlaceState::kApproachPlacePregrasp: {
        position_tolerance = loose_pos_tol_(0);
        orientation_tolerance = loose_rot_tol_;
        fall_back_to_joint_space_interpolation = true;
        duration = kLongDuration;
      } break;

      case PickAndPlaceState::kLiftFromPlace: {
        position_tolerance = loose_pos_tol_(0);
        orientation_tolerance = loose_rot_tol_;
        fall_back_to_joint_space_interpolation = true;
        duration = kExtraLongDuration;
      } break;

      default:  // No action needed for other cases
        break;
    }
    PostureInterpolationRequest request;

    request.max_joint_position_change = 0.5*M_PI_4;
    request.q_initial = q_0;
    request.q_final = q_f;
    double max_delta_q{
      (request.q_final - request.q_initial).cwiseAbs().maxCoeff()};
    int num_via_points = std::min<int>(
        3, std::ceil(2 * max_delta_q / request.max_joint_position_change));
    double dt{duration/static_cast<double>(num_via_points)};
    request.times.resize(num_via_points + 1);
    request.times.front() = 0.0;
    for (int i = 1; i < static_cast<int>(request.times.size()) - 1; ++i) {
      request.times[i] = i*dt;
    }
    request.times.back() = duration;
    request.position_tolerance = position_tolerance;
    request.orientation_tolerance = orientation_tolerance;
    request.fall_back_to_joint_space_interpolation =
        fall_back_to_joint_space_interpolation;

    PostureInterpolationResult result = PlanStraightLineMotion(request, iiwa);

    interpolation_result_map_.emplace(state, result);
    success = success && result.success;
    q_0 = q_f;
  }
  return success;
}

void PickAndPlaceStateMachine::Update(
    const WorldState& env_state, const IiwaPublishCallback& iiwa_callback,
    const WsgPublishCallback& wsg_callback,
    manipulation::planner::ConstraintRelaxingIk* planner) {
  IKResults ik_res;
  robotlocomotion::robot_plan_t stopped_plan{};
  stopped_plan.num_states = 0;

  const RigidBodyTree<double>& iiwa = planner->get_robot();

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

    default: // No action needed for other cases
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
        robotlocomotion::robot_plan_t plan{};
        std::vector<VectorX<double>> q;
        PostureInterpolationResult& result = interpolation_result_map_.at(state_); 
        DRAKE_THROW_UNLESS(result.success);
        const std::vector<double>& times{result.q_traj.getSegmentTimes()};
        q.reserve(times.size());
        for (double t : times) {
          q.push_back(result.q_traj.value(t));
        }
        iiwa_move_.MoveJoints(env_state, iiwa, times, q, &plan);
        iiwa_callback(&plan);

        drake::log()->info("{} at {}", state_, env_state.get_iiwa_time());
        drake::log()->debug("\tq_0 = [{}]", q.front().transpose());
        drake::log()->debug("\tq_f = [{}]", q.back().transpose());
      }
      if (iiwa_move_.ActionFinished(env_state)) {
        state_ = next_state;
        iiwa_move_.Reset();
      }
      break;
    }

    // Schunk gripper actions
    case PickAndPlaceState::kOpenGripper: {
      if (!wsg_act_.ActionStarted()) {
        const Isometry3<double>& obj_pose = env_state.get_object_pose();
        drake::log()->info("Object at: {} {}", obj_pose.translation().transpose(),
                           math::rotmat2rpy(obj_pose.rotation()).transpose());
        const Isometry3<double>& iiwa_pose = env_state.get_iiwa_base();
        drake::log()->info("Base at: {} {}", iiwa_pose.translation().transpose(),
                           math::rotmat2rpy(iiwa_pose.rotation()).transpose());
      }
    } // Intentionally fall through
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
      // Compute all the desired configurations
      bool success{false};
      for (int i = 0; i < 1; ++i) {
        success = ComputeTrajectories(iiwa, env_state);
        if (success) break;
      }
      DRAKE_THROW_UNLESS(success);
      state_ = PickAndPlaceState::kApproachPickPregrasp;
      break;
    }

    case PickAndPlaceState::kReset: {
      next_place_location_++;
      if (next_place_location_ == static_cast<int>(place_locations_.size()) &&
          !loop_) {
        state_ = PickAndPlaceState::kDone;
        iiwa_callback(&stopped_plan);
        drake::log()->info("{} at {}", state_, env_state.get_iiwa_time());
      } else {
        next_place_location_ %= place_locations_.size();
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
