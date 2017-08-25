#include "drake/examples/kuka_iiwa_arm/pick_and_place/pick_and_place_state_machine.h"

#include <random>

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

// Position the gripper 15cm away from the object before grasp.
//const double kPreGraspOffset = 0.15;

// Computes the desired end effector pose in the world frame given the object
// pose in the world frame.
//Isometry3<double> ComputeGraspPose(const Isometry3<double>& X_WObj) {
  // Sets desired end effector location to be 12cm behind the object,
  // with the same orientation relative to the object frame. This number
  // dependents on the length of the finger and how the gripper is attached.
  //const double kEndEffectorToMidFingerDepth = 0.12;
  //Isometry3<double> X_ObjEndEffector_desired;
  //X_ObjEndEffector_desired.translation() =
      //Vector3<double>(-kEndEffectorToMidFingerDepth, 0, 0);
  //X_ObjEndEffector_desired.linear().setIdentity();
  //return X_WObj * X_ObjEndEffector_desired;
//}

// Generates a sequence (@p num_via_points + 1) of key frames s.t. the end
// effector moves in a straight line between @pX_WEndEffector0 and
// @p X_WEndEffector1.
bool PlanStraightLineMotion(
    const VectorX<double>& q_current,
    const MatrixX<double>& q_seed,
    std::vector<Waypoint>::const_iterator waypoints_start,
    std::vector<Waypoint>::const_iterator waypoints_end,
    const RigidBodyTree<double>& original_robot, IKResults* ik_res,
    std::vector<double>* times) {
  MatrixX<double> q_seed_local{q_seed};
  std::unique_ptr<RigidBodyTree<double>> robot{original_robot.Clone()};
  int kNumJoints{robot->get_num_positions()};
  const VectorX<int> joint_indices =
      VectorX<int>::LinSpaced(kNumJoints, 0, kNumJoints - 1);
  int end_effector_body_idx = robot->FindBodyIndex("iiwa_link_ee");
  int world_body_idx = robot->FindBodyIndex("world");
  const double kEndEffectorToMidFingerDepth = 0.12;
  Vector3<double> end_effector_points{kEndEffectorToMidFingerDepth, 0, 0};

  // Create vectors to hold the constraint objects
  //std::vector<std::unique_ptr<WorldPositionConstraint>> position_constraints;
  std::vector<std::unique_ptr<PostureConstraint>> posture_constraints;
  std::vector<std::unique_ptr<WorldQuatConstraint>> orientation_constraints;
  std::vector<std::unique_ptr<Point2LineSegDistConstraint>> point_to_line_seg_constraints;
  std::vector<std::unique_ptr<WorldGazeDirConstraint>> gaze_dir_constraints;
  std::vector<std::unique_ptr<PostureChangeConstraint>> posture_change_constraints;
  std::vector<RigidBodyConstraint*> constraint_array;

  auto kinematics_cache = robot->CreateKinematicsCache();
  kinematics_cache.initialize(q_current);
  robot->doKinematics(kinematics_cache);
  std::pair<Isometry3<double>, Isometry3<double>> X_WE;
  Isometry3<double> X_LE{Isometry3<double>::Identity()};
  X_LE.translation()[0] = kEndEffectorToMidFingerDepth;
  X_WE.first = robot->relativeTransform(kinematics_cache, world_body_idx,
                                        end_effector_body_idx) * X_LE;

  times->clear();
  times->push_back(0);
  for (auto waypoint = waypoints_start; waypoint != waypoints_end; ++waypoint) {
    kinematics_cache.initialize(waypoint->q);
    robot->doKinematics(kinematics_cache);
    X_WE.second = robot->relativeTransform(kinematics_cache, world_body_idx,
                                          end_effector_body_idx) * X_LE;
    std::pair<Vector3<double>, Vector3<double>> r_WE{X_WE.first.translation(),
                                                     X_WE.second.translation()};
    std::pair<Quaternion<double>, Quaternion<double>> quat_WE{
        Quaternion<double>(X_WE.first.linear()),
        Quaternion<double>(X_WE.second.linear())};

    drake::log()->debug(
        "Planning straight line from {} {} to {} {}", r_WE.first.transpose(),
        math::rotmat2rpy(X_WE.first.rotation()).transpose(), r_WE.second.transpose(),
        math::rotmat2rpy(X_WE.second.rotation()).transpose());

    // Define active times for initial, final, and intermediate constraints
    const double segment_start_time = times->back();
    const double segment_end_time = segment_start_time + waypoint->duration;
    const Vector2<double> intermediate_tspan{segment_start_time,
                                             segment_end_time};
    const Vector2<double> waypoint_tspan{segment_end_time-0.1, segment_end_time+0.1};
    const double dt = waypoint->duration / (waypoint->num_via_points);
    for (int i = 0; i < waypoint->num_via_points; ++i) {
      times->push_back(times->back() + dt);
    }

    // Constraints at waypoint
    posture_constraints.emplace_back(
        new PostureConstraint(robot.get(), waypoint_tspan));
    const VectorX<double> q_lb{waypoint->q.array()};
    const VectorX<double> q_ub{waypoint->q.array()};
    posture_constraints.back()->setJointLimits(joint_indices, q_lb, q_ub);
    constraint_array.push_back(posture_constraints.back().get());

    //position_constraints.emplace_back(new WorldPositionConstraint(
        //robot.get(), end_effector_body_idx, end_effector_points,
        //r_WE.second - waypoint->position_tolerance,
        //r_WE.second + waypoint->position_tolerance, waypoint_tspan));
    //constraint_array.push_back(position_constraints.back().get());

    //orientation_constraints.emplace_back(new WorldQuatConstraint(
        //robot.get(), end_effector_body_idx,
        //Eigen::Vector4d(quat_WE.second.w(), quat_WE.second.x(),
                        //quat_WE.second.y(), quat_WE.second.z()),
        //waypoint->orientation_tolerance, waypoint_tspan));
    //constraint_array.push_back(orientation_constraints.back().get());

    if (waypoint->constrain_intermediate_points) {
      // Construct intermediate constraints for via points
      // Find axis-angle representation of the rotation from X_WEndEffector0 to
      // X_WEndEffector1.
      Isometry3<double> X_10 = X_WE.second.inverse() * X_WE.first;
      Eigen::AngleAxis<double> aaxis{X_10.linear()};
      Vector3<double> axis_E{aaxis.axis()};
      Vector3<double> dir_W{X_WE.first.linear() * axis_E};
      drake::log()->debug("Axis in EE frame: ({})", axis_E.transpose());
      drake::log()->debug("Dir in world frame: ({})", dir_W.transpose());

      // We will impose a Point2LineSegDistConstraint and WorldGazeDirConstraint
      // on the via points.
      Eigen::Matrix<double, 3, 2> line_ends_W;
      line_ends_W << r_WE.first, r_WE.second;

      double dist_lb{0.0};
      double dist_ub{waypoint->via_points_position_tolerance};
      point_to_line_seg_constraints.emplace_back(
          new Point2LineSegDistConstraint(robot.get(), end_effector_body_idx,
                                          end_effector_points, world_body_idx,
                                          line_ends_W, dist_lb, dist_ub,
                                          intermediate_tspan));
      constraint_array.push_back(point_to_line_seg_constraints.back().get());

      gaze_dir_constraints.emplace_back(new WorldGazeDirConstraint(
          robot.get(), end_effector_body_idx, axis_E, dir_W,
          waypoint->via_points_orientation_tolerance, intermediate_tspan));

      orientation_constraints.emplace_back(new WorldQuatConstraint(
          robot.get(), end_effector_body_idx,
          Eigen::Vector4d(quat_WE.second.w(), quat_WE.second.x(),
                          quat_WE.second.y(), quat_WE.second.z()),
          waypoint->via_points_orientation_tolerance, intermediate_tspan));
      if (std::abs(aaxis.angle()) < waypoint->via_points_orientation_tolerance) {
        drake::log()->debug("Using quat constraint for intermediate points");
        constraint_array.push_back(orientation_constraints.back().get());
      } else {
        drake::log()->debug("Using gaze constraint for intermediate points: Angle: {} deg", aaxis.angle()*180.0/M_PI);
        constraint_array.push_back(gaze_dir_constraints.back().get());
      }

      const VectorX<double> ub_change =
          5.0/waypoint->num_via_points * (waypoint->q.array() - q_current.array()).abs().matrix();
      const VectorX<double> lb_change = -ub_change;
      for (int i = 0; i < waypoint->num_via_points; ++i) {
        Vector2<double> tspan{*(times->crbegin()+(i+1)), *(times->crbegin()+i)};
        posture_change_constraints.emplace_back(
            new PostureChangeConstraint(robot.get(), joint_indices, lb_change,
              ub_change, tspan));
        constraint_array.push_back(posture_change_constraints.back().get());
      }
    }
    X_WE.first = X_WE.second;
  }

  const int kNumKnots = times->size();

  std::default_random_engine rand_generator{1234};
  //MatrixX<double> q_nom{MatrixX<double>::Zero(kNumJoints, kNumKnots)};
  MatrixX<double> q_nom{q_seed};
  drake::log()->debug("q_seed_local.cols() = {}, kNumKnots = {}", q_seed_local.cols(), kNumKnots);
  DRAKE_THROW_UNLESS(q_seed_local.rows() == kNumJoints && q_seed_local.cols() == kNumKnots);
  int kNumSamplesPerKnot{2};
  Eigen::RowVectorXd t_samples{kNumSamplesPerKnot*(kNumKnots-1)};
  q_seed_local.col(0) = q_current;
  for (int i = 0; i < kNumKnots; ++i) {
    if (i < kNumKnots - 1) {
      for (int j = 0; j < kNumSamplesPerKnot; ++j) {
        double s(static_cast<double>(j+1)/static_cast<double>(kNumSamplesPerKnot+1));
        t_samples(kNumSamplesPerKnot*i + j) = (1-s)*times->at(i) + s*times->at(i+1);
      }
    }
  }
  const Eigen::Map<Eigen::VectorXd> t{times->data(), kNumKnots};
  IKoptions ikoptions(robot.get());
  ikoptions.setFixInitialState(true);
  drake::log()->debug("t_samples = {}", t_samples);
  //ikoptions.setAdditionaltSamples(t_samples);
  //ikoptions.setQ(MatrixX<double>::Zero(robot->get_num_positions(),
                                       //robot->get_num_positions()));
  ikoptions.setQa(MatrixX<double>::Identity(robot->get_num_positions(),
                                            robot->get_num_positions()));
  ikoptions.setQv(MatrixX<double>::Identity(robot->get_num_positions(),
                                            robot->get_num_positions()));
  ikoptions.setMajorOptimalityTolerance(1e-6);
  //ikoptions.setIterationsLimit(2e4);
  const int kNumRestarts = (waypoints_start->fall_back_to_joint_space_interpolation) ? 5 : 50;
  for (int i = 0; i < kNumRestarts; ++i) {
    *ik_res = inverseKinTrajSimple(robot.get(), t, q_seed_local, q_nom,
        constraint_array, ikoptions);
    if (ik_res->info[0] == 1) {
      break;
    } else {
      //VectorX<double> q_end{robot->getRandomConfiguration(rand_generator)};
      for (int j = 1; j < kNumKnots; ++j) {
        //double s = j/(kNumKnots-1);
        //q_seed_local.col(j) = (1-s)*q_current + s*q_end;
        q_seed_local.col(j) = robot->getRandomConfiguration(rand_generator);
      }
      drake::log()->warn("Attempt {} failed with info {}", i, ik_res->info[0]);
    }
  }
  bool planner_result{ik_res->info[0] == 1};
  if (!planner_result &&
      waypoints_start->fall_back_to_joint_space_interpolation) {
    planner_result = true;
    PiecewisePolynomial<double> q_traj =
        PiecewisePolynomial<double>::FirstOrderHold(
            {times->front(), times->back()}, {q_current, waypoints_start->q});
    for (int j = 0; j < kNumKnots; ++j) {
      ik_res->q_sol[j] = q_traj.value(t[j]);
    }
  }
  drake::log()->debug("result: {}", planner_result);
  drake::log()->debug("t = ({})", t.transpose());
  return planner_result;
}

void ExecuteSingleWaypointMove(
    const WorldState& env_state, const RigidBodyTree<double>& iiwa,
    const std::map<PickAndPlaceState, VectorX<double>>& nominal_q_map,
    const std::vector<Waypoint>::const_iterator& waypoint,
    const PickAndPlaceStateMachine::IiwaPublishCallback& iiwa_callback,
    IiwaMove* iiwa_move) {
  VectorX<double> q_current{env_state.get_iiwa_q()};
  int num_knots{waypoint->num_via_points + 1};
  MatrixX<double> q_seed{q_current.rows(), num_knots};
  for (int j = 0; j < num_knots; ++j) {
    double s{static_cast<double>(j) / (static_cast<double>(num_knots) - 1)};
    q_seed.col(j) =
        (1 - s) * q_current + s * nominal_q_map.at(waypoint->state);
  }
  std::vector<double> times;
  IKResults ik_res;
  bool res = PlanStraightLineMotion(q_current, q_seed, waypoint,
                                    waypoint + 1, iiwa, &ik_res, &times);
  DRAKE_THROW_UNLESS(res);

  robotlocomotion::robot_plan_t plan{};
  iiwa_move->MoveJoints(env_state, iiwa, times, ik_res.q_sol, &plan);
  iiwa_callback(&plan);
}

// 
//  (ApproachPickPregrasp,                               (ApproachPlacePregrasp
//   LiftFromPick ),                                      LiftFromPlace)
//       +--------------------------------------------------------+
//       |                                                        |
//       |                                                        |
//       + (ApproachPick)                         (ApproachPlace) +
void ComputeDesiredPoses(
    const WorldState& env_state, const Isometry3<double>& place_location,
    std::map<PickAndPlaceState, Isometry3<double>>* X_WE_desired) {
  X_WE_desired->clear();

  // Set ApproachPick pose
  X_WE_desired->emplace(kApproachPick,
                        env_state.get_object_pose());

  // Set ApproachPickPregrasp pose
  Isometry3<double> X_OE{Isometry3<double>::Identity()};
  X_OE.translation()[2] = kPreGraspHeightOffset;
  //X_OE.translation()[0] = -kPreGraspHeightOffset/2;
  X_WE_desired->emplace(kApproachPickPregrasp, X_WE_desired->at(kApproachPick)*X_OE);
  //X_WE_desired->at(kApproachPickPregrasp).translation()[2] +=
      //kPreGraspOffset;

  X_OE.setIdentity();
  X_OE.rotate(Eigen::AngleAxis<double>(-M_PI_4, Vector3<double>::UnitY()));
  X_OE.translation()[2] = 1.5*kPreGraspHeightOffset;
  X_OE.translation()[0] = -1.5*kPreGraspHeightOffset/2;
  X_WE_desired->emplace(kPrep, X_WE_desired->at(kApproachPickPregrasp)*X_OE);

  // Set LiftFromPick pose
  X_WE_desired->emplace(kLiftFromPick, X_WE_desired->at(kApproachPick));
  X_WE_desired->at(kLiftFromPick).translation()[2] += kPreGraspHeightOffset;

  // Set ApproachPlace pose
  X_WE_desired->emplace(
      kApproachPlace,
      env_state.get_iiwa_base() * place_location);

  // Set ApproachPlacePregrasp pose
  X_WE_desired->emplace(kApproachPlacePregrasp,
                        X_WE_desired->at(kApproachPlace));
  X_WE_desired->at(kApproachPlacePregrasp).translation()[2] +=
      kPreGraspHeightOffset;

  // Set LiftFromPlace pose
  X_WE_desired->emplace(kLiftFromPlace,
                        X_WE_desired->at(kApproachPlacePregrasp));
}

void ComputeNominalConfigurations(
    const RigidBodyTree<double>& iiwa,
    const VectorX<double>& q_seed,
    const std::map<PickAndPlaceState, Isometry3<double>>& X_WE_desired,
    const Vector3<double>& position_tolerance, double orientation_tolerance,
    std::map<PickAndPlaceState, VectorX<double>>* nominal_q_map) {
  nominal_q_map->clear();
  std::default_random_engine rand_generator{1234};
  std::unique_ptr<RigidBodyTree<double>> robot{iiwa.Clone()};
  int kNumJoints{robot->get_num_positions()};
  IKoptions ikoptions(robot.get());
  ikoptions.setFixInitialState(false);
  ikoptions.setQ(MatrixX<double>::Zero(robot->get_num_positions(),
                                       robot->get_num_positions()));
  ikoptions.setQv(MatrixX<double>::Identity(robot->get_num_positions(),
                                       robot->get_num_positions()));
  int end_effector_body_idx = robot->FindBodyIndex("iiwa_link_ee");
  const double kEndEffectorToMidFingerDepth = 0.12;
  Vector3<double> end_effector_points{kEndEffectorToMidFingerDepth, 0, 0};
  //std::vector<PickAndPlaceState> pick_states{kApproachPickPregrasp,
                                             //kApproachPick, kLiftFromPick};
  //std::vector<PickAndPlaceState> place_states{kApproachPlacePregrasp,
                                              //kApproachPlace, kLiftFromPlace};
  std::vector<PickAndPlaceState> pick_and_place_states{
      kPrep, kApproachPickPregrasp,  kApproachPick,  kLiftFromPick,
      kApproachPlacePregrasp, kApproachPlace, kLiftFromPlace};
  VectorX<double> q_seed_local{q_seed};
  //for (std::vector<PickAndPlaceState> states : {pick_states, place_states}) {
  VectorX<double> t{7};
  t << 0, 1, 2, 3, 4, 5, 6;
  int t_count{0};
  for (std::vector<PickAndPlaceState> states : {pick_and_place_states}) {
    // Set up an inverse kinematics trajectory problem with one knot for each
    // state
    int kNumKnots = states.size();
    MatrixX<double> q_nom =
        MatrixX<double>::Zero(robot->get_num_positions(), kNumKnots);

    //  Create vectors to hold the constraint objects
    std::vector<std::unique_ptr<WorldPositionConstraint>> position_constraints;
    std::vector<std::unique_ptr<WorldQuatConstraint>> orientation_constraints;
    std::vector<std::unique_ptr<PostureChangeConstraint>> posture_change_constraints;
    std::vector<RigidBodyConstraint*> constraint_array;
    for (int i = 0; i < kNumKnots; ++i) {
      // Extract desired position and orientation of end effector at the given
      // state
      Vector2<double> tspan{t(t_count), t(t_count)};
      PickAndPlaceState state{states[i]};
      const Isometry3<double>& X_WE = X_WE_desired.at(state);
      const Vector3<double>& r_WE = X_WE.translation();
      const Quaternion<double>& quat_WE{X_WE.rotation()};

      drake::log()->debug("State {}, r_WE = ({}), tspan = ({})", state,
                          r_WE.transpose(), tspan.transpose());

      // Construct position and orientation constraints
      position_constraints.emplace_back(new WorldPositionConstraint(
          robot.get(), end_effector_body_idx, end_effector_points,
          r_WE - position_tolerance, r_WE + position_tolerance, tspan));
      orientation_constraints.emplace_back(new WorldQuatConstraint(
          robot.get(), end_effector_body_idx,
          Eigen::Vector4d(quat_WE.w(), quat_WE.x(), quat_WE.y(), quat_WE.z()),
          orientation_tolerance, tspan));
      constraint_array.push_back(position_constraints.back().get());
      constraint_array.push_back(orientation_constraints.back().get());
      ++t_count;
    }
    const VectorX<int> joint_indices = VectorX<int>::LinSpaced(kNumJoints, 0, kNumJoints-1);
    VectorX<double> ub_change = M_PI_4*VectorX<double>::Ones(kNumJoints);
    VectorX<double> lb_change = -ub_change;

    for (int i : {0, 1, 2, 4, 5}) {
      posture_change_constraints.emplace_back(new PostureChangeConstraint(
          robot.get(), joint_indices, lb_change, ub_change,
          Vector2<double>(t(i)-0.1, t(i + 1)+0.1)));
      constraint_array.push_back(posture_change_constraints.back().get());
    }

    ub_change = 0.75*M_PI*VectorX<double>::Ones(kNumJoints);
    lb_change = -ub_change;
    posture_change_constraints.emplace_back(
        new PostureChangeConstraint(robot.get(), joint_indices, lb_change,
                                    ub_change, Vector2<double>(t(3), t(4))));
    constraint_array.push_back(posture_change_constraints.back().get());

    // Solve the IK problem
    const int kNumRestarts = 50;
    IKResults ik_res;
    for (int i = 0; i < kNumRestarts; ++i) {
      MatrixX<double> q_knots_seed{robot->get_num_positions(), kNumKnots};
      for (int j = 0; j < kNumKnots; ++j) {
        q_knots_seed.col(j) = q_seed_local;
      }
      drake::log()->debug("Attempt {}: t = ({})", i, t.transpose());
      ik_res = inverseKinTrajSimple(robot.get(), t, q_knots_seed, q_nom,
          constraint_array, ikoptions);
      if (ik_res.info[0] == 1) {
        q_seed_local = ik_res.q_sol.back();
        break;
      } else {
        q_seed_local = robot->getRandomConfiguration(rand_generator);
        drake::log()->warn("Attempt {} failed with info {}", i, ik_res.info[0]);
      }
    }
    DRAKE_THROW_UNLESS(ik_res.info[0] ==1);
    drake::log()->debug("Num knots in sol: {}", ik_res.q_sol.size());
    //for (int i = 0; i < kNumKnots; i+=2) {
    for (int i = 0; i < kNumKnots; ++i) {
      drake::log()->debug("State {}: q = ({})", states[i], ik_res.q_sol[i].transpose());
      nominal_q_map->emplace(states[i], ik_res.q_sol[i]);
    }
  }
}

}  // namespace


PickAndPlaceStateMachine::PickAndPlaceStateMachine(
    const std::vector<Isometry3<double>>& place_locations, bool loop)
    : place_locations_(place_locations),
      next_place_location_(0),
      loop_(loop),
      state_(kOpenGripper),
      // Position and rotation tolerances.  These were hand-tuned by
      // adjusting to tighter bounds until IK stopped reliably giving
      // results.
      tight_pos_tol_(0.001, 0.001, 0.001),
      tight_rot_tol_(0.05),
      loose_pos_tol_(0.01, 0.01, 0.01),
      loose_rot_tol_(0.1) {
  DRAKE_THROW_UNLESS(!place_locations.empty());
}

PickAndPlaceStateMachine::~PickAndPlaceStateMachine() {}

void PickAndPlaceStateMachine::Update(
    const WorldState& env_state,
    const IiwaPublishCallback& iiwa_callback,
    const WsgPublishCallback& wsg_callback,
    manipulation::planner::ConstraintRelaxingIk* planner) {
  IKResults ik_res;
  std::vector<double> times;
  robotlocomotion::robot_plan_t stopped_plan{};
  stopped_plan.num_states = 0;

  const RigidBodyTree<double>& iiwa = planner->get_robot();

  switch (state_) {
      // Opens the gripper.
      case kOpenGripper: {
        if (!wsg_act_.ActionStarted()) {
          lcmt_schunk_wsg_command msg;
          wsg_act_.OpenGripper(env_state, &msg);
          wsg_callback(&msg);

          drake::log()->info("kOpenGripper at {}",
                             env_state.get_iiwa_time());
        }

        if (wsg_act_.ActionFinished(env_state)) {
          state_ = kPrep;
          wsg_act_.Reset();
        }
        break;
      }

      case kPrep: {
        // Compute all the desired end-effector poses now
        if (X_WE_desired_.empty()) {
          ComputeDesiredPoses(env_state, place_locations_[next_place_location_],
                              &X_WE_desired_);
          ComputeNominalConfigurations(iiwa, env_state.get_iiwa_q(),
                                       X_WE_desired_, tight_pos_tol_,
                                       tight_rot_tol_, &nominal_q_map_);
          waypoints_.clear();
          waypoints_.emplace_back();
          waypoints_.back().state = kPrep;
          waypoints_.back().num_via_points = 1;
          waypoints_.back().duration = 2;
          waypoints_.back().constrain_intermediate_points = false;
          waypoints_.back().fall_back_to_joint_space_interpolation = true;

          waypoints_.emplace_back();
          waypoints_.back().state = kApproachPickPregrasp;
          waypoints_.back().X_WE = X_WE_desired_.at(kApproachPickPregrasp);
          waypoints_.back().num_via_points = 3;
          waypoints_.back().duration = 1;
          waypoints_.back().constrain_intermediate_points = true;
          waypoints_.back().fall_back_to_joint_space_interpolation = true;

          waypoints_.emplace_back();
          waypoints_.back().state = kApproachPick;
          waypoints_.back().X_WE = X_WE_desired_.at(kApproachPick);
          waypoints_.back().num_via_points = 3;
          waypoints_.back().duration = 1;
          waypoints_.back().constrain_intermediate_points = true;

          waypoints_.emplace_back();
          waypoints_.back().state = kLiftFromPick;
          waypoints_.back().X_WE = X_WE_desired_.at(kLiftFromPick);
          waypoints_.back().num_via_points = 3;
          waypoints_.back().duration = 1;
          waypoints_.back().constrain_intermediate_points = true;

          waypoints_.emplace_back();
          waypoints_.back().state = kApproachPlacePregrasp;
          waypoints_.back().X_WE = X_WE_desired_.at(kApproachPlacePregrasp);
          waypoints_.back().num_via_points = 3;
          waypoints_.back().duration = 2;
          waypoints_.back().constrain_intermediate_points = true;
          waypoints_.back().fall_back_to_joint_space_interpolation = true;

          waypoints_.emplace_back();
          waypoints_.back().state = kApproachPlace;
          waypoints_.back().X_WE = X_WE_desired_.at(kApproachPlace);
          waypoints_.back().num_via_points = 3;
          waypoints_.back().duration = 1;
          waypoints_.back().constrain_intermediate_points = true;

          waypoints_.emplace_back();
          waypoints_.back().state = kLiftFromPlace;
          waypoints_.back().X_WE = X_WE_desired_.at(kLiftFromPlace);
          waypoints_.back().num_via_points = 3;
          waypoints_.back().duration = 1;
          waypoints_.back().constrain_intermediate_points = true;

          for (auto waypoint = waypoints_.begin();
               waypoint != waypoints_.end(); ++waypoint) {
            waypoint->position_tolerance = tight_pos_tol_;
            waypoint->orientation_tolerance = tight_rot_tol_;
            waypoint->via_points_position_tolerance = tight_pos_tol_(1);
            waypoint->via_points_orientation_tolerance = tight_rot_tol_;
            waypoint->q = nominal_q_map_.at(waypoint->state);
          }
        }

        if (!iiwa_move_.ActionStarted()) {
          bool skip_to_approach_pick =
              nominal_q_map_.at(kApproachPickPregrasp)
                  .isApprox(env_state.get_iiwa_q(), 10 * M_PI / 180);
          if (skip_to_approach_pick) {
            state_ = kApproachPick;
            next_waypoint_ = waypoints_.begin() + 2;
          } else {
            next_waypoint_ = waypoints_.begin();
            next_waypoint_->via_points_position_tolerance = 0.3;
            ExecuteSingleWaypointMove(env_state, iiwa, nominal_q_map_,
                                      next_waypoint_, iiwa_callback,
                                      &iiwa_move_);

            drake::log()->info("kPrep at {}", env_state.get_iiwa_time());
          }
        }
        if (iiwa_move_.ActionFinished(env_state)) {
          state_ = kApproachPickPregrasp;
          next_waypoint_ += 1;
          iiwa_move_.Reset();
        }
        break;
      }
      case kApproachPickPregrasp: {
        // Approaches kPreGraspHeightOffset above the center of the object.
        if (!iiwa_move_.ActionStarted()) {
          next_waypoint_ = waypoints_.begin();
          next_waypoint_->via_points_position_tolerance = 0.3;
          ExecuteSingleWaypointMove(env_state, iiwa, nominal_q_map_,
                                    next_waypoint_, iiwa_callback, &iiwa_move_);
          ++next_waypoint_;
          next_waypoint_->via_points_position_tolerance = 0.3;
          ExecuteSingleWaypointMove(env_state, iiwa, nominal_q_map_,
                                    next_waypoint_, iiwa_callback, &iiwa_move_);

          drake::log()->info("kApproachPickPregrasp at {}",
                             env_state.get_iiwa_time());
        }

        if (iiwa_move_.ActionFinished(env_state)) {
          state_ = kApproachPick;
          next_waypoint_ += 1;
          iiwa_move_.Reset();
        }
        break;
    }

    case kApproachPick: {
      if (!iiwa_move_.ActionStarted()) {
        ExecuteSingleWaypointMove(env_state, iiwa,
                                      nominal_q_map_, next_waypoint_,
                                      iiwa_callback, &iiwa_move_);

        drake::log()->info("kApproachPick at {}", env_state.get_iiwa_time());
      }

      if (iiwa_move_.ActionFinished(env_state)) {
        state_ = kGrasp;
        next_waypoint_ += 1;
        iiwa_move_.Reset();
      }
      break;
    }

    case kGrasp: {
      // Grasps the object.
      if (!wsg_act_.ActionStarted()) {
        lcmt_schunk_wsg_command msg;
        wsg_act_.CloseGripper(env_state, &msg);
        wsg_callback(&msg);

        drake::log()->info("kGrasp at {}", env_state.get_iiwa_time());
      }

      if (wsg_act_.ActionFinished(env_state)) {
        state_ = kLiftFromPick;
        wsg_act_.Reset();
      }
      break;
    }

    case kLiftFromPick: {
      // Lifts the object straight up.
      if (!iiwa_move_.ActionStarted()) {
        ExecuteSingleWaypointMove(env_state, iiwa,
                                      nominal_q_map_, next_waypoint_,
                                      iiwa_callback, &iiwa_move_);

        drake::log()->info("kLiftFromPick at {}", env_state.get_iiwa_time());
      }

      if (iiwa_move_.ActionFinished(env_state)) {
        state_ = kApproachPlacePregrasp;
        next_waypoint_ += 1;
        iiwa_move_.Reset();
      }
      break;
    }

    case kApproachPlacePregrasp: {
      if (!iiwa_move_.ActionStarted()) {
        next_waypoint_->via_points_position_tolerance = 0.3;
        ExecuteSingleWaypointMove(env_state, iiwa,
                                      nominal_q_map_, next_waypoint_,
                                      iiwa_callback, &iiwa_move_);

        drake::log()->info("kApproachPlacePregrasp at {}", env_state.get_iiwa_time());
      }

      if (iiwa_move_.ActionFinished(env_state)) {
        state_ = kApproachPlace;
        next_waypoint_ += 1;
        iiwa_move_.Reset();
      }
      break;
    }

    case kApproachPlace: {
      if (!iiwa_move_.ActionStarted()) {
        ExecuteSingleWaypointMove(env_state, iiwa,
                                      nominal_q_map_, next_waypoint_,
                                      iiwa_callback, &iiwa_move_);

        drake::log()->info("kApproachPlace at {}", env_state.get_iiwa_time());
      }

      if (iiwa_move_.ActionFinished(env_state)) {
        state_ = kPlace;
        next_waypoint_ += 1;
        iiwa_move_.Reset();
      }
      break;
    }

    case kPlace: {
      // Releases the object.
      if (!wsg_act_.ActionStarted()) {
        lcmt_schunk_wsg_command msg;
        wsg_act_.OpenGripper(env_state, &msg);
        wsg_callback(&msg);

        drake::log()->info("kPlace at {}", env_state.get_iiwa_time());
      }

      if (wsg_act_.ActionFinished(env_state)) {
        state_ = kLiftFromPlace;
        wsg_act_.Reset();
      }
      break;
    }

    case kLiftFromPlace: {
      // Moves straight up.
      if (!iiwa_move_.ActionStarted()) {
        ExecuteSingleWaypointMove(env_state, iiwa,
                                      nominal_q_map_, next_waypoint_,
                                      iiwa_callback, &iiwa_move_);


        drake::log()->info("kLiftFromPlace at {}", env_state.get_iiwa_time());
      }

      if (iiwa_move_.ActionFinished(env_state)) {
        next_place_location_++;
        if (next_place_location_ == static_cast<int>(place_locations_.size()) &&
            !loop_) {
          state_ = kDone;
          iiwa_callback(&stopped_plan);
          drake::log()->info("kDone at {}", env_state.get_iiwa_time());
        } else {
          next_place_location_ %= place_locations_.size();
          X_WE_desired_.clear();
          state_ = kOpenGripper;
        }
        iiwa_move_.Reset();
      }
      break;
    }

    case kDone: {
      break;
    }
  }
}

}  // namespace pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
