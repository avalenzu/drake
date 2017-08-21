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

// Computes the desired end effector pose in the world frame given the object
// pose in the world frame.
Isometry3<double> ComputeGraspPose(const Isometry3<double>& X_WObj) {
  // Sets desired end effector location to be 12cm behind the object,
  // with the same orientation relative to the object frame. This number
  // dependents on the length of the finger and how the gripper is attached.
  const double kEndEffectorToMidFingerDepth = 0.12;
  Isometry3<double> X_ObjEndEffector_desired;
  X_ObjEndEffector_desired.translation() =
      Vector3<double>(-kEndEffectorToMidFingerDepth, 0, 0);
  X_ObjEndEffector_desired.linear().setIdentity();
  return X_WObj * X_ObjEndEffector_desired;
}

// Generates a sequence (@p num_via_points + 1) of key frames s.t. the end
// effector moves in a straight line between @pX_WEndEffector0 and
// @p X_WEndEffector1. 
bool PlanStraightLineMotion(const VectorX<double>& q_current,
                            const int num_via_points, double duration,
                            const Isometry3<double>& X_WEndEffector0,
                            const Isometry3<double>& X_WEndEffector1,
                            const Vector3<double>& via_points_pos_tolerance,
                            const double via_points_rot_tolerance,
                            ConstraintRelaxingIk* planner, IKResults* ik_res,
                            std::vector<double>* times) {
  DRAKE_THROW_UNLESS(duration > 0 && num_via_points >= 0);

  std::unique_ptr<RigidBodyTree<double>> robot{planner->get_robot().Clone()};
  int end_effector_body_idx = robot->FindBodyIndex("iiwa_link_ee");
  int world_body_idx = robot->FindBodyIndex("world");

  const std::vector<MatrixX<double>> pos = {X_WEndEffector0.translation(),
                                            X_WEndEffector1.translation()};

  const eigen_aligned_std_vector<Quaternion<double>> quats = {
      Quaternion<double>(X_WEndEffector0.linear()),
      Quaternion<double>(X_WEndEffector1.linear())};

  drake::log()->debug("Planning straight line from {} {} to {} {}",
                      pos[0].transpose(),
                      math::rotmat2rpy(X_WEndEffector0.rotation()).transpose(),
                      pos[1].transpose(),
                      math::rotmat2rpy(X_WEndEffector1.rotation()).transpose());

  // Setup knot times
  const double dt = duration / (num_via_points + 1);
  double time = 0;
  times->clear();
  times->push_back(time);
  for (int i = 0; i <= num_via_points; ++i) {
    time += dt;
    times->push_back(time);
  }

  // Define active times for initial, final, and intermediate constraints
  const Vector2<double> initial_tspan{times->front(), times->front()};
  const Vector2<double> intermediate_tspan{times->front(), times->back()};
  const Vector2<double> final_tspan{times->back(), times->back()};

  // Initial and final constraints
  const Vector3<double> end_points_pos_tolerance{0.1 * via_points_pos_tolerance};
  const double end_points_rot_tolerance{0.1 * via_points_rot_tolerance};

  // Construct position and orientation constraints
  WorldPositionConstraint initial_position_constraint{
      robot.get(),
      end_effector_body_idx,
      Vector3<double>::Zero(),
      pos[0] - end_points_pos_tolerance,
      pos[0] + end_points_pos_tolerance,
      initial_tspan};
  WorldQuatConstraint initial_orientation_constraint{
      robot.get(), end_effector_body_idx,
      Eigen::Vector4d(quats[0].w(), quats[0].x(), quats[0].y(), quats[0].z()),
      end_points_rot_tolerance, initial_tspan};

  WorldPositionConstraint final_position_constraint{
      robot.get(),
      end_effector_body_idx,
      Vector3<double>::Zero(),
      pos[1] - end_points_pos_tolerance,
      pos[1] + end_points_pos_tolerance,
      final_tspan};
  WorldQuatConstraint final_orientation_constraint{
      robot.get(), end_effector_body_idx,
      Eigen::Vector4d(quats[1].w(), quats[1].x(), quats[1].y(), quats[1].z()),
      end_points_rot_tolerance, final_tspan};

  // Construct intermediate constraints for via points
  // Find axis-angle representation of the rotation from X_WEndEffector0 to
  // X_WEndEffector1.
  Isometry3<double> X_10 = X_WEndEffector1.inverse() * X_WEndEffector0;
  Eigen::AngleAxis<double> aaxis{X_10.linear()};
  Vector3<double> axis_E{aaxis.axis()};
  Vector3<double> dir_W{X_WEndEffector0.linear() * axis_E};
  drake::log()->debug("Axis in EE frame: ({})", axis_E.transpose());
  drake::log()->debug("Dir in world frame: ({})", dir_W.transpose());

  // We will impose a Point2LineSegDistConstraint and WorldGazeDirConstraint on
  // the via points.
  Eigen::Matrix<double, 3, 2> line_ends_W;
  line_ends_W << X_WEndEffector0.translation(), X_WEndEffector1.translation();


  double dist_lb{0.0};
  double dist_ub{via_points_pos_tolerance(0)};
  Point2LineSegDistConstraint intermediate_position_constraint{
      robot.get(),
      end_effector_body_idx,
      Vector3<double>::Zero(),
      world_body_idx,
      line_ends_W,
      dist_lb,
      dist_ub,
      intermediate_tspan};

  WorldGazeDirConstraint intermediate_orientation_constraint{
      robot.get(), end_effector_body_idx,    axis_E,
      dir_W,       via_points_rot_tolerance, intermediate_tspan};

  std::vector<RigidBodyConstraint*> constraint_array;
  //constraint_array.push_back(&initial_position_constraint);
  //constraint_array.push_back(&initial_orientation_constraint);
  constraint_array.push_back(&intermediate_position_constraint);
  constraint_array.push_back(&intermediate_orientation_constraint);
  constraint_array.push_back(&final_position_constraint);
  constraint_array.push_back(&final_orientation_constraint);

  const int kNumKnots = times->size();
  std::default_random_engine rand_generator{1234};
  MatrixX<double> q_seed{robot->get_num_positions(), kNumKnots};
  MatrixX<double> q_nom =
      MatrixX<double>::Zero(robot->get_num_positions(), kNumKnots);
  for (int j = 0; j < kNumKnots; ++j) {
    q_seed.col(j) = q_current;
  }
  const Eigen::Map<Eigen::VectorXd> t{times->data(), kNumKnots};
  IKoptions ikoptions(robot.get());
  ikoptions.setFixInitialState(true);
  ikoptions.setQ(MatrixX<double>::Zero(robot->get_num_positions(),
                                       robot->get_num_positions()));
  *ik_res = inverseKinTrajSimple(robot.get(), t, q_seed, q_nom,
                                constraint_array, ikoptions);
  bool planner_result{ik_res->info[0] == 1};
  drake::log()->debug("result: {}", planner_result);
  drake::log()->debug("t = ({})", t.transpose());
  return planner_result;
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
                        ComputeGraspPose(env_state.get_object_pose()));

  // Set ApproachPickPregrasp pose
  X_WE_desired->emplace(kApproachPickPregrasp, X_WE_desired->at(kApproachPick));
  X_WE_desired->at(kApproachPickPregrasp).translation()[2] +=
      kPreGraspHeightOffset;

  // Set LiftFromPick pose
  X_WE_desired->emplace(kLiftFromPick, X_WE_desired->at(kApproachPickPregrasp));

  // Set ApproachPlace pose
  X_WE_desired->emplace(
      kApproachPlace,
      ComputeGraspPose(env_state.get_iiwa_base() * place_location));

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
    const std::map<PickAndPlaceState, Isometry3<double>>& X_WE_desired,
    const Vector3<double>& position_tolerance, double orientation_tolerance,
    std::map<PickAndPlaceState, VectorX<double>>* nominal_q_map) {
  std::default_random_engine rand_generator{1234};
  std::unique_ptr<RigidBodyTree<double>> robot{iiwa.Clone()};
  IKoptions ikoptions(robot.get());
  ikoptions.setFixInitialState(false);
  ikoptions.setQ(MatrixX<double>::Zero(robot->get_num_positions(),
                                       robot->get_num_positions()));
  //ikoptions.setQv(MatrixX<double>::Identity(robot->get_num_positions(),
                                       //robot->get_num_positions()));
  int end_effector_body_idx = robot->FindBodyIndex("iiwa_link_ee");
  Matrix3X<double> end_effector_points = Matrix3X<double>::Zero(3, 1);
  std::vector<PickAndPlaceState> pick_states{kApproachPickPregrasp,
                                             kApproachPick, kLiftFromPick};
  std::vector<PickAndPlaceState> place_states{kApproachPlacePregrasp,
                                              kApproachPlace, kLiftFromPlace};
  VectorX<double> q_seed{robot->getRandomConfiguration(rand_generator)};
  for (std::vector<PickAndPlaceState> states : {pick_states, place_states}) {
    // Set up an inverse kinematics trajectory problem with one knot for each
    // state
    int num_knots = states.size();
    VectorX<double> t = VectorX<double>::LinSpaced(num_knots, 0, num_knots - 1);
    MatrixX<double> q_nom =
        MatrixX<double>::Zero(robot->get_num_positions(), num_knots);

    // Create vectors to hold the constraint objects
    std::vector<std::unique_ptr<WorldPositionConstraint>> position_constraints;
    std::vector<std::unique_ptr<WorldQuatConstraint>> orientation_constraints;
    std::vector<RigidBodyConstraint*> constraint_array;
    Vector2<double> tspan;
    tspan << 0, 0;
    for (int i = 0; i < num_knots; ++i) {
      // Extract desired position and orientation of end effector at the given
      // state
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
      tspan[0] += 1.0;
      tspan[1] += 1.0;
    }
    // Solve the IK problem
    const int kNumRestarts = 50;
    IKResults ik_res;
    for (int i = 0; i < kNumRestarts; ++i) {
      MatrixX<double> q_knots_seed{robot->get_num_positions(), num_knots};
      //VectorX<double> q_seed{robot->getZeroConfiguration()};
      for (int j = 0; j < num_knots; ++j) {
        q_knots_seed.col(j) = q_seed;
      }
      drake::log()->debug("Attempt {}: t = ({})", i, t.transpose());
      ik_res = inverseKinTrajSimple(robot.get(), t, q_knots_seed, q_nom,
          constraint_array, ikoptions);
      if (ik_res.info[0] == 1) {
        q_seed = ik_res.q_sol.back();
        break;
      } else {
        q_seed = robot->getRandomConfiguration(rand_generator);
        drake::log()->warn("Attempt {} failed with info {}", i, ik_res.info[0]);
      }
    }
    DRAKE_THROW_UNLESS(ik_res.info[0] ==1);
    drake::log()->debug("Num knots in sol: {}", ik_res.q_sol.size());
    //for (int i = 0; i < num_knots; i+=2) {
    for (int i = 0; i < num_knots; ++i) {
      //nominal_q_map->emplace(states[i/2], ik_res.q_sol[i]);
      //VectorX<double> constraint_value;
      VectorX<double> lb;
      VectorX<double> ub;
      position_constraints[i]->bounds(&t[i], lb, ub);
      drake::log()->debug("State {}: q = ({})", states[i], ik_res.q_sol[i].transpose());
      drake::log()->debug("State {}: lb: ({}), ub: ({})", states[i], lb.transpose(), ub.transpose());
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
      tight_pos_tol_(0.005, 0.005, 0.005),
      tight_rot_tol_(0.05),
      loose_pos_tol_(0.05, 0.05, 0.05),
      loose_rot_tol_(0.5) {
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
          state_ = kApproachPickPregrasp;
          wsg_act_.Reset();
        }
        break;
      }

    case kApproachPickPregrasp: {
      // Compute all the desired end-effector poses now
      ComputeDesiredPoses(env_state, place_locations_[next_place_location_],
                          &X_WE_desired_);
      if (nominal_q_map_.empty()) {
        ComputeNominalConfigurations(planner->get_robot(), X_WE_desired_, tight_pos_tol_, tight_rot_tol_, &nominal_q_map_);
      }

      // Approaches kPreGraspHeightOffset above the center of the object.
      if (!iiwa_move_.ActionStarted()) {
        // Computes the desired end effector pose in the world frame to be
        // kPreGraspHeightOffset above the object.
        X_Wend_effector_0_ = env_state.get_iiwa_end_effector_pose();
        X_Wend_effector_1_ = X_WE_desired_[kApproachPickPregrasp];

        // 2 seconds, no via points.
        //bool res = PlanStraightLineMotion(
            //env_state.get_iiwa_q(), 0, 2,
            //X_Wend_effector_0_, X_Wend_effector_1_,
            //loose_pos_tol_, loose_rot_tol_, planner, &ik_res, &times);
        //DRAKE_THROW_UNLESS(res);

        robotlocomotion::robot_plan_t plan{};
        times.clear();
        times.push_back(0.0);
        times.push_back(2.0);
        std::vector<VectorX<double>> q_knots{
            env_state.get_iiwa_q(), nominal_q_map_.at(kApproachPickPregrasp)};
        iiwa_move_.MoveJoints(env_state, iiwa, times, q_knots, &plan);
        iiwa_callback(&plan);

        drake::log()->info("kApproachPickPregrasp at {}",
                           env_state.get_iiwa_time());
      }

      if (iiwa_move_.ActionFinished(env_state)) {
        state_ = kApproachPick;
        iiwa_move_.Reset();
     }
      break;
    }

    case kApproachPick: {
      // Moves gripper straight down.
      if (!iiwa_move_.ActionStarted()) {
        X_Wend_effector_0_ = X_Wend_effector_1_;
        X_Wend_effector_1_ = X_WE_desired_[kApproachPick];

        // 1 second, 3 via points. More via points to ensure the end effector
        // moves in more or less a straight line.
        bool res = PlanStraightLineMotion(
            env_state.get_iiwa_q(), 3, 1,
            X_Wend_effector_0_, X_Wend_effector_1_,
            tight_pos_tol_, tight_rot_tol_, planner, &ik_res, &times);
        DRAKE_THROW_UNLESS(res);

        robotlocomotion::robot_plan_t plan{};
        iiwa_move_.MoveJoints(env_state, iiwa, times, ik_res.q_sol, &plan);
        iiwa_callback(&plan);

        drake::log()->info("kApproachPick at {}",
                           env_state.get_iiwa_time());
      }

      if (iiwa_move_.ActionFinished(env_state)) {
        state_ = kGrasp;
        iiwa_callback(&stopped_plan);
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
        X_Wend_effector_0_ = X_Wend_effector_1_;
        X_Wend_effector_1_ = X_WE_desired_[kLiftFromPick];

        // 1 second, 3 via points. More via points to ensure the end effector
        // moves in more or less a straight line.
        bool res = PlanStraightLineMotion(
            env_state.get_iiwa_q(), 3, 1,
            X_Wend_effector_0_, X_Wend_effector_1_,
            tight_pos_tol_, tight_rot_tol_, planner, &ik_res, &times);
        DRAKE_THROW_UNLESS(res);

        robotlocomotion::robot_plan_t plan{};
        iiwa_move_.MoveJoints(env_state, iiwa, times, ik_res.q_sol, &plan);
        iiwa_callback(&plan);

        drake::log()->info("kLiftFromPick at {}", env_state.get_iiwa_time());
      }

      if (iiwa_move_.ActionFinished(env_state)) {
        state_ = kApproachPlacePregrasp;
        iiwa_move_.Reset();
      }
      break;
    }

    case kApproachPlacePregrasp: {
      // Uses 2 seconds to move to right about the target place location.
      if (!iiwa_move_.ActionStarted()) {
        X_Wend_effector_0_ = X_Wend_effector_1_;
        X_Wend_effector_1_ = X_WE_desired_[kApproachPlacePregrasp];

        // 2 seconds, no via points.
        //bool res = PlanStraightLineMotion(
            //env_state.get_iiwa_q(), 0, 2,
            //X_Wend_effector_0_, X_Wend_effector_1_,
            //loose_pos_tol_, loose_rot_tol_, planner, &ik_res, &times);
        //DRAKE_THROW_UNLESS(res);

        robotlocomotion::robot_plan_t plan{};
        times.clear();
        times.push_back(0.0);
        times.push_back(2.0);
        std::vector<VectorX<double>> q_knots{
            nominal_q_map_.at(kLiftFromPick),
            nominal_q_map_.at(kApproachPlacePregrasp)};
        iiwa_move_.MoveJoints(env_state, iiwa, times, q_knots, &plan);
        iiwa_callback(&plan);

        drake::log()->info("kApproachPlacePregrasp at {}",
                           env_state.get_iiwa_time());
      }

      if (iiwa_move_.ActionFinished(env_state)) {
        state_ = kApproachPlace;
        iiwa_move_.Reset();
      }
      break;
    }

    case kApproachPlace: {
      // Moves straight down.
      if (!iiwa_move_.ActionStarted()) {
        // Computes the desired end effector pose in the world frame.
        X_Wend_effector_0_ = X_Wend_effector_1_;
        X_Wend_effector_1_ = X_WE_desired_[kApproachPlace];

        // 1 seconds, 3 via points.
        bool res = PlanStraightLineMotion(
            env_state.get_iiwa_q(), 3, 1,
            X_Wend_effector_0_, X_Wend_effector_1_,
            tight_pos_tol_, tight_rot_tol_, planner, &ik_res, &times);
        DRAKE_THROW_UNLESS(res);

        robotlocomotion::robot_plan_t plan{};
        iiwa_move_.MoveJoints(env_state, iiwa, times, ik_res.q_sol, &plan);
        iiwa_callback(&plan);

        drake::log()->info("kApproachPlace at {}", env_state.get_iiwa_time());
      }

      if (iiwa_move_.ActionFinished(env_state)) {
        state_ = kPlace;
        iiwa_callback(&stopped_plan);
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
        X_Wend_effector_0_ = X_Wend_effector_1_;
        X_Wend_effector_1_ = X_WE_desired_[kLiftFromPlace];

        // 2 seconds, 5 via points.
        bool res = PlanStraightLineMotion(
            env_state.get_iiwa_q(), 5, 2,
            X_Wend_effector_0_, X_Wend_effector_1_,
            tight_pos_tol_, tight_rot_tol_, planner, &ik_res, &times);
        DRAKE_THROW_UNLESS(res);

        robotlocomotion::robot_plan_t plan{};
        iiwa_move_.MoveJoints(env_state, iiwa, times, ik_res.q_sol, &plan);
        iiwa_callback(&plan);

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
