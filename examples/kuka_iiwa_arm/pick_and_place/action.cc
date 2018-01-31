#include "drake/examples/kuka_iiwa_arm/pick_and_place/action.h"

#include <limits>

#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/util/lcmUtil.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace pick_and_place {

Action::~Action() {}

bool Action::ActionStarted() const {
  if (act_start_time_ < 0) return false;
  return true;
}

void Action::Reset() { act_start_time_ = -1; }

void Action::StartAction(double start_time) {
  DRAKE_DEMAND(start_time >= 0);
  act_start_time_ = start_time;
}

IiwaMove::IiwaMove() {}

void IiwaMove::MoveJoints(const WorldState& est_state,
                          const std::vector<std::string>& joint_names,
                          const std::vector<double>& time_in,
                          const std::vector<VectorX<double>>& q,
                          robotlocomotion::robot_plan_t* plan) {
  std::vector<double> time = time_in;
  DRAKE_DEMAND(time.size() == q.size());
  DRAKE_DEMAND(plan != nullptr);

  std::vector<int> info(time.size(), 1);
  MatrixX<double> q_mat(q.front().size(), q.size());
  for (size_t i = 0; i < q.size(); ++i) q_mat.col(i) = q[i];
  ApplyJointVelocityLimits(q_mat, &time);
  *plan = EncodeKeyFrames(joint_names, time, info, q_mat);
  StartAction(est_state.get_iiwa_time());
  // Set the duration for this action to be longer than that of the plan to
  // ensure that we do not advance to the next action befor the robot finishes
  // executing the plan.
  const double additional_duaration{0.5};
  duration_ = time.back() + additional_duaration;
}

void IiwaMove::MoveCartesian(const WorldState& est_state,
                             const Isometry3<double>& X_WG_desired,
                             double duration,
                             robotlocomotion::robot_plan_t* plan) {
  DRAKE_DEMAND(plan != nullptr);
  int num_time_steps = 1;
  plan->utime = 0;  // I (sam.creasey) don't think this is used?
  plan->robot_name = "iiwa";  // Arbitrary, probably ignored
  plan->num_states = num_time_steps;
  const bot_core::robot_state_t default_robot_state{};
  plan->plan.resize(num_time_steps, default_robot_state);
  plan->plan_info.resize(num_time_steps, 0);
  /// Encode X_WG_desired into the pose of the robot states.
  for (int i = 0; i < num_time_steps; i++) {
    EncodePose(X_WG_desired, plan->plan[i].pose);
  }
  plan->num_grasp_transitions = 0;
  plan->left_arm_control_type = plan->POSITION;
  plan->right_arm_control_type = plan->NONE;
  plan->left_leg_control_type = plan->NONE;
  plan->right_leg_control_type = plan->NONE;
  plan->num_bytes = 0;

  StartAction(est_state.get_iiwa_time());
  // Set the duration for this action to be longer than that of the plan to
  // ensure that we do not advance to the next action befor the robot finishes
  // executing the plan.
  const double additional_duaration{0.5};
  duration_ = duration + additional_duaration;
}

void IiwaMove::Reset() {
  Action::Reset();
  duration_ = std::numeric_limits<double>::infinity();
}

bool IiwaMove::ActionFinished(const WorldState& est_state) const {
  if (!ActionStarted()) return false;

  const double max_finished_velocity = 1e-1;
  if (get_time_since_action_start(est_state.get_iiwa_time()) > duration_ &&
      est_state.get_iiwa_v().norm() < max_finished_velocity) {
    return true;
  } else {
    return false;
  }
}

WsgAction::WsgAction() {}

void WsgAction::OpenGripper(const WorldState& est_state, double grip_force,
                            lcmt_schunk_wsg_command* msg) {
  StartAction(est_state.get_wsg_time());
  *msg = lcmt_schunk_wsg_command();
  msg->utime = est_state.get_wsg_time() * 1e6;
  msg->target_position_mm = 100;  // Maximum aperture for WSG
  msg->force = grip_force;
  last_command_ = kOpen;
}

void WsgAction::CloseGripper(const WorldState& est_state, double grip_force,
                             lcmt_schunk_wsg_command* msg) {
  StartAction(est_state.get_wsg_time());
  *msg = lcmt_schunk_wsg_command();
  msg->utime = est_state.get_wsg_time() * 1e6;
  msg->target_position_mm = 8;  // 0 would smash the fingers together
                                // and keep applying force on a real
                                // WSG when no object is grasped.
  msg->force = grip_force;
  last_command_ = kClose;
}

bool WsgAction::ActionFinished(const WorldState& est_state) const {
  if (!ActionStarted()) return false;
  if (std::abs(est_state.get_wsg_v()) < kFinalSpeedThreshold &&
      (get_time_since_action_start(est_state.get_wsg_time()) > 0.5)) {
    if (last_command_ == kOpen &&
        est_state.get_wsg_q() > kOpenPositionThreshold) {
      return true;
    } else if (last_command_ == kClose &&
               est_state.get_wsg_q() < kOpenPositionThreshold) {
      return true;
    }
  }
  return false;
}

}  // namespace pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
