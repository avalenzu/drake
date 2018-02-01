#include "drake/manipulation/planner/pose_interpolator.h"

#include <map>

#include "drake/common/constants.h"
#include "drake/common/text_logging.h"

namespace drake {
namespace manipulation {
namespace planner {

using systems::BasicVector;
using systems::Context;
using systems::kVectorValued;
using systems::Value;

PoseInterpolator::PoseInterpolator(double update_interval) {
  // Declare inputs.
  trajectory_input_port_ = this->DeclareAbstractInputPort().get_index();
  // Output
  pose_output_port_ =
      this->DeclareAbstractOutputPort(Isometry3<double>::Identity(),
                                      &PoseInterpolator::CalcPoseOutput)
          .get_index();
  // Declare state.
  trajectory_state_index_ = this->DeclareAbstractState(
      Value<PiecewiseCartesianTrajectory<double>>::Make(
          PiecewiseCartesianTrajectory<double>()));
  this->DeclareDiscreteState(1);
  start_time_index_ = 0;

  // Set update rate.
  this->DeclarePeriodicUnrestrictedUpdate(update_interval, 0);
}

PoseInterpolator::~PoseInterpolator() {}

void PoseInterpolator::CalcPoseOutput(const Context<double>& context,
                                      Isometry3<double>* output) const {
  const auto trajectory =
      context.get_abstract_state<PiecewiseCartesianTrajectory<double>>(
          trajectory_state_index_);
  const auto start_time =
      context.get_discrete_state().get_vector().GetAtIndex(start_time_index_);
  if (trajectory.empty()) {
    *output = Isometry3<double>::Identity();
  } else {
    // drake::log()->debug("xyz = {}", output->translation().transpose());
    *output = trajectory.get_pose(context.get_time() - start_time);
  }
}

void PoseInterpolator::DoCalcUnrestrictedUpdate(
    const Context<double>& context,
    const std::vector<const systems::UnrestrictedUpdateEvent<double>*>&,
    systems::State<double>* state) const {
  // Extract the state and input trajectories.
  PiecewiseCartesianTrajectory<double>& trajectory_state =
      state->get_mutable_abstract_state<PiecewiseCartesianTrajectory<double>>(
          trajectory_state_index_);
  const PiecewiseCartesianTrajectory<double>& trajectory_input =
      this->EvalAbstractInput(context, trajectory_input_port_)
          ->GetValue<PiecewiseCartesianTrajectory<double>>();
  // Only update the state if the input is a different trajectory.
  if (trajectory_state.empty() ||
      !trajectory_input.is_approx(trajectory_state, kComparisonTolerance)) {
    trajectory_state = trajectory_input;
    // t = 0 in the input trajectory corresponds to the current time.
    state->get_mutable_discrete_state()
        .get_mutable_vector()
        .get_mutable_value()(start_time_index_) = context.get_time();
    drake::log()->debug(
        "t0' = {}, tf' = {}",
        trajectory_state.get_position_trajectory().get_start_time(),
        trajectory_state.get_position_trajectory().get_end_time());
  }
}
}  // namespace planner
}  // namespace manipulation
}  // namespace drake
