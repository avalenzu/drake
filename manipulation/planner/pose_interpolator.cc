#include "drake/manipulation/planner/pose_interpolator.h"

#include <map>

#include "drake/common/constants.h"
#include "drake/common/text_logging.h"

namespace drake {
namespace manipulation {
namespace planner {

using systems::BasicVector;
using systems::kVectorValued;
using systems::Context;
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

  // Set update rate.
  this->DeclarePeriodicUnrestrictedUpdate(update_interval, 0);
}

void PoseInterpolator::CalcPoseOutput(const Context<double>& context,
                                      Isometry3<double>* output) const {
  const auto trajectory =
      context.get_abstract_state<PiecewiseCartesianTrajectory<double>>(
          trajectory_state_index_);
  *output = trajectory.get_pose(context.get_time());
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
  if (!trajectory_input.is_approx(trajectory_state, kComparisonTolerance)) {
    trajectory_state = trajectory_input;
    // t = 0 in the input trajectory corresponds to the current time.
    trajectory_state.ShiftRight(context.get_time());
  }
}
}  // namespace planner
}  // namespace manipulation
}  // namespace drake
