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

PoseInterpolator::PoseInterpolator() {
  // Input
  trajectory_input_port_ = this->DeclareAbstractInputPort().get_index();
  // Output
  pose_output_port_ =
      this->DeclareAbstractOutputPort(Isometry3<double>::Identity(),
                                      &PoseInterpolator::CalcPoseOutput)
          .get_index();
  // State
  trajectory_state_index_ = this->DeclareAbstractState(
      Value<PiecewiseCartesianTrajectory<double>>::Make(
          PiecewiseCartesianTrajectory<double>()));
}

void PoseInterpolator::CalcDesiredEndEffectorVelocityOutput(
    const Context<double>& context,
    systems::BasicVector<double>* output) const {
  auto trajectory =
      context.get_abstract_state<PiecewiseCartesianTrajectory<double>>(
          trajectory_state_index_);
  *output = trajectory
}
}  // namespace planner
}  // namespace manipulation
}  // namespace drake
