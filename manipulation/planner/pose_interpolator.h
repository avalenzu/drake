#pragma once

#include "drake/manipulation/util/trajectory_utils.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace manipulation {
namespace planner {

/** A system that interpolates SE(3) trajectories
 This system outputs the result of evaluating a PiecewiseCartesianTrajectory at
 the current time. The system checks the trajectory input periodically and
 updates its stored trajectory when a new trajectory appears.

     trajectory ┌────────────────┐pose
     ──────────▶│PoseInterpolator├────▶
                └────────────────┘
 */
class PoseInterpolator final : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PoseInterpolator)

  PoseInterpolator(double update_interval = kDefaultPlanUpdateInterval);
  ~PoseInterpolator() override;

  const systems::InputPortDescriptor<double>& trajectory_input_port() const {
    return this->get_input_port(trajectory_input_port_);
  }

  const systems::OutputPort<double>& pose_output_port() const {
    return this->get_output_port(pose_output_port_);
  }

  const PiecewiseCartesianTrajectory<double>& trajectory_state(
      const systems::Context<double>& context) const {
    return context.get_abstract_state<PiecewiseCartesianTrajectory<double>>(
        trajectory_state_index_);
  }

 protected:
  void DoCalcUnrestrictedUpdate(
      const systems::Context<double>& context,
      const std::vector<const systems::UnrestrictedUpdateEvent<double>*>&,
      systems::State<double>* state) const override;

 private:
  static constexpr double kDefaultPlanUpdateInterval = 0.1;
  static constexpr double kComparisonTolerance = 1e-8;
  void CalcPoseOutput(const systems::Context<double>& context,
                      Isometry3<double>* output) const;
  // Inputs
  int trajectory_input_port_{-1};
  // Outputs
  int pose_output_port_{-1};
  // Abstract state
  int trajectory_state_index_{-1};
};

}  // namespace planner
}  // namespace manipulation
}  // namespace drake
