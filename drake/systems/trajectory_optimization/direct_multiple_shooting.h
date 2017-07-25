#pragma once

#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/constraint.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/trajectory_optimization/direct_trajectory_optimization.h"

namespace drake {
namespace systems {

/**
 * DirectMultipleShootingTranectoryOptimization
 */
class DirectMultipleShootingTrajectoryOptimization : public DirectTrajectoryOptimization {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DirectMultipleShootingTrajectoryOptimization)

  DirectMultipleShootingTrajectoryOptimization(const System<double>* system,
                               const Context<double>& context,
                               int num_time_samples,
                               double trajectory_time_lower_bound,
                               double trajectory_time_upper_bound);

  ~DirectMultipleShootingTrajectoryOptimization() override {}

 private:
  void DoAddRunningCost(const symbolic::Expression& e) override;

  void DoAddRunningCost(std::shared_ptr<solvers::Cost> cost) override;

  const System<double>* system_{nullptr};
  const std::unique_ptr<Context<double>> context_{nullptr};
  const std::unique_ptr<ContinuousState<double>> continuous_state_{nullptr};
};

}  // namespace systems
}  // namespace drake
