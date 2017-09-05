#include "drake/manipulation/planner/kinematic_trajectory_optimization.h"

#include "drake/systems/trajectory_optimization/direct_transcription.h"

using Eigen::MatrixXd;
using drake::systems::LinearSystem;
using drake::systems::trajectory_optimization::DirectTranscription;
using drake::solvers::SolutionResult;

namespace drake {
namespace manipulation {
namespace planner {

KinematicTrajectoryOptimization::KinematicTrajectoryOptimization(
    const RigidBodyTree<double>& tree, int num_time_samples)
    : tree_(tree.Clone()),
      num_time_samples_(num_time_samples) {
  const int kNumStates{3 * num_positions()};
  const int kNumInputs{num_positions()};
  const int kNumOutputs{0};

  const double dt{0.1};
  const MatrixXd kZero{MatrixXd::Zero(num_positions(), num_positions())};
  const MatrixXd kIdentity{
      MatrixXd::Identity(num_positions(), num_positions())};

  MatrixX<double> A{kNumStates, kNumStates};
  A << kIdentity, dt*kIdentity, kZero, kZero, kIdentity, dt*kIdentity, kZero, kZero, kIdentity;

  MatrixXd B{kNumStates, kNumInputs};
  B << kZero, kZero, dt*kIdentity;

  MatrixXd C{kNumOutputs, kNumStates};
  MatrixXd D{kNumOutputs, kNumInputs};

  system_ = std::make_unique<LinearSystem<double>>(A, B, C, D, dt);

  prog_ = std::make_unique<DirectTranscription>(
      system_.get(), *(system_->CreateDefaultContext()), num_time_samples_);
};

void KinematicTrajectoryOptimization::AddRunningCost(
    const symbolic::Expression& g) {
  prog_->AddRunningCost(g);
};

SolutionResult KinematicTrajectoryOptimization::Solve() {
    return prog_->Solve();
};

}  // }amespace planner
}  // namespace manipulation
}  // namespace drake
