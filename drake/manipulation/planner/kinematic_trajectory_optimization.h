#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/primitives/linear_system.h"
#include "drake/systems/trajectory_optimization/multiple_shooting.h"

namespace drake {
namespace manipulation {
namespace planner {

/// This class implements a plant with the following dynamics:
///
/// ⌈ qdot ⌉ = ⌈ 0 I 0 ⌉ ⌈ q ⌉   ⌈ 0 ⌉
/// | vdot | = | 0 0 I | | v | + | 0 | u
/// ⌊ adot ⌋ = ⌊ 0 0 0 ⌋ ⌊ a ⌋   ⌊ I ⌋
///
///
class KinematicTrajectoryOptimization {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(KinematicTrajectoryOptimization)

  KinematicTrajectoryOptimization(const RigidBodyTree<double>& tree,
                                  int num_time_samples);

  int num_time_samples() { return num_time_samples_; };

  const RigidBodyTree<double>& tree() { return *tree_; };

  int num_positions() const { return tree_->get_num_positions(); };

  int num_velocities() const { return tree_->get_num_velocities(); };

  systems::trajectory_optimization::MultipleShooting* mutable_prog() const {
    return prog_.get();
  };

  const solvers::VectorXDecisionVariable& state() const {
    return prog_->state();
  }

  const solvers::VectorXDecisionVariable& input() const {
    return prog_->input();
  }

  /// Adds an integrated cost to all time steps, of the form
  ///    @f[ cost = \int_0^T g(t,x,u) dt, @f]
  /// where any instances of time(), state(), and/or input() placeholder
  /// variables are substituted with the relevant variables for each current
  /// time index.  The particular integration scheme is determined by the
  /// derived class implementation.
  void AddRunningCost(const symbolic::Expression& g);

  template <typename Derived>
  void AddRunningCost(const Eigen::MatrixBase<Derived>& g) {
    DRAKE_DEMAND(g.rows() == 1 && g.cols() == 1);
    prog_->AddRunningCost(g(0, 0));
  }

  void AddSpatialVelocityCost(const std::string& body_name, double weight);

  solvers::SolutionResult Solve();

 private:
  std::unique_ptr<systems::trajectory_optimization::MultipleShooting> prog_;
  std::unique_ptr<const RigidBodyTree<double>> tree_;
  int num_time_samples_{0};
  std::unique_ptr<systems::LinearSystem<double>> system_;
  double dt_{0.1};
};

}  // namespace planner
}  // namespace manipulation
}  // namespace drake
