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

  KinematicTrajectoryOptimization(std::unique_ptr<RigidBodyTree<double>> tree,
                                  int num_time_samples, double minimum_timestep,
                                  double maximum_timestep);

  int num_time_samples() { return num_time_samples_; };

  const RigidBodyTree<double>& tree() { return *tree_; };

  int num_positions() const { return tree_->get_num_positions(); };

  int num_velocities() const { return tree_->get_num_velocities(); };

  systems::trajectory_optimization::MultipleShooting* mutable_prog() const {
    return prog_.get();
  }

  const solvers::VectorDecisionVariable<1>& time() const {
    return prog_->time();
  }

  const solvers::VectorXDecisionVariable state() const {
    return prog_->state();
  }

  const solvers::VectorXDecisionVariable state(int index) const {
    return prog_->state(index);
  }

  const solvers::VectorXDecisionVariable& input() const {
    return prog_->input();
  }

  const Eigen::VectorBlock<const solvers::VectorXDecisionVariable>
  initial_state() const {
    return prog_->initial_state();
  }

  const Eigen::VectorBlock<const solvers::VectorXDecisionVariable>
  final_state() const {
    return prog_->final_state();
  }

  /// Adds an integrated cost to all time steps, of the form
  ///    @f[ cost = \int_0^T g(t,x,u) dt, @f]
  /// where any instances of time(), state(), and/or input() placeholder
  /// variables are substituted with the relevant variables for each current
  /// time index.  The particular integration scheme is determined by the
  /// derived class implementation.
  void AddRunningCost(const symbolic::Expression& g);

  void AddFinalCost(const symbolic::Expression& g);

  void AddEqualTimeIntervalsConstraints();

  void AddDurationBounds(double lower_bound, double upper_bound);

  /**
   * Set the initial guess for the decision variables stored in @p var to be x0.
   * Variables begin with a default initial guess of NaN to indicate that no
   * guess is available.
   */
  template <typename DerivedA, typename DerivedB>
  void SetInitialGuess(const Eigen::MatrixBase<DerivedA>& decision_variable_mat,
                       const Eigen::MatrixBase<DerivedB>& x0) {
    prog_->SetInitialGuess(decision_variable_mat, x0);
  };

  template <typename Derived>
  void AddRunningCost(const Eigen::MatrixBase<Derived>& g) {
    DRAKE_DEMAND(g.rows() == 1 && g.cols() == 1);
    prog_->AddRunningCost(g(0, 0));
  }

  void AddConstraintToAllKnotPoints(const symbolic::Formula& f);

  void AddSpatialVelocityCost(const std::string& body_name, double weight);

  void AddBodyPoseConstraint(
      int index, const std::string& body_name, const Isometry3<double>& X_WFd,
      double position_tolerance = 0.0, double orientation_tolerance = 0.0,
      const Isometry3<double>& X_BF = Isometry3<double>::Identity());

  void AddCollisionAvoidanceConstraint(double collision_avoidance_threshold);

  solvers::SolutionResult Solve();

  PiecewisePolynomialTrajectory ReconstructStateTrajectory() const;

  PiecewisePolynomialTrajectory ReconstructInputTrajectory() const;

  template<typename T>
  void SetSolverOption(const solvers::SolverId& solver_id,
                       const std::string& solver_option,
                       T option_value);

  void AddLinearConstraint(const symbolic::Formula& f);

 private:
  std::unique_ptr<systems::trajectory_optimization::MultipleShooting> prog_;
  std::unique_ptr<RigidBodyTree<double>> tree_;
  int num_time_samples_{0};
  std::unique_ptr<systems::LinearSystem<double>> system_;

};

}  // namespace planner
}  // namespace manipulation
}  // namespace drake
