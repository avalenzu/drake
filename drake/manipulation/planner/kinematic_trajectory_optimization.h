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

  /// Returns a placeholder decision variable (not actually declared as a
  /// decision variable in the MathematicalProgram) associated with the time, t.
  /// This variable will be substituted for real decision variables at
  /// particular times in methods like AddRunningCost.  Passing this variable
  /// directly into objectives/constraints for the parent classes will result
  /// in an error.
  const solvers::VectorDecisionVariable<1>& time() const {
    return placeholder_t_var_;
  }

  /// Returns a placeholder decision variable (not actually declared as a
  /// decision variable in the MathematicalProgram) associated with the
  /// generalized position vector, q.  This variable will be substituted for
  /// real decision variables at particular times in methods like
  /// AddRunningCost.  Passing this variable directly into
  /// objectives/constraints for the parent classes will result in an error.
  const solvers::VectorXDecisionVariable& position() const {
    return placeholder_q_vars_;
  }

  /// Returns a placeholder decision variable (not actually declared as a
  /// decision variable in the MathematicalProgram) associated with the
  /// generalized velocity vector, q.  This variable will be substituted for
  /// real decision variables at particular times in methods like
  /// AddRunningCost.  Passing this variable directly into
  /// objectives/constraints for the parent classes will result in an error.
  const solvers::VectorXDecisionVariable& velocity() const {
    return placeholder_v_vars_;
  }

  /// Returns a placeholder decision variable (not actually declared as a
  /// decision variable in the MathematicalProgram) associated with the
  /// generalized acceleration vector, q.  This variable will be substituted for
  /// real decision variables at particular times in methods like
  /// AddRunningCost.  Passing this variable directly into
  /// objectives/constraints for the parent classes will result in an error.
  const solvers::VectorXDecisionVariable& acceleration() const {
    return placeholder_a_vars_;
  }

  /// Returns a placeholder decision variable (not actually declared as a
  /// decision variable in the MathematicalProgram) associated with the
  /// generalized jerk vector, q.  This variable will be substituted for
  /// real decision variables at particular times in methods like
  /// AddRunningCost.  Passing this variable directly into
  /// objectives/constraints for the parent classes will result in an error.
  const solvers::VectorXDecisionVariable& jerk() const {
    return placeholder_j_vars_;
  }

  int num_time_samples() { return num_time_samples_; };

  const RigidBodyTree<double>& tree() { return *tree_; };

  int num_positions() const { return tree_->get_num_positions(); };

  int num_velocities() const { return tree_->get_num_velocities(); };

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

  void SetInitialTrajectory(const PiecewisePolynomial<double>& traj_init_q) {
    DRAKE_DEMAND(traj_init_q.rows() == num_positions() &&
                 traj_init_q.cols() == 1);
    initial_position_trajectory_ = traj_init_q;
  }

  template <typename Derived>
  void AddRunningCost(const Eigen::MatrixBase<Derived>& g) {
    DRAKE_DEMAND(g.rows() == 1 && g.cols() == 1);
    AddRunningCost(g(0,0));
  }

  void TrackSpatialVelocityOfBody(const std::string& body_name);

  void AddSpatialVelocityCost(const std::string& body_name, double weight);

  void AddBodyPoseConstraint(
      double time, const std::string& body_name, const Isometry3<double>& X_WFd,
      double position_tolerance = 0.0, double orientation_tolerance = 0.0,
      const Isometry3<double>& X_BF = Isometry3<double>::Identity());

  void AddBodyPoseConstraint(
      Vector2<double> plan_interval, const std::string& body_name,
      const Isometry3<double>& X_WFd, double position_tolerance = 0.0,
      double orientation_tolerance = 0.0,
      const Isometry3<double>& X_BF = Isometry3<double>::Identity());

  void AddCollisionAvoidanceConstraint(Vector2<double> plan_interval,
                                       double collision_avoidance_threshold);

  void AddCollisionAvoidanceConstraint(double collision_avoidance_threshold);

  solvers::SolutionResult Solve();

  PiecewisePolynomialTrajectory GetPositionTrajectory() const;

  template<typename T>
  void SetSolverOption(const solvers::SolverId& solver_id,
                       const std::string& solver_option,
                       T option_value);

  void AddLinearConstraint(const symbolic::Formula& f);

  void AddLinearConstraint(const symbolic::Formula& f, double time);

  void AddLinearConstraint(const symbolic::Formula& f, Vector2<double> plan_interval);

 private:
  struct FormulaWrapper {
    symbolic::Formula formula;
    Vector2<double> plan_interval;
  };

  struct CostWrapper {
    std::shared_ptr<solvers::Cost> cost;
    solvers::VectorXDecisionVariable vars;
    Vector2<double> plan_interval;
  };

  struct ConstraintWrapper {
    std::shared_ptr<solvers::Constraint> constraint;
    solvers::VectorXDecisionVariable vars;
    Vector2<double> plan_interval;
  };

  // Helper method that performs the work for SubstitutePlaceHolderVariables
  symbolic::Substitution ConstructPlaceholderVariableSubstitution(
      const systems::trajectory_optimization::MultipleShooting&) const;
  symbolic::Substitution ConstructPlaceholderVariableSubstitution(
      const systems::trajectory_optimization::MultipleShooting&,
      int index) const;

  solvers::VectorXDecisionVariable SubstitutePlaceholderVariables(
      const solvers::VectorXDecisionVariable& vars,
      const systems::trajectory_optimization::MultipleShooting& prog);

  solvers::VectorXDecisionVariable SubstitutePlaceholderVariables(
      const solvers::VectorXDecisionVariable& vars,
      const systems::trajectory_optimization::MultipleShooting& prog,
      int index);

  symbolic::Formula SubstitutePlaceholderVariables(
      const symbolic::Formula& f,
      const systems::trajectory_optimization::MultipleShooting& prog,
      int index);

  symbolic::Expression SubstitutePlaceholderVariables(
      const symbolic::Expression& g,
      const systems::trajectory_optimization::MultipleShooting& prog);

  void AddConstraintToProgram(
      const ConstraintWrapper& constraint,
      systems::trajectory_optimization::MultipleShooting* prog);

  void AddLinearConstraintToProgram(
      const FormulaWrapper& constraint,
      systems::trajectory_optimization::MultipleShooting* prog);

  void AddRunningCostToProgram(
      const symbolic::Expression& cost,
      systems::trajectory_optimization::MultipleShooting* prog);

  void AddCostToProgram(
      const CostWrapper&,
      systems::trajectory_optimization::MultipleShooting* prog);

  std::unique_ptr<systems::System<double>> CreateSystem() const;

  std::unique_ptr<systems::trajectory_optimization::MultipleShooting>
  CreateMathematicalProgram() const;

  const solvers::VectorXDecisionVariable GetPositionVariablesFromProgram(
      const systems::trajectory_optimization::MultipleShooting& prog) const;

  const solvers::VectorXDecisionVariable GetPositionVariablesFromProgram(
      const systems::trajectory_optimization::MultipleShooting& prog, int index) const;

  static bool IsValidPlanInterval(const Vector2<double>&);

  std::vector<int> ActiveKnotsForPlanInterval(
      const systems::trajectory_optimization::MultipleShooting& prog,
      const Vector2<double>& plan_interval);

  void UpdatePositionTrajectory(const systems::trajectory_optimization::MultipleShooting& prog);

  std::unique_ptr<RigidBodyTree<double>> tree_;
  int num_time_samples_{0};
  std::unique_ptr<systems::System<double>> system_;

  // See description of the public time(), position(), velocity(),
  // acceleration() and jerk() accessor methods
  // for details about the placeholder variables.
  const solvers::VectorDecisionVariable<1> placeholder_t_var_;
  const solvers::VectorXDecisionVariable placeholder_q_vars_;
  const solvers::VectorXDecisionVariable placeholder_v_vars_;
  const solvers::VectorXDecisionVariable placeholder_a_vars_;
  const solvers::VectorXDecisionVariable placeholder_j_vars_;
  std::map<std::string, solvers::VectorDecisionVariable<6>>
      placeholder_spatial_velocity_vars_;

  std::vector<std::unique_ptr<const symbolic::Expression>> running_cost_expressions_;
  std::vector<std::unique_ptr<const CostWrapper>> running_cost_objects_;
  std::vector<std::unique_ptr<const symbolic::Expression>> final_cost_expressions_;
  std::vector<std::unique_ptr<const FormulaWrapper>> formula_linear_constraints_;
  std::vector<std::unique_ptr<const ConstraintWrapper>> object_constraints_;

  double duration_lower_bound_{0};
  double duration_upper_bound_{std::numeric_limits<double>::infinity()};

  bool has_equal_time_intervals_{false};
  
  double minimum_timestep_;
  double maximum_timestep_;

  PiecewisePolynomial<double> initial_position_trajectory_;
  PiecewisePolynomialTrajectory position_trajectory_;

  solvers::MathematicalProgram solver_options_container_;
};

}  // namespace planner
}  // namespace manipulation
}  // namespace drake
