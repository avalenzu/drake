#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_polynomial_trajectory.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/systems/primitives/linear_system.h"

namespace drake {
namespace manipulation {
namespace planner {

class KinematicTrajectoryOptimization {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(KinematicTrajectoryOptimization)

  KinematicTrajectoryOptimization(std::unique_ptr<RigidBodyTree<double>> tree,
                                  int num_time_samples, double minimum_timestep,
                                  double maximum_timestep);

  void AddFixedBoxToWorld(Vector3<double> size, Isometry3<double> X_WB);

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

  /// Returns a placeholder decision variable (not actually declared as a
  /// decision variable in the MathematicalProgram) associated with the
  /// spatial velocity vector of @p body_name.  This variable will be
  /// substituted for real decision variables at particular times in methods
  /// like AddRunningCost.  Passing this variable directly into
  /// objectives/constraints for the parent classes will result in an error.
  const solvers::VectorXDecisionVariable spatial_velocity_of_body(
      const std::string& body_name) const {
    return placeholder_spatial_velocity_vars_.at(body_name);
  }

  int num_time_samples() const { return num_time_samples_; };

  void set_num_time_samples(int num_time_samples) {
    DRAKE_DEMAND(num_time_samples >= 2);
    num_time_samples_ = num_time_samples;
  };

  double spatial_velocity_tolerance() const {
    return spatial_velocity_tolerance_;
  };

  void set_spatial_velocity_tolerance(double spatial_velocity_tolerance) {
    spatial_velocity_tolerance_ = spatial_velocity_tolerance;
  }

  const RigidBodyTree<double>& tree() const { return *tree_; };

  int num_positions() const { return tree_->get_num_positions(); };

  int num_velocities() const { return tree_->get_num_velocities(); };

  int num_body_spatial_velocity_variables() const {
    return 6 * placeholder_spatial_velocity_vars_.size();
  };

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
    AddRunningCost(g(0, 0));
  }

  void TrackSpatialVelocityOfBody(const std::string& body_name,
                                  double tolerance = 0);

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

  bool IsPositionTrajectoryCollisionFree(double threshold) const;

  solvers::SolutionResult Solve();

  PiecewisePolynomialTrajectory GetPositionTrajectory() const;

  template <typename T>
  void SetSolverOption(const solvers::SolverId& solver_id,
                       const std::string& solver_option, T option_value);

  void AddLinearConstraint(const symbolic::Formula& f);

  void AddLinearConstraint(const symbolic::Formula& f, double time);

  void AddLinearConstraint(const symbolic::Formula& f,
                           Vector2<double> plan_interval);

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
      const solvers::VectorXDecisionVariable& position_variables,
      int index = -1) const;

  solvers::VectorXDecisionVariable SubstitutePlaceholderVariables(
      const solvers::VectorXDecisionVariable& vars,
      const solvers::VectorXDecisionVariable& position_variables,
      int index = -1);

  symbolic::Formula SubstitutePlaceholderVariables(
      const symbolic::Formula& f,
      const solvers::MathematicalProgram& prog,
      int index = -1);

  symbolic::Expression SubstitutePlaceholderVariables(
      const symbolic::Expression& g,
      const solvers::MathematicalProgram& prog,
      int index = -1);

  void AddConstraintToProgram(
      const ConstraintWrapper& constraint,
      solvers::MathematicalProgram* prog);

  void AddLinearConstraintToProgram(
      const FormulaWrapper& constraint,
      solvers::MathematicalProgram* prog);

  void AddRunningCostToProgram(
      const symbolic::Expression& cost,
      solvers::MathematicalProgram* prog);

  void AddFinalCostToProgram(
      const symbolic::Expression& cost,
      solvers::MathematicalProgram* prog);

  void AddCostToProgram(
      const CostWrapper&,
      solvers::MathematicalProgram* prog);

  void SetInitialTrajectoryOnProgram(
      solvers::MathematicalProgram* prog);

  const VectorX<symbolic::Expression> GetSplineVariableExpression(
      double time, int derivative_order) const;

  const symbolic::Expression ConstructPositionExpression(
      const solvers::MathematicalProgram& prog,
      double time) const;

  const symbolic::Expression ConstructVelocityExpression(
      const solvers::MathematicalProgram& prog,
      double time) const;

  const symbolic::Expression ConstructAccelerationExpression(
      const solvers::MathematicalProgram& prog,
      double time) const;

  const symbolic::Expression ConstructJerkExpression(
      const solvers::MathematicalProgram& prog,
      double time) const;

  bool AreVariablesPresentInProgram(symbolic::Variables vars) const;

  static bool IsValidPlanInterval(const Vector2<double>&);

  std::vector<int> ActiveKnotsForPlanInterval(
      const solvers::MathematicalProgram& prog,
      const Vector2<double>& plan_interval);

  void UpdatePositionTrajectory(
      const solvers::MathematicalProgram& prog);

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

  std::vector<std::unique_ptr<const symbolic::Expression>>
      running_cost_expressions_;
  std::vector<std::unique_ptr<const CostWrapper>> running_cost_objects_;
  std::vector<std::unique_ptr<const symbolic::Expression>>
      final_cost_expressions_;
  std::vector<std::unique_ptr<const FormulaWrapper>>
      formula_linear_constraints_;
  std::vector<std::unique_ptr<const ConstraintWrapper>> object_constraints_;

  double duration_lower_bound_{0};
  double duration_upper_bound_{std::numeric_limits<double>::infinity()};

  bool has_equal_time_intervals_{false};
  int spline_order_{4};
  int num_control_points_{4};

  PiecewisePolynomial<double> initial_position_trajectory_;
  PiecewisePolynomialTrajectory position_trajectory_;

  solvers::MathematicalProgram solver_options_container_;
  std::vector<solvers::SolverId> solver_options_solver_ids_;

  double spatial_velocity_tolerance_{1e-6};
};

}  // namespace planner
}  // namespace manipulation
}  // namespace drake
