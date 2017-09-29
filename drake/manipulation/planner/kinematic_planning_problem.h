#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/solvers/binding.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/cost.h"
#include "drake/solvers/decision_variable.h"

namespace drake {
namespace manipulation {
namespace planner {

class KinematicPlanningProblem {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(KinematicPlanningProblem)

  KinematicPlanningProblem(const RigidBodyTree<double>& tree);

  void AddFixedBoxToWorld(Vector3<double> size, Isometry3<double> X_WB);

  void AddDurationBounds(double lower_bound, double upper_bound);

  void AddCost(const solvers::Binding<solvers::Cost>& binding,
               double plan_time);

  void AddRunningCost(const solvers::Binding<solvers::Cost>& binding,
                      const Vector2<double>& plan_interval = {0, 1});

  void AddConstraint(const solvers::Binding<solvers::Constraint>& binding,
                     const Vector2<double>& plan_interval = {0, 1});

  void AddConstraint(const solvers::Binding<solvers::Constraint>& binding,
                     double plan_time);

  void AddCollisionAvoidanceConstraint(double threshold,
                                       const Vector2<double>& plan_interval = {
                                           0, 1});

  const solvers::VectorDecisionVariable<1>& time() const;

  const solvers::VectorXDecisionVariable& position() const;

  const solvers::VectorXDecisionVariable& velocity() const;

  const solvers::VectorXDecisionVariable& acceleration() const;

  const solvers::VectorXDecisionVariable& jerk() const;

  int num_positions() const { return num_positions_; };

  int num_velocities() const { return num_velocities_; };

 private:
  struct CostWrapper {
    std::shared_ptr<solvers::Cost> cost;
    solvers::VectorXDecisionVariable vars;
    const Vector2<double> plan_interval;
  };

  struct ConstraintWrapper {
    std::shared_ptr<solvers::Constraint> constraint;
    solvers::VectorXDecisionVariable vars;
    const Vector2<double> plan_interval;
  };

  struct CollisionAvoidanceWrapper {
    const double threshold;
    solvers::VectorXDecisionVariable vars;
    const Vector2<double> plan_interval;
  };

  const int num_positions_;
  const int num_velocities_;

  // See description of the public time(), position(), velocity(),
  // acceleration() and jerk() accessor methods
  // for details about the placeholder variables.
  const solvers::VectorDecisionVariable<1> placeholder_time_var_;
  const solvers::VectorXDecisionVariable placeholder_position_vars_;
  const solvers::VectorXDecisionVariable placeholder_velocity_vars_;
  const solvers::VectorXDecisionVariable placeholder_acceleration_vars_;
  const solvers::VectorXDecisionVariable placeholder_jerk_vars_;
  std::map<std::string, solvers::VectorDecisionVariable<6>>
      placeholder_body_spatial_velocity_vars_;
  std::map<std::string, solvers::VectorDecisionVariable<7>>
      placeholder_body_pose_vars_;

  std::vector<std::unique_ptr<const CostWrapper>> costs_;
  std::vector<std::unique_ptr<const ConstraintWrapper>> constraints_;
  std::vector<std::unique_ptr<const CollisionAvoidanceWrapper>>
      collision_constraints_;

  std::vector<
      std::pair<std::unique_ptr<const DrakeShapes::Geometry>, Isometry3<double>>>
      world_geometry_;

  double duration_lower_bound_{0};
  double duration_upper_bound_{std::numeric_limits<double>::infinity()};
};

}  // namespace planner
}  // namespace manipulation
}  // namespace drake
