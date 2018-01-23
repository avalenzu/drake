#include "drake/manipulation/planner/differential_inverse_kinematics.h"

namespace drake {
namespace manipulation {
namespace planner {

DifferentialInverseKinematicsResult DoDifferentialInverseKinematics(
    const VectorX<double> q_current, const VectorX<double>& v_current,
    const Vector6<double>& V, const MatrixX<double>& J,
    const DifferentialInverseKinematicsParameters& parameters) {
  const int num_positions = parameters.num_positions();
  const int num_velocities = parameters.num_velocities();
  const int num_cart_constraints = V.size();
  DRAKE_ASSERT(q_current.size() == num_positions);
  DRAKE_ASSERT(v_current.size() == num_velocities);
  DRAKE_ASSERT(J.rows() == num_cart_constraints);
  DRAKE_ASSERT(J.cols() == parameters.num_velocities());

  const auto identity_num_positions =
      MatrixX<double>::Identity(num_positions, num_positions);

  drake::solvers::MathematicalProgram prog;
  drake::solvers::VectorXDecisionVariable v_next =
      prog.NewContinuousVariables(num_velocities, "v_next");
  drake::solvers::VectorXDecisionVariable alpha =
      prog.NewContinuousVariables(1, "alpha");

  // Add ee vel constraint.
  const solvers::QuadraticCost* cart_cost = nullptr;

  if (num_cart_constraints > 0) {
    Vector6<double> V_dir = V.normalized();
    double V_mag = V.norm();

    // Constrain the end effector motion to be in the direction of V_dir,
    // and penalize magnitude difference from V_mag.
    Eigen::MatrixXd A(6, J.cols() + 1);
    A.topLeftCorner(6, J.cols()) = J;
    A.topRightCorner(6, 1) = -V_dir;
    drake::log()->info("A =\n{}", A);
    prog.AddLinearEqualityConstraint(A, Vector6<double>::Zero(),
                                     {v_next, alpha});
    cart_cost = prog.AddQuadraticErrorCost(drake::Vector1<double>(100),
                                           drake::Vector1<double>(V_mag), alpha)
                    .constraint()
                    .get();

    Eigen::JacobiSVD<MatrixX<double>> svd(J, Eigen::ComputeFullV);

    // Add constrained the unconstrained dof's velocity to be small, which is
    // used to fullfil the regularization cost.
    for (int i = num_cart_constraints; i < num_velocities; i++) {
      prog.AddLinearConstraint(
          svd.matrixV().col(i).transpose(),
          -parameters.unconstrained_degrees_of_freedom_velocity_limit(),
          parameters.unconstrained_degrees_of_freedom_velocity_limit(), v_next);
    }
  }

  const VectorX<double> q_error{q_current -
                                parameters.nominal_joint_position()};

  // If redundant, add a small regularization term to q_nominal.
  const double dt{parameters.timestep()};
  if (num_cart_constraints < num_velocities) {
    prog.AddQuadraticCost(identity_num_positions * dt * dt, q_error / dt,
                          v_next);
  }

  // Add q upper and lower joint limit.
  if (parameters.joint_position_limits()) {
    prog.AddBoundingBoxConstraint(
        (parameters.joint_position_limits()->first - q_current) / dt,
        (parameters.joint_position_limits()->second - q_current) / dt, v_next);
  }

  // Add v_next constraint.
  if (parameters.joint_velocity_limits()) {
    prog.AddBoundingBoxConstraint(parameters.joint_velocity_limits()->first,
                                  parameters.joint_velocity_limits()->second,
                                  v_next);
  }

  // Add vd constraint.
  if (parameters.joint_acceleration_limits()) {
    prog.AddLinearConstraint(
        identity_num_positions,
        parameters.joint_acceleration_limits()->first * dt + v_current,
        parameters.joint_acceleration_limits()->second * dt + v_current,
        v_next);
  }

  // Solve
  drake::solvers::SolutionResult result = prog.Solve();

  if (result != drake::solvers::SolutionResult::kSolutionFound) {
    return {nullopt, DifferentialInverseKinematicsStatus::kNoSolutionFound};
  }

  Eigen::VectorXd cost(1);
  cart_cost->Eval(prog.GetSolution(alpha), cost);

  // Not tracking the desired vel norm, and computed vel is small.
  if (num_cart_constraints && cost(0) > 5 &&
      prog.GetSolution(alpha)[0] <= 1e-2) {
    drake::log()->info("v_next = {}", prog.GetSolution(v_next).transpose());
    drake::log()->info("alpha = {}", prog.GetSolution(alpha).transpose());
    return {nullopt, DifferentialInverseKinematicsStatus::kStuck};
  }
  drake::log()->info("alpha = {}", prog.GetSolution(alpha).transpose());

  return {prog.GetSolution(v_next),
          DifferentialInverseKinematicsStatus::kSolutionFound};
}

DifferentialInverseKinematicsResult DoDifferentialInverseKinematics(
    const RigidBodyTree<double>& robot, const VectorX<double>& q_current,
    const VectorX<double>& v_current, const RigidBodyFrame<double>& frame_E,
    const Vector6<double>& V_WE,
    const DifferentialInverseKinematicsParameters& parameters) {
  KinematicsCache<double> cache = robot.doKinematics(q_current);
  return DoDifferentialInverseKinematics(robot, cache, v_current, frame_E, V_WE,
                                         parameters);
}

DifferentialInverseKinematicsResult DoDifferentialInverseKinematics(
    const RigidBodyTree<double>& robot, const KinematicsCache<double>& cache,
    const VectorX<double>& v_last, const RigidBodyFrame<double>& frame_E,
    const Vector6<double>& V_WE,
    const DifferentialInverseKinematicsParameters& parameters) {
  Eigen::Isometry3d X_WE = robot.CalcFramePoseInWorldFrame(cache, frame_E);

  drake::Matrix6<double> R_EW = drake::Matrix6<double>::Zero();
  R_EW.block<3, 3>(0, 0) = X_WE.linear().transpose();
  R_EW.block<3, 3>(3, 3) = R_EW.block<3, 3>(0, 0);

  // Rotate the velocity into E frame.
  Eigen::MatrixXd J_WE_E =
      R_EW * robot.CalcFrameSpatialVelocityJacobianInWorldFrame(cache, frame_E);

  Vector6<double> V_WE_E = R_EW * V_WE;

  Vector6<double> V_WE_E_scaled;
  MatrixX<double> J_WE_E_scaled{6, J_WE_E.cols()};
  int num_cart_constraints = 0;
  for (int i = 0; i < 6; i++) {
    const double gain{parameters.end_effector_velocity_gain()(i)};
    if (gain > 0) {
      J_WE_E_scaled.row(num_cart_constraints) = gain * J_WE_E.row(i);
      V_WE_E_scaled(num_cart_constraints) = gain * V_WE_E(i);
      num_cart_constraints++;
    }
  }

  MatrixX<double> J = J_WE_E_scaled.topRows(num_cart_constraints);
  VectorX<double> V = V_WE_E_scaled.head(num_cart_constraints);

  return DoDifferentialInverseKinematics(cache.getQ(), v_last, V, J,
                                         parameters);
}

DifferentialInverseKinematicsParameters::
    DifferentialInverseKinematicsParameters(int num_positions,
                                            int num_velocities)
    : num_positions_(num_positions),
      num_velocities_(num_velocities),
      nominal_joint_position_(VectorX<double>::Zero(num_positions)) {}

void DifferentialInverseKinematicsParameters::SetJointPositionLimits(
    const std::pair<VectorX<double>, VectorX<double>>& q_bounds) {
  DRAKE_DEMAND(q_bounds.first.size() == num_positions());
  DRAKE_DEMAND(q_bounds.second.size() == num_positions());
  DRAKE_DEMAND((q_bounds.second.array() >= q_bounds.first.array()).all());
  q_bounds_ = q_bounds;
}

void DifferentialInverseKinematicsParameters::SetJointVelocityLimits(
    const std::pair<VectorX<double>, VectorX<double>>& v_bounds) {
  DRAKE_DEMAND(v_bounds.first.size() == num_velocities());
  DRAKE_DEMAND(v_bounds.second.size() == num_velocities());
  DRAKE_DEMAND((v_bounds.second.array() >= v_bounds.first.array()).all());
  v_bounds_ = v_bounds;
}

void DifferentialInverseKinematicsParameters::SetJointAccelerationLimits(
    const std::pair<VectorX<double>, VectorX<double>>& vd_bounds) {
  DRAKE_DEMAND(vd_bounds.first.size() == num_velocities());
  DRAKE_DEMAND(vd_bounds.second.size() == num_velocities());
  DRAKE_DEMAND((vd_bounds.second.array() >= vd_bounds.first.array()).all());
  vd_bounds_ = vd_bounds;
}

void DifferentialInverseKinematicsParameters::SetEndEffectorVelocityGain(
    const Vector6<double>& gain_E) {
  DRAKE_THROW_UNLESS((gain_E.array() >= 0).all() &&
                     (gain_E.array() <= 1).all());
  gain_E_ = gain_E;
}

DifferentialInverseKinematics::DifferentialInverseKinematics(
    std::unique_ptr<RigidBodyTree<double>> robot,
    const std::string& end_effector_frame_name)
    : robot_(robot),
      frame_E_(robot_->findFrame(end_effector_frame_name)),
      q_current_(robot_->getZeroConfiguration()),
      v_current_(VectorX<double>::Zero(robot_->get_num_velocities())),
      V_WE_desired_(Vector6<double>::Zero()),
      parameters_(robot->get_num_positions(), robot->get_num_velocities()) {
  parameters_.SetJointPositionLimits(
      {robot_->joint_limit_min, robot_->joint_limit_max});
}

void DifferentialInverseKinematics::SetEndEffectorVelocityGain(
    const Vector6<double>& gain_E) {
  parameters_.SetEndEffectorVelocityGain(gain_E);
}

void DifferentialInverseKinematics::SetJointPositionLimits(
    const std::pair<VectorX<double>, VectorX<double>>& q_bounds) {
  parameters_.SetJointPositionLimits(q_bounds);
}

void DifferentialInverseKinematics::SetJointVelocityLimits(
    const std::pair<VectorX<double>, VectorX<double>>& v_bounds) {
  parameters_.SetJointVelocityLimits(v_bounds);
}

void DifferentialInverseKinematics::SetJointAccelerationLimits(
    const std::pair<VectorX<double>, VectorX<double>>& vd_bounds) {
  parameters_.SetJointAccelerationLimits(vd_bounds);
}

DifferentialInverseKinematicsResult DifferentialInverseKinematics::Solve()
    const {
  return DoDifferentialInverseKinematics(*robot_, q_current_, v_current_,
                                         *frame_E_, V_WE_desired_, parameters_);
}

DifferentialInverseKinematicsResult
DifferentialInverseKinematics::ComputeJointVelocities(
    const VectorX<double>& q, const VectorX<double>& v_last,
    const Vector6<double>& V_WE, double dt) {
  set_current_joint_position(q);
  set_current_joint_velocity(v_last);
  set_desired_end_effector_velocity(V_WE);
  set_timestep(dt);
  return Solve();
}
}  // namespace planner
}  // namespace manipulation
}  // namespace drake
