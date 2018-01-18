#include "drake/manipulation/planner/differential_inverse_kinematics.h"

namespace drake {
namespace manipulation {
namespace planner {

std::pair<optional<VectorX<double>>, DifferentialInverseKinematicsStatus>
DoDifferentialInverseKinematics(
    const VectorX<double> q_current, const VectorX<double>& v_current,
    const Vector6<double>& V_WE_E, const MatrixX<double>& J_WE_E,
    const VectorX<double> q_nominal,
    const std::pair<VectorX<double>, VectorX<double>>& q_bounds,
    const optional<std::pair<VectorX<double>, VectorX<double>>>& v_bounds,
    const optional<std::pair<VectorX<double>, VectorX<double>>>& vd_bounds,
    double dt, double unconstrained_dof_v_limit,
    const Vector6<double>& gain_E) {
  const int num_velocities = v_current.size();
  const int num_positions = q_current.size();
  DRAKE_ASSERT(num_positions == num_velocities);
  DRAKE_ASSERT(q_nominal.size() == num_positions);
  DRAKE_ASSERT(dt > 0);
  DRAKE_ASSERT(J_WE_E.rows() == V_WE_E.size());
  DRAKE_ASSERT(J_WE_E.cols() == V_WE_E.size());

  const auto identity_num_positions =
      MatrixX<double>::Identity(num_positions, num_positions);
  Eigen::VectorXd ret;

  drake::solvers::MathematicalProgram prog;
  drake::solvers::VectorXDecisionVariable v_next =
      prog.NewContinuousVariables(num_velocities, "v_next");
  drake::solvers::VectorXDecisionVariable alpha =
      prog.NewContinuousVariables(1, "alpha");

  // Add ee vel constraint.
  const VectorX<double> V_WE_E_scaled =
      (V_WE_E.array() * gain_E.array()).matrix();

  Vector6<double> V_WE_E_dir = V_WE_E_scaled.normalized();
  double V_WE_E_mag = V_WE_E_scaled.norm();

  Eigen::MatrixXd A(6, J_WE_E.cols() + 1);
  A.topLeftCorner(6, J_WE_E.cols()) = J_WE_E;
  A.topRightCorner(6, 1) = -V_WE_E_dir;
  prog.AddLinearEqualityConstraint(A, Vector6<double>::Zero(), {v_next, alpha});
  auto err_cost = prog.AddQuadraticErrorCost(
      drake::Vector1<double>(1), drake::Vector1<double>(V_WE_E_mag), alpha);

  const VectorX<double> q_error{q_current - q_nominal};

  // Add a small regularization.
  auto posture_cost = prog.AddQuadraticCost(
      1e-3 * identity_num_positions * dt * dt, 1e-3 * q_error * dt,
      1e-3 * q_error.squaredNorm(), v_next);

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(J_WE_E, Eigen::ComputeFullV);

  // Add v_next constraint.
  //if (v_bounds) {
    //prog.AddBoundingBoxConstraint(v_bounds->first, v_bounds->second, v_next);
  //}

  // Add vd constraint.
  //if (vd_bounds) {
    //prog.AddLinearConstraint(identity_num_positions,
                             //vd_bounds->first * dt + v_current,
                             //vd_bounds->second * dt + v_current, v_next);
  //}

  // Add constrained the unconstrained dof's velocity to be small, which is used
  // to fullfil the regularization cost.
  //prog.AddLinearConstraint(svd.matrixV().col(6).transpose(),
                           //-unconstrained_dof_v_limit,
                           //unconstrained_dof_v_limit, v_next);

  // Add q upper and lower joint limit.
  //prog.AddLinearConstraint(identity_num_positions * dt,
                           //q_bounds.first - q_current,
                           //q_bounds.second - q_current, v_next);

  // Do the collision constraints.
  // for (const std::pair<Capsule, Capsule>& col_pair : collisions) {
  // Eigen::Vector3d p0, p1;
  // const Capsule& c0 = col_pair.first;
  // const Capsule& c1 = col_pair.second;

  //// Computes the closest points in world frame.
  // c0.GetClosestPointsOnAxis(c1, &p0, &p1);
  //// Transform p0 and p1 into their respective body frames.
  // Eigen::Isometry3d X0(Eigen::Translation3d(
  // robot->CalcBodyPoseInWorldFrame(cache, c0.get_body()).inverse() * p0));
  // Eigen::Isometry3d X1(Eigen::Translation3d(
  // robot->CalcBodyPoseInWorldFrame(cache, c1.get_body()).inverse() * p1));

  // auto J0 = robot->CalcFrameSpatialVelocityJacobianInWorldFrame(
  // cache, c0.get_body(), X0);
  // auto J1 = robot->CalcFrameSpatialVelocityJacobianInWorldFrame(
  // cache, c1.get_body(), X1);

  // Eigen::Matrix3d Rinv = c0.ComputeEscapeFrame(p0, p1).transpose();

  // auto AA = Rinv * (J1.bottomRows(3) - J0.bottomRows(3)) * dt;
  // Eigen::Vector3d bb = Rinv * (p1 - p0);
  // drake::Vector1<double> min_dist(
  //-(bb[2] - c0.get_radius() - c1.get_radius()));
  // drake::Vector1<double> max_dist(1e6);
  // prog.AddLinearConstraint(AA.row(2), min_dist, max_dist, v_next);
  //}

  // Solve
  drake::solvers::SolutionResult result = prog.Solve();
  if (result != drake::solvers::SolutionResult::kSolutionFound) {
    std::cout << "SCS CANT SOLVE: " << result << "\n";
    return {nullopt, DifferentialInverseKinematicsStatus::kNoSolutionFound};
  }
  ret = prog.GetSolution(v_next);

  Eigen::VectorXd cost(1);
  err_cost.constraint()->Eval(prog.GetSolution(alpha), cost);

  // Not tracking the desired vel norm, and computed vel is small.
  if (cost(0) > 5 && prog.GetSolution(alpha)[0] <= 1e-2) {
    drake::log()->info("v_next = {}", prog.GetSolution(v_next).transpose());
    drake::log()->info("alpha = {}", prog.GetSolution(alpha).transpose());
    return {nullopt, DifferentialInverseKinematicsStatus::kStuck};
  }

  // std::cout << "err_cost: " << cost(0) << ", " <<
  // prog.GetSolution(alpha).norm() << "\n";

  posture_cost.constraint()->Eval(prog.GetSolution(v_next), cost);
  // std::cout << "posture_cost: " << cost(0) << "\n";

  return {ret, DifferentialInverseKinematicsStatus::kSolutionFound};
}

std::pair<optional<VectorX<double>>, DifferentialInverseKinematicsStatus>
DoDifferentialInverseKinematics(
    const RigidBodyTree<double>& robot, const KinematicsCache<double>& cache,
    const RigidBodyFrame<double>& frame_E, const Vector6<double>& V_WE,
    double dt, const VectorX<double> q_nominal, const VectorX<double>& v_last,
    const std::pair<VectorX<double>, VectorX<double>>& q_bounds,
    const optional<std::pair<VectorX<double>, VectorX<double>>>& v_bounds,
    const optional<std::pair<VectorX<double>, VectorX<double>>>& vd_bounds,
    double unconstrained_dof_v_limit, const Vector6<double>& gain_E) {
  Eigen::Isometry3d X_WE = robot.CalcFramePoseInWorldFrame(cache, frame_E);

  drake::Matrix6<double> R_EW = drake::Matrix6<double>::Zero();
  R_EW.block<3, 3>(0, 0) = X_WE.linear().transpose();
  R_EW.block<3, 3>(3, 3) = R_EW.block<3, 3>(0, 0);

  // Rotate the velocity into E frame.
  Eigen::MatrixXd J_WE_E =
      R_EW * robot.CalcFrameSpatialVelocityJacobianInWorldFrame(cache, frame_E);

  for (int i = 0; i < 6; i++) {
    J_WE_E.row(i) = gain_E(i) * J_WE_E.row(i);
  }

  Vector6<double> V_WE_E = R_EW * V_WE;
  return DoDifferentialInverseKinematics(
      cache.getQ(), v_last, V_WE_E, J_WE_E, q_nominal, q_bounds,
      v_bounds, vd_bounds, dt, unconstrained_dof_v_limit, gain_E);
}

DifferentialInverseKinematics::DifferentialInverseKinematics(
    std::unique_ptr<RigidBodyTree<double>> robot,
    const std::string& end_effector_frame_name)
    : robot_(robot),
      frame_E_(robot_->findFrame(end_effector_frame_name)),
      q_nominal_(robot_->getZeroConfiguration()),
      q_bounds_(robot_->joint_limit_min, robot_->joint_limit_max),
      q_current_(robot_->getZeroConfiguration()),
      v_current_(VectorX<double>::Zero(robot_->get_num_velocities())),
      V_WE_desired_(Vector6<double>::Zero()) {}

void DifferentialInverseKinematics::SetEndEffectorVelocityGain(
    const Vector6<double>& gain_E) {
  DRAKE_THROW_UNLESS((gain_E.array() >= 0).all() &&
                     (gain_E.array() <= 1).all());
  gain_E_ = gain_E;
}

void DifferentialInverseKinematics::SetJointPositionLimits(
    const std::pair<VectorX<double>, VectorX<double>>& q_bounds) {
  DRAKE_DEMAND(q_bounds.first.size() == robot_->get_num_positions());
  DRAKE_DEMAND(q_bounds.second.size() == robot_->get_num_positions());
  DRAKE_DEMAND((q_bounds.second.array() >= q_bounds.first.array()).all());
  q_bounds_ = q_bounds;
}

void DifferentialInverseKinematics::SetJointVelocityLimits(
    const std::pair<VectorX<double>, VectorX<double>>& v_bounds) {
  DRAKE_DEMAND(v_bounds.first.size() == robot_->get_num_velocities());
  DRAKE_DEMAND(v_bounds.second.size() == robot_->get_num_velocities());
  DRAKE_DEMAND((v_bounds.second.array() >= v_bounds.first.array()).all());
  v_bounds_ = v_bounds;
}

void DifferentialInverseKinematics::SetJointAccelerationLimits(
    const std::pair<VectorX<double>, VectorX<double>>& vd_bounds) {
  DRAKE_DEMAND(vd_bounds.first.size() == robot_->get_num_velocities());
  DRAKE_DEMAND(vd_bounds.second.size() == robot_->get_num_velocities());
  DRAKE_DEMAND((vd_bounds.second.array() >= vd_bounds.first.array()).all());
  vd_bounds_ = vd_bounds;
}

std::pair<optional<VectorX<double>>, DifferentialInverseKinematicsStatus>
DifferentialInverseKinematics::Solve() const {
  KinematicsCache<double> cache = robot_->doKinematics(q_current_);
  return DoDifferentialInverseKinematics(
      *robot_, cache, *frame_E_, V_WE_desired_, dt_, q_nominal_, v_current_,
      q_bounds_, v_bounds_, vd_bounds_, unconstrained_dof_v_limit_, gain_E_);
}

std::pair<optional<VectorX<double>>, DifferentialInverseKinematicsStatus>
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
