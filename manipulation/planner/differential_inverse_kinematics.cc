#include "drake/manipulation/planner/differential_inverse_kinematics.h"

namespace drake {
namespace manipulation {
namespace planner {
optional<VectorX<double>> DifferentialInverseKinematics(
    const RigidBodyTree<double>& robot, const KinematicsCache<double>& cache,
    const RigidBodyFrame<double>& frame_E, const Vector6<double>& V_WE,
    double dt, const VectorX<double> q_nominal, const VectorX<double>& v_last,
    const std::pair<VectorX<double>, VectorX<double>>& q_bounds,
    const std::pair<VectorX<double>, VectorX<double>>& v_bounds,
    const VectorX<double>& unconstrained_dof_v_limit,
    const Vector6<double>& gain_E) {
  const int num_positions = robot.get_num_positions();
  DRAKE_DEMAND(q_nominal.size() == num_positions);
  DRAKE_DEMAND(dt > 0);

  const auto identity_num_positions =
      MatrixX<double>::Identity(num_positions, num_positions);
  Eigen::VectorXd ret;

  drake::solvers::MathematicalProgram prog;
  drake::solvers::VectorXDecisionVariable v =
      prog.NewContinuousVariables(robot.get_num_velocities(), "v");
  drake::solvers::VectorXDecisionVariable alpha =
      prog.NewContinuousVariables(1, "alpha");

  // Add ee vel constraint.
  Eigen::Isometry3d X_WE = robot.CalcFramePoseInWorldFrame(cache, frame_E);

  drake::Matrix6<double> R_EW = drake::Matrix6<double>::Zero();
  R_EW.block<3, 3>(0, 0) = X_WE.linear().transpose();
  R_EW.block<3, 3>(3, 3) = R_EW.block<3, 3>(0, 0);

  // Rotate the velocity into E frame.
  Eigen::MatrixXd J_WE_E =
      R_EW *
      robot.CalcFrameSpatialVelocityJacobianInWorldFrame(cache, frame_E);

  for (int i = 0; i < 6; i++) {
    J_WE_E.row(i) = gain_E(i) * J_WE_E.row(i);
  }

  Vector6<double> V_WE_E = R_EW * V_WE;
  V_WE_E = (V_WE_E.array() * gain_E.array()).matrix();

  Vector6<double> V_WE_E_dir = V_WE_E.normalized();
  double V_WE_E_mag = V_WE_E.norm();

  Eigen::MatrixXd A(6, J_WE_E.cols() + 1);
  A.topLeftCorner(6, J_WE_E.cols()) = J_WE_E;
  A.topRightCorner(6, 1) = -V_WE_E_dir;
  prog.AddLinearEqualityConstraint(A, Vector6<double>::Zero(), {v, alpha});
  auto err_cost = prog.AddQuadraticErrorCost(
      drake::Vector1<double>(1), drake::Vector1<double>(V_WE_E_mag), alpha);

  /*
  prog.AddL2NormCost(J_WE_E, V_WE_E, v);
  */

  // Add a small regularization.
  auto posture_cost = prog.AddQuadraticCost(
      1e-3 * identity_num_positions * dt * dt, 1e-3 * (cache.getQ() - q_nominal) * dt,
      1e-3 * (cache.getQ() - q_nominal).squaredNorm(), v);

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(J_WE_E, Eigen::ComputeFullV);

  // Add v constraint.
  prog.AddBoundingBoxConstraint(v_bounds.first, v_bounds.second, v);

  /*
  // Add vd constraint.
  prog.AddLinearConstraint(identity_num_positions,
                           vd_lower_ * dt + v_last,
                           vd_upper_ * dt + v_last,
                           v);
  */

  // Add constrained the unconstrained dof's velocity to be small, which is used
  // to fullfil the regularization cost.
  prog.AddLinearConstraint(svd.matrixV().col(6).transpose(),
                           -unconstrained_dof_v_limit,
                           unconstrained_dof_v_limit, v);

  // Add q upper and lower joint limit.
  prog.AddLinearConstraint(identity_num_positions * dt, q_bounds.first - cache.getQ(),
                           q_bounds.second - cache.getQ(), v);

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
  // prog.AddLinearConstraint(AA.row(2), min_dist, max_dist, v);
  //}

  // Solve
  drake::solvers::SolutionResult result = prog.Solve();
  if (result != drake::solvers::SolutionResult::kSolutionFound) {
    std::cout << "SCS CANT SOLVE: " << result << "\n";
    //*is_stuck = false;
    return nullopt;
  }
  ret = prog.GetSolution(v);

  Eigen::VectorXd cost(1);
  err_cost.constraint()->Eval(prog.GetSolution(alpha), cost);
  // Not tracking the desired vel norm, and computed vel is small.
  //*is_stuck = cost(0) > 5 && prog.GetSolution(alpha)[0] <= 1e-2;

  // std::cout << "err_cost: " << cost(0) << ", " <<
  // prog.GetSolution(alpha).norm() << "\n";

  posture_cost.constraint()->Eval(prog.GetSolution(v), cost);
  // std::cout << "posture_cost: " << cost(0) << "\n";

  return ret;
}
}  // namespace planner
}  // namespace manipulation
}  // namespace drake
