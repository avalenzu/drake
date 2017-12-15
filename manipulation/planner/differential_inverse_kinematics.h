#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/solvers/mathematical_program.h"

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
    const Vector6<double>& gain_E = Vector6<double>::Constant(1)); 

}  // namespace planner
}  // namespace manipulation
}  // namespace drake
