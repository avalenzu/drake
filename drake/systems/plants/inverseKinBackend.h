#pragma once

#include <string>

#include <Eigen/StdVector>
#include <Eigen/Dense>

#include "drake/solvers/solution_result.h"

class RigidBodyTree;
class RigidBodyConstraint;
class IKoptions;

namespace drake {
namespace solvers {
class OptimizationProblem;
}
}

namespace Drake {
namespace systems {
namespace plants {

/// This function is primarily documented through RigidBodyIK.h.  All
/// parameters are passthroughs from there.
template <typename DerivedA, typename DerivedB, typename DerivedC,
          typename DerivedD, typename DerivedE>
void inverseKinBackend(RigidBodyTree *model, const int mode, const int nT,
                       const double *t,
                       const Eigen::MatrixBase<DerivedA>& q_seed,
                       const Eigen::MatrixBase<DerivedB>& q_nom,
                       int num_constraints,
                       RigidBodyConstraint **const constraint_array,
                       const IKoptions& ikoptions,
                       Eigen::MatrixBase<DerivedC>* q_sol,
                       Eigen::MatrixBase<DerivedD>* qdot_sol,
                       Eigen::MatrixBase<DerivedE>* qddot_sol, int *INFO,
                       std::vector<std::string>* infeasible_constraint);

template <typename DerivedA, typename DerivedB, typename DerivedC,
          typename DerivedD, typename DerivedE>
void inverseKinSnoptBackend(RigidBodyTree *model, const int mode, const int nT,
                            const double *t,
                            const Eigen::MatrixBase<DerivedA>& q_seed,
                            const Eigen::MatrixBase<DerivedB>& q_nom,
                            int num_constraints,
                            RigidBodyConstraint** const constraint_array,
                            const IKoptions& ikoptions,
                            Eigen::MatrixBase<DerivedC>* q_sol,
                            Eigen::MatrixBase<DerivedD>* qdot_sol,
                            Eigen::MatrixBase<DerivedE>* qddot_sol, int *INFO,
                            std::vector<std::string>* infeasible_constraint);

/// Translate a solver result into something expected for the INFO
/// output parameter.
int GetIKSolverInfo(const drake::solvers::OptimizationProblem& prog,
                    drake::solvers::SolutionResult result);

/// Set solver options based on IK options.
void SetIKSolverOptions(const IKoptions& ikoptions,
                        drake::solvers::OptimizationProblem* prog);
}
}
}
