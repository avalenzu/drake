#include <vector>

#include "drake/common/symbolic.h"
#include "drake/common/trajectories/bspline_trajectory.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mathematical_program_result.h"

namespace drake {
namespace solvers {
trajectories::BsplineTrajectory<symbolic::Expression> AddCurveThroughRegions(
    const std::vector<symbolic::Formula>& regions,
    const VectorXDecisionVariable& position_variables,
    const symbolic::Variable& indicator_variable,
    const math::BsplineBasis<double>& basis, MathematicalProgram* program);

trajectories::BsplineTrajectory<double> GetSolutionCurve(
    const MathematicalProgramResult& result,
    const trajectories::BsplineTrajectory<symbolic::Expression>& symbolic_curve);

VectorX<symbolic::Expression> AddPointInRegions(
    const std::vector<symbolic::Formula>& regions,
    const VectorXDecisionVariable& position_variables,
    const symbolic::Variable& indicator_variable,
    MathematicalProgram* program);

VectorX<double> GetSolutionPoint(
    const MathematicalProgramResult& result,
    const VectorX<symbolic::Expression>& symbolic_curve);

VectorX<double> ClosestPointInRegions(
    const std::vector<symbolic::Formula>& regions,
    const VectorX<double>& target_point,
    const VectorXDecisionVariable& position_variables,
    const symbolic::Variable& indicator_variable);

std::pair<VectorX<double>, VectorX<double>> ClosestPointsInSetsOfRegions(
    const std::pair<const std::vector<symbolic::Formula>,
                    const std::vector<symbolic::Formula>>& regions,
    const VectorXDecisionVariable& position_variables,
    const symbolic::Variable& indicator_variable);
}  // namespace solvers
}  // namespace drake
