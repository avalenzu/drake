#include <vector>

#include "drake/common/symbolic.h"
#include "drake/math/bspline_curve.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
math::BsplineCurve<symbolic::Expression> AddCurveThroughRegions(
    const std::vector<symbolic::Formula>& regions,
    const VectorXDecisionVariable& position_variables,
    const symbolic::Variable& indicator_variable,
    const math::BsplineBasis<double>& basis, MathematicalProgram* program);

math::BsplineCurve<double> GetSolutionCurve(
    const MathematicalProgramResult& result,
    const math::BsplineCurve<symbolic::Expression>& symbolic_curve);

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
