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
}  // namespace solvers
}  // namespace drake
