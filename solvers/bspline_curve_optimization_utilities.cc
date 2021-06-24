#include "drake/solvers/bspline_curve_optimization_utilities.h"

#include <fmt/format.h>

#include "drake/solvers/solve.h"

using drake::math::BsplineBasis;
using drake::symbolic::Environment;
using drake::symbolic::Expression;
using drake::symbolic::Formula;
using drake::symbolic::Substitution;
using drake::symbolic::Variable;
using drake::trajectories::BsplineTrajectory;

namespace drake {
namespace solvers {
BsplineTrajectory<Expression> AddCurveThroughRegions(
    const std::vector<Formula>& regions,
    const VectorXDecisionVariable& position_variables,
    const Variable& indicator_variable,
    const BsplineBasis<double>& basis,
    MathematicalProgram* program) {
  const std::vector<Formula>& S = regions;
  const int n = position_variables.size();
  const int n_S = static_cast<int>(S.size());
  const int n_x = basis.num_basis_functions();
  const int spline_order = basis.order();

  // Let 𝑥ₖ ∈ 𝑋 for 𝑘 ∈ {1, ..., 𝑛ₓ} be a collection of points in 𝑋.
  std::vector<MatrixX<Expression>> x;
  for (int i = 0; i < n_x; ++i) {
    x.push_back(program->NewContinuousVariables(n, 1, fmt::format("x{}", i)));
  }

  // Let 𝐾ⱼ for 𝑗 ∈ {1, ..., 𝑛ₖ} be a collection of index sets
  //     𝐾ⱼ = {𝑘ⱼ₁, ..., 𝑘ⱼₗ, ..., 𝑘ⱼₘⱼ}
  // where 𝑘ⱼₗ ∈ {1, ..., 𝑛ₓ}, and 𝑚ⱼ ∈ {1, ..., 𝑛ₓ}, and 𝑘ⱼₗ≠ 𝑘ⱼₚ.
  std::vector<std::vector<int>> K{};
  const int n_K = n_x - spline_order + 1;
  for (int j = 0; j < n_K; ++j) {
    K.emplace_back();
    for (int ell = j; ell < j + spline_order; ++ell) {
      K.back().push_back(ell);
    }
  }

  // We want to enforce that
  //    ∀ 𝑗 ∈ {1, ..., 𝑛ₖ} ∃ 𝑖 ∈ {1, ..., 𝑛ₛ} such that 𝑥ₖ ∈ 𝑆ᵢ, ∀ 𝑘 ∈ 𝐾ⱼ.
  //
  // We can do this by introducing 𝑛ₛ 𝑛ₖ binary variables bᵢⱼ
  MatrixXDecisionVariable b = program->NewBinaryVariables(n_S, n_K, "b");

  // and sum(nₘⱼ ⋅ 𝑛ₛ, 𝑗 = 1, 𝑗 = 𝑛ₖ) continuous variables 𝑥̂ᵢⱼₗ
  std::vector<std::vector<MatrixXDecisionVariable>> x_hat{};
  for (int i = 0; i < n_S; ++i) {
    x_hat.emplace_back();
    for (int j = 0; j < n_K; ++j) {
      x_hat.back().push_back(
          program->NewContinuousVariables(n, static_cast<int>(K.at(j).size()),
                                      fmt::format("x_hat_{}_{}_", i, j)));
    }
  }

  // along with the following constraints
  //     (1) 𝑥̂ᵢⱼₗ ∈ bᵢⱼ𝑆ᵢ ∀ 𝑖 ∈ {1, ..., 𝑛ₛ}, 𝑗 ∈ {1, ..., 𝑛ₖ}, and 𝑙 ∈ {1, ...,
  //     𝑚ⱼ}. (2) sum(𝑥̂ᵢⱼₗ, 𝑖 = 1, 𝑖 = 𝑛ₛ) = 𝑥ₖ,
  //         where 𝑘 = 𝑘ⱼₗ ∀  𝑗 ∈ {1, ..., 𝑛ₖ}, and 𝑙 ∈ {1, ..., 𝑚ⱼ} .
  //     (3) sum(bᵢⱼ, 𝑖 = 1, 𝑖 = 𝑛ₛ) = 1
  for (int j = 0; j < n_K; ++j) {
    const int m_j = static_cast<int>(K.at(j).size());
    for (int ell = 0; ell < m_j; ++ell) {
      VectorX<Expression> x_sum(n);
      x_sum.setZero();
      for (int i = 0; i < n_S; ++i) {
        x_sum += x_hat.at(i).at(j).col(ell);
        Substitution substitution{{indicator_variable, b(i, j)}};
        for (int d = 0; d < n; ++d) {
          substitution.insert(
              {position_variables(d), x_hat.at(i).at(j).col(ell)(d)});
        }
        program->AddLinearConstraint(S.at(i).Substitute(substitution));
      }
      program->AddLinearEqualityConstraint(x_sum == x.at(K.at(j).at(ell)));
    }
    program->AddLinearEqualityConstraint(b.col(j).cast<Expression>().sum() == 1);
  }

  return BsplineTrajectory<Expression>(basis, x);
}

BsplineTrajectory<double> GetSolutionCurve(
    const MathematicalProgramResult& result,
    const BsplineTrajectory<Expression>& symbolic_curve) {
  std::vector<MatrixX<double>> x_sol{};
  for (const auto& x_k : symbolic_curve.control_points()) {
    x_sol.push_back(GetSolutionPoint(result, x_k));
  }
  std::vector<double> knots_double;
  knots_double.reserve(symbolic_curve.basis().knots().size());
  for (const auto& symbolic_knot : symbolic_curve.basis().knots()) {
    knots_double.push_back(ExtractDoubleOrThrow(symbolic_knot));
  }

  return {BsplineBasis<double>(symbolic_curve.basis().order(),knots_double), x_sol};
}

VectorX<Expression> AddPointInRegions(
    const std::vector<Formula>& regions,
    const VectorXDecisionVariable& position_variables,
    const Variable& indicator_variable,
    MathematicalProgram* program) {
  BsplineBasis<double> point_basis{1, 1, math::KnotVectorType::kClampedUniform};
  return solvers::AddCurveThroughRegions(regions, position_variables,
                                         indicator_variable, point_basis,
                                         program)
      .InitialValue();
}

VectorX<double> GetSolutionPoint(
    const MathematicalProgramResult& result,
    const VectorX<symbolic::Expression>& symbolic_point) {
  Environment env{};
  for (const auto variable : symbolic::GetDistinctVariables(symbolic_point)) {
    env.insert(variable, result.GetSolution(variable));
  }
  return symbolic::Evaluate(symbolic_point, env);
}

VectorX<double> ClosestPointInRegions(
    const std::vector<symbolic::Formula>& regions,
    const VectorX<double>& target_point,
    const VectorXDecisionVariable& position_variables,
    const symbolic::Variable& indicator_variable) {
  MathematicalProgram nearest_point_program;
  VectorX<Expression> nearest_point = solvers::AddPointInRegions(
      regions, position_variables, indicator_variable, &nearest_point_program);
  DRAKE_DEMAND(position_variables.size() == nearest_point.size());
  nearest_point_program.AddQuadraticCost(
      (target_point - nearest_point).squaredNorm());
  MathematicalProgramResult nearest_point_result =
      Solve(nearest_point_program);
  return GetSolutionPoint(nearest_point_result, nearest_point);
}

std::pair<VectorX<double>, VectorX<double>> ClosestPointsInSetsOfRegions(
    const std::pair<const std::vector<symbolic::Formula>,
    const std::vector<symbolic::Formula>>& regions,
    const VectorXDecisionVariable& position_variables,
    const symbolic::Variable& indicator_variable) {
  MathematicalProgram program;
  VectorX<Expression> q_A = solvers::AddPointInRegions(
      regions.first, position_variables, indicator_variable, &program);
  VectorX<Expression> q_B = solvers::AddPointInRegions(
      regions.second, position_variables, indicator_variable, &program);
  program.AddQuadraticCost((q_A - q_B).squaredNorm());
  MathematicalProgramResult result = Solve(program, {}, {});
  DRAKE_DEMAND(result.is_success());
  return {GetSolutionPoint(result, q_A), GetSolutionPoint(result, q_B)};
}
}  // namespace solvers
}  // namespace drake
