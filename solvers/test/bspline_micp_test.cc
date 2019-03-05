#include <vector>

#include <gflags/gflags.h>
#include <gtest/gtest.h>
#include <fmt/format.h>

#include "drake/common/proto/call_python.h"
#include "drake/common/symbolic.h"
#include "drake/common/text_logging.h"
#include "drake/math/bspline_curve.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solve.h"

using drake::common::CallPython;
using drake::math::BsplineBasis;
using drake::math::BsplineCurve;
using drake::symbolic::Environment;
using drake::symbolic::Expression;
using drake::symbolic::Formula;
using drake::symbolic::Substitution;
using drake::symbolic::Variable;
using drake::symbolic::Variables;

DEFINE_bool(clamped, false, "If true, use clamped knot vector.");
DEFINE_int32(num_plotting_points, 100, "Number of points for visualizing solution");
DEFINE_int32(order, 4, "Spline order.");
DEFINE_int32(num_control_points, 16, "Number of spline control points.");
DEFINE_double(S0_x_min, 0, "Minimum x value of S0.");
DEFINE_double(S0_x_max, 1, "Maximum x value of S0.");
DEFINE_double(S0_y_min, -1, "Minimum y value of S0.");
DEFINE_double(S0_y_max, -0.5, "Maximum y value of S0.");
DEFINE_double(S1_x_min, -1, "Minimum x value of S1.");
DEFINE_double(S1_x_max, 0, "Maximum x value of S1.");
DEFINE_double(S1_y_min, 0.5, "Minimum y value of S1.");
DEFINE_double(S1_y_max, 1, "Maximum y value of S1.");
DEFINE_double(S2_x_min, -0.05, "Minimum x value of S2.");
DEFINE_double(S2_x_max, 0.05, "Maximum x value of S2.");
DEFINE_double(S2_y_min, -1, "Minimum y value of S2.");
DEFINE_double(S2_y_max, 1, "Maximum y value of S2.");
DEFINE_double(x0, 0.5, "Initial x value.");
DEFINE_double(y0, -0.75, "Initial x value.");
DEFINE_double(xf, -0.5, "Final x value.");
DEFINE_double(yf, 0.75, "Final x value.");

namespace drake {
namespace solvers {
namespace {
void PlotBox(const VectorX<double>& lower_bounds,
             const VectorX<double>& upper_bounds) {
  const std::vector<int> box_plot_x_signs{1, -1, -1, 1, 1};
  const std::vector<int> box_plot_y_signs{1, 1, -1, -1, 1};
  VectorX<double> box_plot_x(5);
  VectorX<double> box_plot_y(5);
  for (int i = 0; i < 5; ++i) {
    box_plot_x(i) = box_plot_x_signs[i] > 0 ? upper_bounds(0) : lower_bounds(0);
    box_plot_y(i) = box_plot_y_signs[i] > 0 ? upper_bounds(1) : lower_bounds(1);
  }
  drake::log()->info("box_x: {}", box_plot_x.transpose());
  drake::log()->info("box_y: {}", box_plot_y.transpose());
  CallPython("plot", box_plot_x, box_plot_y);
}

Formula BoxFormula(const VectorX<double>& lower_bounds,
                   const VectorX<double>& upper_bounds,
                   const VectorXDecisionVariable& x,
                   const Variable& indicator) {
  // clang-format off
  const auto A = (MatrixX<double>(4, 2) <<
                   MatrixX<double>::Identity(2, 2),
                  -MatrixX<double>::Identity(2, 2)
                  ).finished();
  // clang-format on
  const auto b =
      (MatrixX<double>(4, 1) << upper_bounds, -lower_bounds).finished();
  return A * x <= indicator * b;
}

BsplineCurve<Expression> AddCurveThroughRegions(
    const std::vector<Formula>& regions,
    const VectorXDecisionVariable& position_variables,
    const Variable& indicator_variable,
    const BsplineBasis<double>& basis,
    MathematicalProgram* program) {
  const std::vector<Formula>& S = regions;
  const int n = position_variables.size();
  const int n_S = static_cast<int>(S.size());
  const int n_x = basis.num_control_points();
  const int spline_order = basis.order();

  // Let 𝑥ₖ ∈ 𝑋 for 𝑘 ∈ {1, ..., 𝑛ₓ} be a collection of points in 𝑋.
  std::vector<MatrixXDecisionVariable> x;
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

  return {basis, x};
}

BsplineCurve<double> GetSolutionCurve(
    const MathematicalProgramResult& result,
    const BsplineCurve<Expression>& symbolic_curve) {
  std::vector<MatrixX<double>> x_sol{};
  for (const auto& x_k : symbolic_curve.control_points()) {
    Environment env{};
    for (const auto variable : symbolic::GetDistinctVariables(x_k)) {
      env.insert(variable, result.GetSolution(variable));
    }
    x_sol.push_back(symbolic::Evaluate(x_k, env));
  }
  return {symbolic_curve.basis(), x_sol};
}

}  // namespace

GTEST_TEST(BsplineMicpTests, ThreeRegionTest) {
  // Let 𝑋 be a metric space. In this case, 𝑋 = ℝ².
  // Let 𝑆ᵢ ⊆ 𝑋 for 𝑖 ∈ {1, ..., 𝑛ₛ} be a collection of sub-sets defined by
  //     𝑆ᵢ = {𝑥 ∈ 𝑋 | 𝑥̲ᵢ ≤ 𝑥 ≤ 𝑥̅ᵢ}
  const int n = 2;  // 𝑥ₖ ∈ ℝ²
  VectorXDecisionVariable x_placeholder(n);
  Variable indicator_placeholder("b");
  for (int i = 0; i < n; ++i) {
    x_placeholder(i) = Variable(fmt::format("x_placeholder_{}", i));
  }
  std::vector<Formula> S{};
  const auto S0_lower_bounds =
      (VectorX<double>(2, 1) << FLAGS_S0_x_min, FLAGS_S0_y_min).finished();
  const auto S0_upper_bounds =
      (VectorX<double>(2, 1) << FLAGS_S0_x_max, FLAGS_S0_y_max).finished();
  const auto S1_lower_bounds =
      (VectorX<double>(2, 1) << FLAGS_S1_x_min, FLAGS_S1_y_min).finished();
  const auto S1_upper_bounds =
      (VectorX<double>(2, 1) << FLAGS_S1_x_max, FLAGS_S1_y_max).finished();
  const auto S2_lower_bounds =
      (VectorX<double>(2, 1) << FLAGS_S2_x_min, FLAGS_S2_y_min).finished();
  const auto S2_upper_bounds =
      (VectorX<double>(2, 1) << FLAGS_S2_x_max, FLAGS_S2_y_max).finished();
  S.push_back(BoxFormula(S0_lower_bounds, S0_upper_bounds, x_placeholder,
                          indicator_placeholder));
  PlotBox(S0_lower_bounds, S0_upper_bounds);
  S.push_back(BoxFormula(S1_lower_bounds, S1_upper_bounds, x_placeholder,
                          indicator_placeholder));
  PlotBox(S1_lower_bounds, S1_upper_bounds);
  S.push_back(BoxFormula(S2_lower_bounds, S2_upper_bounds, x_placeholder,
                          indicator_placeholder));
  PlotBox(S2_lower_bounds, S2_upper_bounds);

  MathematicalProgram prog;
  BsplineBasis<double> basis{FLAGS_order, FLAGS_num_control_points,
                             FLAGS_clamped
                                 ? math::KnotVectorType::kClampedUniform
                                 : math::KnotVectorType::kUniform};
  BsplineCurve<Expression> x_curve_symbolic = AddCurveThroughRegions(
      S, x_placeholder, indicator_placeholder, basis, &prog);
  const auto xdot_curve_symbolic = x_curve_symbolic.Derivative();
  // const auto jerk_curve_symbolic = xdot_curve_symbolic.Derivative().Derivative();
  const double t0 = x_curve_symbolic.start_time();
  const double tf = x_curve_symbolic.end_time();
  const auto x0_symbolic = x_curve_symbolic.value(t0);
  const auto xf_symbolic = x_curve_symbolic.value(tf);
  const auto xdot0_symbolic = xdot_curve_symbolic.value(t0);
  const auto xdotf_symbolic = xdot_curve_symbolic.value(tf);
  prog.AddLinearEqualityConstraint(x0_symbolic == Vector2<double>(FLAGS_x0, FLAGS_y0));
  prog.AddLinearEqualityConstraint(xf_symbolic == Vector2<double>(FLAGS_xf, FLAGS_yf));
  prog.AddLinearEqualityConstraint(xdot0_symbolic == Vector2<double>::Zero());
  prog.AddLinearEqualityConstraint(xdotf_symbolic == Vector2<double>::Zero());

  // and costs on the control points of the velocity curve:
  // for (const auto& control_point : jerk_curve_symbolic.control_points()) {
  for (const auto& control_point : xdot_curve_symbolic.control_points()) {
    prog.AddQuadraticCost((control_point.transpose() * control_point)(0,0));
  }

  drake::log()->info("Calling Solve ...");
  MathematicalProgramResult result = Solve(prog);
  drake::log()->info("Done: {}", result.is_success());
  ASSERT_TRUE(result.is_success());

  const BsplineCurve<double> x_curve_sol =
      GetSolutionCurve(result, x_curve_symbolic);
  constexpr int num_plotting_points = 100;
  const auto t = VectorX<double>::LinSpaced(num_plotting_points, t0, tf);
  MatrixX<double> x_plotting(n, num_plotting_points);
  for (int i = 0; i < num_plotting_points; ++i) {
    x_plotting.col(i) = x_curve_sol.value(t(i));
  }

  CallPython("plot", x_plotting.row(0).transpose(), x_plotting.row(1).transpose());

  CallPython("axis", "equal");
  CallPython("grid", "on");
}

}  // namespace solvers
}  // namespace drake
