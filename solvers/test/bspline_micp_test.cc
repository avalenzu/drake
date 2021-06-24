#include <vector>

#include <gflags/gflags.h>
#include <gtest/gtest.h>
#include <fmt/format.h>

#include "drake/common/proto/call_python.h"
#include "drake/common/symbolic.h"
#include "drake/common/text_logging.h"
#include "drake/common/trajectories/bspline_trajectory.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/bspline_curve_optimization_utilities.h"
#include "drake/solvers/solve.h"

using drake::common::CallPython;
using drake::math::BsplineBasis;
using drake::symbolic::Environment;
using drake::symbolic::Expression;
using drake::symbolic::Formula;
using drake::symbolic::Substitution;
using drake::symbolic::Variable;
using drake::symbolic::Variables;
using drake::trajectories::BsplineTrajectory;

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
  CallPython("clf");
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
  BsplineTrajectory<Expression> x_curve_symbolic = AddCurveThroughRegions(
      S, x_placeholder, indicator_placeholder, basis, &prog);
  const std::unique_ptr<BsplineTrajectory<Expression>> xdot_curve_symbolic(
      static_cast<BsplineTrajectory<Expression>*>(
          x_curve_symbolic.MakeDerivative().release()));
  // const auto jerk_curve_symbolic = xdot_curve_symbolic.Derivative().Derivative();
  const double t0 = ExtractDoubleOrThrow(x_curve_symbolic.start_time());
  const double tf = ExtractDoubleOrThrow(x_curve_symbolic.end_time());
  const auto x0_symbolic = x_curve_symbolic.value(t0);
  const auto xf_symbolic = x_curve_symbolic.value(tf);
  const auto xdot0_symbolic = xdot_curve_symbolic->value(t0);
  const auto xdotf_symbolic = xdot_curve_symbolic->value(tf);
  prog.AddLinearEqualityConstraint(x0_symbolic == Vector2<double>(FLAGS_x0, FLAGS_y0));
  prog.AddLinearEqualityConstraint(xf_symbolic == Vector2<double>(FLAGS_xf, FLAGS_yf));
  prog.AddLinearEqualityConstraint(xdot0_symbolic == Vector2<double>::Zero());
  prog.AddLinearEqualityConstraint(xdotf_symbolic == Vector2<double>::Zero());

  // and costs on the control points of the velocity curve:
  // for (const auto& control_point : jerk_curve_symbolic.control_points()) {
  for (const auto& control_point : xdot_curve_symbolic->control_points()) {
    prog.AddQuadraticCost((control_point.transpose() * control_point)(0,0));
  }

  drake::log()->info("Calling Solve ...");
  MathematicalProgramResult result = Solve(prog);
  drake::log()->info("Done: {}", result.is_success());
  ASSERT_TRUE(result.is_success());

  const BsplineTrajectory<double> x_curve_sol =
      GetSolutionCurve(result, x_curve_symbolic);
  int num_plotting_points = FLAGS_num_plotting_points;
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
