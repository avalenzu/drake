#include "drake/geometry/optimization/bspline_graphs_of_convex_sets.h"

#include <map>

#include <fmt/format.h>

#include "drake/geometry/optimization/graph_of_convex_sets.h"
#include "drake/math/bspline_basis.h"
#include "drake/math/knot_vector_type.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mathematical_program_result.h"
#include "drake/solvers/solve.h"
#include "drake/solvers/constraint.h"
#include "drake/common/symbolic.h"
#include "drake/common/symbolic_decompose.h"
#include "drake/common/pointer_cast.h"

namespace drake {
namespace geometry {
namespace optimization {

using Eigen::MatrixXd;
using Eigen::VectorXd;
using math::BsplineBasis;
using math::KnotVectorType;
using symbolic::DecomposeLinearExpressions;
using symbolic::Expression;
using symbolic::ExtractVariablesFromExpression;
using symbolic::intersect;
using symbolic::Variable;
using symbolic::Variables;
using trajectories::BsplineTrajectory;

BsplineTrajectoryThroughUnionOfHPolyhedra::
    BsplineTrajectoryThroughUnionOfHPolyhedra(VectorX<double> source,
                                              VectorX<double> target,
                                              std::vector<HPolyhedron> regions)
    : source_index_(regions.size()),
      target_index_(regions.size() + 1),
      source_(source),
      target_(target),
      regions_(regions) {
  DRAKE_THROW_UNLESS(source.size() == ambient_dimension());
  DRAKE_THROW_UNLESS(target.size() == ambient_dimension());
  max_velocity_ = VectorXd::Constant(ambient_dimension(),
                                     std::numeric_limits<double>::infinity());
  for (const auto& region : regions_) {
    DRAKE_THROW_UNLESS(region.ambient_dimension() == ambient_dimension());
  }
  regions_.push_back(HPolyhedron::MakeBox(source, source));
  regions_.push_back(HPolyhedron::MakeBox(target, target));
}

std::optional<trajectories::BsplineTrajectory<double>>
BsplineTrajectoryThroughUnionOfHPolyhedra::Solve(bool) const {
  DRAKE_THROW_UNLESS(max_repetitions() == 1);
  ShortestPathProblem problem{};

  // Construct edges between all intersecting regions.
  std::multimap<int, int> edge_descriptions;
  for (int i{0}; i < num_regions(); ++i) {
    for (int j{i + 1}; j < num_regions(); ++j) {
      if (regions_[i].IntersectsWith(regions_[j])) {
        edge_descriptions.insert({i, j});
        edge_descriptions.insert({j, i});
      }
    }
  }

  std::vector<HPolyhedron> replicated_regions{};
  const int control_points_per_region =
      order() - 1 + extra_control_points_per_region_;
  for (const auto& region : regions_) {
    replicated_regions.push_back(
        region.CartesianPower(control_points_per_region));
  }
  std::vector<ShortestPathProblem::Vertex*> vertices;
  for (const auto& region : replicated_regions) {
    vertices.push_back(problem.AddVertex(region));
  }
  ShortestPathProblem::Vertex* source_region = vertices[source_index_];
  ShortestPathProblem::Vertex* target_region = vertices[target_index_];

  // Set up edge cost.
  int control_points_per_edge = 2 * control_points_per_region;
  MatrixXd H{};
  {
    MatrixX<Variable> u_control_points = symbolic::MakeMatrixVariable(
        ambient_dimension(), control_points_per_region, "xu");
    MatrixX<Variable> v_control_points = symbolic::MakeMatrixVariable(
        ambient_dimension(), control_points_per_region, "xv");
    std::vector<MatrixX<Expression>> edge_control_points;
    edge_control_points.reserve(control_points_per_edge);
    for (int i{0}; i < control_points_per_region; ++i) {
      edge_control_points.push_back(u_control_points.col(i));
    }
    for (int i{0}; i < control_points_per_region; ++i) {
      edge_control_points.push_back(v_control_points.col(i));
    }
    BsplineTrajectory<Expression> position_trajectory{
      BsplineBasis<Expression>(order(),
                               control_points_per_edge,
                               KnotVectorType::kUniform, 0.0,
                               control_points_per_edge - (order() - 1.0)),
      edge_control_points
    };
    int min_derivative_order{1};
    int max_derivative_order{1};
    std::vector<Expression> cost_components{};
    for (int derivative_order{min_derivative_order}; derivative_order <= max_derivative_order; ++derivative_order) {
      std::unique_ptr<BsplineTrajectory<Expression>> derivative_trajectory{
          dynamic_pointer_cast_or_throw<BsplineTrajectory<Expression>>(
              position_trajectory.MakeDerivative(derivative_order))};
      for (const auto& control_point : derivative_trajectory->control_points()) {
        for (int i{0}; i < ambient_dimension(); ++i) {
          Variables variables_in_this_expression{
              ExtractVariablesFromExpression(control_point(i, 0)).first};
          Variables variables_in_u_control_points{Eigen::Map<VectorX<Variable>>(
              u_control_points.data(), u_control_points.size())};
          if (!intersect(
                   variables_in_u_control_points,
                   variables_in_this_expression)
                   .empty()) {
            cost_components.push_back(control_point(i, 0));
          }
        }
      }
    }
    VectorX<Variable> edge_variables(control_points_per_edge *
                                     ambient_dimension());
    edge_variables << Eigen::Map<VectorX<Variable>>(u_control_points.data(),
                                                    u_control_points.size()),
        Eigen::Map<VectorX<Variable>>(v_control_points.data(),
                                      u_control_points.size());
    H.resize(cost_components.size(), edge_variables.size());
    DecomposeLinearExpressions(
        Eigen::Map<VectorX<Expression>>(cost_components.data(),
                                        cost_components.size()),
        edge_variables, &H);
  }
  auto cost =
      std::make_shared<solvers::L2NormCost>(H, VectorXd::Zero(H.rows()));
  drake::log()->info("H =\n{}", H);

  // Add edges.
  std::vector<ShortestPathProblem::Edge*> edges;
  edges.reserve(edge_descriptions.size());
  for (const auto& edge_description : edge_descriptions) {
    edges.push_back(
        problem.AddEdge(edge_description.first, edge_description.second,
                        fmt::format("({}, {})", edge_description.first,
                                    edge_description.second)));
    ShortestPathProblem::Edge& edge = *edges.back();
    edge.AddCost(solvers::Binding(cost, {edge.xu(), edge.xv()}));
    const Eigen::Map<const MatrixX<Variable>> v_control_points(
        edge.xv().data(), ambient_dimension(),
        control_points_per_region);
    const HPolyhedron& Xu = regions_.at(edge_description.first);
    for (int j{0}; j < order() - 1; ++j) {
      edge.AddConstraint({std::make_shared<solvers::LinearConstraint>(
          Xu.A(),
          VectorXd::Constant(Xu.b().size(),
                             -std::numeric_limits<double>::infinity()),
          Xu.b()), v_control_points.col(j)});
    }
    drake::log()->info("Added edge {} ...", edge.name());
  }

  problem.set_source(*source_region);
  problem.set_target(*target_region);

  solvers::MathematicalProgramResult result = problem.Solve();

  if (!result.is_success()) return std::nullopt;

  std::vector<ShortestPathProblem::Edge*> active_edges;
  for (const auto& edge : edges) {
    drake::log()->info("Checking edge {}", edge->name());
    const double phi{result.GetSolution(edge->phi())};
    if (&edge->u() == source_region && phi > 0.0) {
      drake::log()->info("Added edge {} ...", edge->name());
      active_edges.push_back(edge);
      break;
    }
  }
  drake::log()->info("Found first active edge ...");
  while (&active_edges.back()->v() != target_region) {
    drake::log()->info("Looking for successor to {}",
                       active_edges.back()->name());
    for (const auto& edge : edges) {
      drake::log()->info("Checking edge {}", edge->name());
      if (&edge->u() == &active_edges.back()->v() &&
          result.GetSolution(edge->phi()) > 0.0) {
        drake::log()->info("Added edge {} ...", edge->name());
        active_edges.push_back(edge);
        break;
      }
    }
  }

  std::vector<MatrixXd> control_points{};
  control_points.reserve(control_points_per_region * (active_edges.size() + 1));
  {
    VectorXd vertex_control_points_flattened =
        result.GetSolution(active_edges.front()->xu());
    drake::log()->info("vertex_control_points_flattened.size(): {}", vertex_control_points_flattened.size());
    Eigen::Map<MatrixXd> vertex_control_points(
        vertex_control_points_flattened.data(), ambient_dimension(),
        control_points_per_region);
    drake::log()->info("vertex_control_points:\n{}", vertex_control_points);
    for (int i{0}; i < control_points_per_region; ++i) {
      control_points.push_back(vertex_control_points.col(i));
    }
  }
  for (const auto& edge : active_edges) {
    VectorXd vertex_control_points_flattened = result.GetSolution(edge->xv());
    drake::log()->info("vertex_control_points_flattened.size(): {}", vertex_control_points_flattened.size());
    Eigen::Map<MatrixXd> vertex_control_points(
        vertex_control_points_flattened.data(), ambient_dimension(),
        control_points_per_region);
    drake::log()->info("vertex_control_points:\n{}", vertex_control_points);
    for (int i{0}; i < control_points_per_region; ++i) {
      control_points.push_back(vertex_control_points.col(i));
    }
  }
  drake::log()->info("Solution trajectory has {} control points",
                     control_points.size());

  for (const auto& control_point : control_points) {
    drake::log()->info(control_point.transpose());
  }

  return trajectories::BsplineTrajectory<double>(
      math::BsplineBasis<double>(order(), control_points.size(),
                                 math::KnotVectorType::kUniform),
      control_points);
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
