#pragma once

#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/common/symbolic.h"
#include "drake/geometry/optimization/convex_set.h"
#include "drake/solvers/mathematical_program_result.h"

namespace drake {
namespace geometry {
namespace optimization {

/**
 Design note: This class avoids providing any direct access to the
 MathematicalProgram that it constructs nor to the decision variables /
 constraints.  The users should be able to write constraints against faux
 decision variables on the vertices and edges, but these get translated in
 non-trivial ways to the underlying program.  Providing direct access to the
 slack variables is asking for trouble.

 The design also allows for the possibility of the shortest path details to be
 separated from the more general GraphsOfConvexSets machinery.  A shortest path
 problem simply adds some flow and degree constraints on top of an otherwise
 more general concept.
*/
class ShortestPathProblem {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ShortestPathProblem)

  /** Constructs the (empty) problem. */
  ShortestPathProblem() {}

  class Edge;  // forward declaration.

  class Vertex {
   public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Vertex)

    Vertex(const ConvexSet& set, std::string name);
    virtual ~Vertex() {}

    int size() const { return set_.ambient_dimension(); }
    const std::string& name() const { return name_; }
    const VectorX<symbolic::Variable>& x() const { return placeholder_x_; }
    const ConvexSet& set() const { return set_; }

    Eigen::VectorXd GetSolution(
        const solvers::MathematicalProgramResult& result) const;

    // TODO(russt): Support AddCost/AddConstraint directly on the vertices.

   private:
    const ConvexSet& set_;
    const std::string name_;
    const VectorX<symbolic::Variable> placeholder_x_{};
  };

  class Edge {
   public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Edge)

    Edge(const Vertex& u, const Vertex& v, std::string name);

    const std::string& name() const { return name_; }
    const Vertex& u() const { return u_; }
    const Vertex& v() const { return v_; }

    /** The binary variable associated with this edge.  Provided here only for
    use in examining the MathematicalProgramResult. */
    const symbolic::Variable& phi() const { return phi_; }

    double GetSolutionCost(
        const solvers::MathematicalProgramResult& result) const;

    const VectorX<symbolic::Variable>& xu() const { return u_.x(); }
    const VectorX<symbolic::Variable>& xv() const { return v_.x(); }

    /** Add a cost to this edge, described by a symbolic::Expression @p e
    containing *only* elements of xu() and xv() as variables.

    @throws std::exception if e.GetVariables() is not a subset of xu() ∩ xv().
    */
    std::pair<symbolic::Variable, solvers::Binding<solvers::Cost>> AddCost(
        const symbolic::Expression& e);

    /** Add a cost to this vertex, described by a solvers::Cost @p c.  @p vars
    must contain *only* elements of xu() and xv() as variables.

    @throws std::exception if vars is not a subset of xu() ∩ xv(). */
    std::pair<symbolic::Variable, solvers::Binding<solvers::Cost>> AddCost(
        const solvers::Binding<solvers::Cost>& binding);

    /** Add a cost to this edge, described by a symbolic::Formula @p f
    containing *only* elements of xu() and xv() as variables.

    @throws std::exception if f.GetVariables() is not a subset of xu() ∩ xv().
    */
    solvers::Binding<solvers::Constraint> AddConstraint(
        const symbolic::Formula& f);

    /** Add a constraint to this vertex, described by a solvers::Constraint @p
    c.  @p vars must contain *only* elements of xu() and xv() as variables.

    @throws std::exception if vars is not a subset of xu() ∩ xv(). */
    solvers::Binding<solvers::Constraint> AddConstraint(
        const solvers::Binding<solvers::Constraint>& binding);

   private:
    const Vertex& u_;
    const Vertex& v_;
    symbolic::Variables allowed_vars_;
    symbolic::Variable phi_;
    const VectorX<symbolic::Variable> y_{};
    const VectorX<symbolic::Variable> z_{};
    const std::string name_;

    std::unordered_map<symbolic::Variable, symbolic::Variable> x_to_yz_{};
    solvers::VectorXDecisionVariable ell_{};
    std::vector<solvers::Binding<solvers::Cost>> costs_{};
    std::vector<solvers::Binding<solvers::Constraint>> constraints_{};

    friend class ShortestPathProblem;
  };

  const std::vector<std::unique_ptr<Vertex>>& vertices() const {
    return vertices_;
  }
  const std::vector<std::unique_ptr<Edge>>& edges() const { return edges_; }

  Vertex* AddVertex(std::unique_ptr<ConvexSet> set, std::string name = "");

  /** Convenience method.  Clones the set then adds it to the program. */
  Vertex* AddVertex(const ConvexSet& set, std::string name = "");

  void set_source(const Vertex& s);
  void set_source(int s_index);
  void set_target(const Vertex& t);
  void set_target(int t_index);

  // TODO(russt): Provide AddVertexWithCopies or DuplicateVertex to facilitate
  // multiple visits to the same set.  This is the first reason why this class
  // owns the sets, not the vertices.

  Edge* AddEdge(const Vertex& u, const Vertex& v, std::string name = "");

  /** Convenience method that allows users to specify existing vertices by the
  order in which they were added (rather than curate the list of vertices
  themselves).  For complex graphs, prefer the variant that accepts `const
  Vertex&`. */
  Edge* AddEdge(int u_index, int v_index, std::string name = "");

  // TODO(russt): std::string GetGraphvizString(const
  // std::optional<solvers::MathematicalProgramResult>& = std::nullopt) const;

  // TODO(russt): add optional<Solver> argument.
  solvers::MathematicalProgramResult Solve(bool convex_relaxation = false);

 private:
  ConvexSets sets_{};
  std::vector<std::unique_ptr<Vertex>> vertices_{};
  std::vector<std::unique_ptr<Edge>> edges_{};
  const Vertex* source_{nullptr};
  const Vertex* target_{nullptr};
};

// Thinking ahead:
//
// class BSplineVertex : public ShortestPathProblem::Vertex {
//   BSplineVertex(ConvexSet, int order, int num_points);
//   AddVelocityConstraint...?
// }
// class BSplineEdge : public ShortestPathProblem::Edge {}
//
// class ProbabilisticRoadmapWithConvexSets {
// private:
//   ShortestPathProblem spp;
// }

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
