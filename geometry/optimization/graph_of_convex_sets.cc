#include "drake/geometry/optimization/graph_of_convex_sets.h"

#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <fmt/format.h>

#include "drake/math/quadratic_form.h"
#include "drake/solvers/create_cost.h"
#include "drake/solvers/mosek_solver.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace geometry {
namespace optimization {

typedef ShortestPathProblem::Vertex Vertex;
typedef ShortestPathProblem::Edge Edge;

using Eigen::MatrixXd;
using Eigen::Ref;
using Eigen::RowVectorXd;
using Eigen::VectorXd;
using solvers::Binding;
using solvers::Constraint;
using solvers::Cost;
using solvers::L2NormCost;
using solvers::LinearConstraint;
using solvers::LinearCost;
using solvers::LinearEqualityConstraint;
using solvers::MathematicalProgram;
using solvers::MathematicalProgramResult;
using solvers::QuadraticCost;
using solvers::VariableRefList;
using solvers::VectorXDecisionVariable;
using symbolic::Expression;
using symbolic::Variable;
using symbolic::Variables;

Vertex::Vertex(const ConvexSet& set, std::string name)
    : set_(set),
      name_(name),
      placeholder_x_(symbolic::MakeVectorContinuousVariable(
          set_.ambient_dimension(), name)) {}

VectorXd Vertex::GetSolution(const MathematicalProgramResult& result) const {
  return result.GetSolution(placeholder_x_);
}

Edge::Edge(const Vertex& u, const Vertex& v, std::string name)
    : u_{u},
      v_{v},
      allowed_vars_{u_.x()},
      phi_{"phi", symbolic::Variable::Type::BINARY},
      y_{symbolic::MakeVectorContinuousVariable(u_.size(), "y")},
      z_{symbolic::MakeVectorContinuousVariable(v_.size(), "z")},
      name_{name},
      x_to_yz_(u_.size() + v_.size()) {
  allowed_vars_.insert(Variables(v_.x()));
  for (int i = 0; i < u_.size(); ++i) {
    x_to_yz_.emplace(u_.x()[i], y_[i]);
  }
  for (int i = 0; i < v_.size(); ++i) {
    x_to_yz_.emplace(v_.x()[i], z_[i]);
  }
}

double Edge::GetSolutionCost(const MathematicalProgramResult& result) const {
  return result.GetSolution(ell_).sum();
}

std::pair<Variable, Binding<Cost>> Edge::AddCost(
    const symbolic::Expression& e) {
  return AddCost(solvers::internal::ParseCost(e));
}

std::pair<Variable, Binding<Cost>> Edge::AddCost(const Binding<Cost>& binding) {
  DRAKE_DEMAND(Variables(binding.variables()).IsSubsetOf(allowed_vars_));
  const int n = ell_.size();
  ell_.conservativeResize(n + 1);
  ell_[n] = Variable(fmt::format("ell{}", n), Variable::Type::CONTINUOUS);
  costs_.emplace_back(binding);
  return std::pair<Variable, Binding<Cost>>(ell_[n], costs_.back());
}

Binding<Constraint> Edge::AddConstraint(const symbolic::Formula& f) {
  return AddConstraint(solvers::internal::ParseConstraint(f));
}

Binding<Constraint> Edge::AddConstraint(const Binding<Constraint>& binding) {
  DRAKE_DEMAND(Variables(binding.variables()).IsSubsetOf(allowed_vars_));
  constraints_.emplace_back(binding);
  return constraints_.back();
}

Vertex* ShortestPathProblem::AddVertex(std::unique_ptr<ConvexSet> set,
                                       std::string name) {
  if (name.empty()) {
    name = fmt::format("v{}", vertices_.size());
  }
  sets_.emplace_back(std::move(set));
  vertices_.emplace_back(std::make_unique<Vertex>(*sets_.back(), name));
  return vertices_.back().get();
}

Vertex* ShortestPathProblem::AddVertex(const ConvexSet& set, std::string name) {
  return AddVertex(set.Clone(), name);
}

void ShortestPathProblem::set_source(const Vertex& s) { source_ = &s; }

void ShortestPathProblem::set_source(int s_index) {
  DRAKE_DEMAND(s_index >= 0 && s_index < static_cast<int>(vertices_.size()));
  set_source(*vertices_[s_index]);
}

void ShortestPathProblem::set_target(const Vertex& t) { target_ = &t; }

void ShortestPathProblem::set_target(int t_index) {
  DRAKE_DEMAND(t_index >= 0 && t_index < static_cast<int>(vertices_.size()));
  set_target(*vertices_[t_index]);
}

Edge* ShortestPathProblem::AddEdge(const Vertex& u, const Vertex& v,
                                   std::string name) {
  if (name.empty()) {
    name = fmt::format("e{}", edges_.size());
  }
  edges_.emplace_back(std::make_unique<Edge>(u, v, name));
  return edges_.back().get();
}

Edge* ShortestPathProblem::AddEdge(int u_index, int v_index, std::string name) {
  DRAKE_DEMAND(u_index >= 0 && u_index < static_cast<int>(vertices_.size()));
  DRAKE_DEMAND(v_index >= 0 && v_index < static_cast<int>(vertices_.size()));
  return AddEdge(*vertices_[u_index], *vertices_[v_index], name);
}

MathematicalProgramResult ShortestPathProblem::Solve(bool) {
  MathematicalProgram prog;

  std::unordered_map<const Vertex*, std::vector<Edge*>> incoming_edges(
      vertices_.size());
  std::unordered_map<const Vertex*, std::vector<Edge*>> outgoing_edges(
      vertices_.size());
  const double inf = std::numeric_limits<double>::infinity();

  for (const auto& e : edges_) {
    outgoing_edges[&e->u()].emplace_back(e.get());
    incoming_edges[&e->v()].emplace_back(e.get());

    prog.AddDecisionVariables(Vector1<Variable>(e->phi_));
    prog.AddDecisionVariables(e->y_);
    prog.AddDecisionVariables(e->z_);
    prog.AddDecisionVariables(e->ell_);
    prog.AddLinearCost(VectorXd::Ones(e->ell_.size()), e->ell_);

    // Spatial non-negativity: y ∈ ϕX, z ∈ ϕX.
    e->u().set().AddPointInNonnegativeScalingConstraints(&prog, e->y_, e->phi_);
    e->v().set().AddPointInNonnegativeScalingConstraints(&prog, e->z_, e->phi_);

    // Edge costs.
    for (int i = 0; i < e->ell_.size(); ++i) {
      const Binding<Cost>& b = e->costs_[i];

      const VectorXDecisionVariable& old_vars = b.variables();
      VectorXDecisionVariable vars(old_vars.size() + 2);
      // vars = [phi; ell; yz_vars]
      vars[0] = e->phi_;
      vars[1] = e->ell_[i];
      for (int j = 0; j < old_vars.size(); ++j) {
        vars[j + 2] = e->x_to_yz_.at(old_vars[j]);
      }

      // TODO(russt): Avoid this use of RTTI, which mirrors the current pattern
      // in MathematicalProgram::AddCost.
      Cost* cost = b.evaluator().get();
      if (LinearCost* lc = dynamic_cast<LinearCost*>(cost)) {
        // a*old_vars + b is now
        // a*yz_vars + phi*b <= ell or [b, -1.0, a][phi; ell; yz_vars] <= 0
        RowVectorXd a(vars.size());
        a[0] = lc->b();
        a[1] = -1.0;
        a.tail(old_vars.size()) = lc->a();
        prog.AddLinearConstraint(a, -inf, 0.0, vars);
      } else if (QuadraticCost* qc = dynamic_cast<QuadraticCost*>(cost)) {
        // .5 x'Qx + b'x + c becomes a rotated Lorentz cone constraint:
        // y ≥ 0, ℓ ≥ 0, and  y ϕ ≥ x'Qx, with y := ℓ - b'x - ϕ c.

        // TODO(russt): Allow users to set this tolerance.  (Probably in
        // solvers::QuadraticCost instead of here).
        const double tol = 1e-10;
        Eigen::MatrixXd R =
            math::DecomposePSDmatrixIntoXtransposeTimesX(.5 * qc->Q(), tol);
        MatrixXd A_cone = MatrixXd::Zero(R.rows() + 2, vars.size());
        A_cone(0, 0) = 1.0;  // z₀ = ϕ.
        // z₁ = ℓ - b'x - ϕ c.
        A_cone(1, 1) = 1.0;
        A_cone.block(1, 2, 1, qc->b().rows()) = -qc->b().transpose();
        A_cone(1, 0) = -qc->c();
        // z₂ ... z_{n+1} = R x.
        A_cone.block(2, 2, R.rows(), R.cols()) = R;
        prog.AddRotatedLorentzConeConstraint(
            A_cone, VectorXd::Zero(A_cone.rows()), vars);
      } else if (L2NormCost* l2c = dynamic_cast<L2NormCost*>(cost)) {
        // |Ax + b|₂ becomes ℓ ≥ |Ax+bϕ|₂.
        MatrixXd A_cone = MatrixXd::Zero(l2c->A().rows() + 2, vars.size());
        A_cone(0, 1) = 1.0;                                 // z₀ = ℓ.
        A_cone.block(1, 0, l2c->A().rows(), 1) = l2c->b();  // bϕ.
        A_cone.block(1, 2, l2c->A().rows(), l2c->A().cols()) = l2c->A();  // Ax.
        prog.AddLorentzConeConstraint(A_cone, VectorXd::Zero(A_cone.rows()),
                                      vars);
      } else {
        throw std::runtime_error(fmt::format(
            "ShortestPathProblem::Edge does not support this binding type: {}",
            b.to_string()));
      }
    }

    // Edge constraints.
    for (const Binding<Constraint>& b : e->constraints_) {
      const VectorXDecisionVariable& old_vars = b.variables();
      VectorXDecisionVariable vars(old_vars.size() + 1);
      // vars = [phi; yz_vars]
      vars[0] = e->phi_;
      for (int j = 0; j < old_vars.size(); ++j) {
        vars[j + 1] = e->x_to_yz_.at(old_vars[j]);
      }

      // Note: The use of perspective functions here does not check (nor assume)
      // that the constraints describe a bounded set.  The boundedness is
      // ensured by the intersection of these constraints with the convex sets
      // (on the vertices).
      Constraint* constraint = b.evaluator().get();
      if (LinearEqualityConstraint* lec =
              dynamic_cast<LinearEqualityConstraint*>(constraint)) {
        // A*x = b becomes A*x = phi*b.
        MatrixXd Aeq(lec->A().rows(), lec->A().cols() + 1);
        Aeq.col(0) = -lec->lower_bound();
        Aeq.rightCols(lec->A().cols()) = lec->A();
        prog.AddLinearEqualityConstraint(Aeq, VectorXd::Zero(lec->A().rows()),
                                         vars);
        // Note that LinearEqualityConstraint must come before LinearConstraint,
        // because LinearEqualityConstraint isa LinearConstraint.
      } else if (LinearConstraint* lc =
                     dynamic_cast<LinearConstraint*>(constraint)) {
        // lb <= A*x <= ub becomes
        // A*x <= phi*ub and phi*lb <= A*x, which can be spelled
        // [-ub, A][phi; x] <= 0, and 0 <= [-lb, A][phi; x].
        RowVectorXd a(vars.size());
        for (int i = 0; i < lc->A().rows(); ++i) {
          if (std::isfinite(lc->upper_bound()[i])) {
            a[0] = -lc->upper_bound()[i];
            a.tail(lc->A().cols()) = lc->A().row(i);
            prog.AddLinearConstraint(a, -inf, 0, vars);
          }
          if (std::isfinite(lc->lower_bound()[i])) {
            a[0] = -lc->lower_bound()[i];
            a.tail(lc->A().cols()) = lc->A().row(i);
            prog.AddLinearConstraint(a, 0, inf, vars);
          }
        }
      } else {
        throw std::runtime_error(
            fmt::format("ShortestPathProblem::Edge does not support this "
                        "binding type: {}",
                        b.to_string()));
      }
    }
  }

  for (const auto& v : vertices_) {
    const bool is_source = (source_ == v.get());
    const bool is_target = (target_ == v.get());

    const std::vector<Edge*>& incoming = incoming_edges[v.get()];
    const std::vector<Edge*>& outgoing = outgoing_edges[v.get()];

    // TODO(russt): Make the bindings of these constraints available to the user
    // so that they can check the dual solution.  Or perhaps better, create more
    // placeholder variables for the dual solutions, and just pack them into the
    // program result in the standard way.

    if (incoming.size() + outgoing.size() > 0) {
      VectorXDecisionVariable vars(incoming.size() + outgoing.size());
      RowVectorXd a(incoming.size() + outgoing.size());
      a << RowVectorXd::Constant(incoming.size(), -1.0),
          RowVectorXd::Ones(outgoing.size());

      // Conservation of flow: ∑ ϕ_out - ∑ ϕ_in = δ(is_source) - δ(is_target).
      int count = 0;
      for (const auto* e : incoming) {
        vars[count++] = e->phi_;
      }
      for (const auto* e : outgoing) {
        vars[count++] = e->phi_;
      }
      prog.AddLinearEqualityConstraint(
          a, (is_source ? 1.0 : 0.0) - (is_target ? 1.0 : 0.0), vars);

      // Spatial conservation of flow: ∑ z_in = ∑ y_out.
      if (!is_source && !is_target) {
        for (int i = 0; i < v->size(); ++i) {
          count = 0;
          for (const auto* e : incoming) {
            vars[count++] = e->z_[i];
          }
          for (const auto* e : outgoing) {
            vars[count++] = e->y_[i];
          }
          prog.AddLinearEqualityConstraint(a, 0, vars);
        }
      }
    }

    // Degree constraint: ∑ ϕ_out <= δ(is_target).
    if (outgoing.size() > 0) {
      VectorXDecisionVariable phi_out(outgoing.size());
      for (int i = 0; i < static_cast<int>(outgoing.size()); ++i) {
        phi_out[i] = outgoing[i]->phi_;
      }
      prog.AddLinearConstraint(RowVectorXd::Ones(outgoing.size()), 0.0,
                               is_target ? 0.0 : 1.0, phi_out);
    }
  }

  /* See #15359
  solvers::SolverOptions options;
  options.SetOption(solvers::CommonSolverOption::kPrintToConsole, 1);
  prog.SetSolverOptions(options);
  MathematicalProgramResult result = solvers::Solve(prog);
  */
  solvers::MosekSolver solver;
  //  solver.set_stream_logging(true, "");
  //  std::cout << prog << std::endl;
  MathematicalProgramResult result = solver.Solve(prog);

  // Push the placeholder variables into the result, so that they can be
  // accessed as if they were real variables.
  int num_placeholder_vars = 0;
  for (const auto& v : vertices_) {
    num_placeholder_vars += v->size();
  }
  std::unordered_map<symbolic::Variable::Id, int> decision_variable_index =
      prog.decision_variable_index();
  int count = result.get_x_val().size();
  Eigen::VectorXd x_val(count + num_placeholder_vars);
  x_val.head(count) = result.get_x_val();
  for (const auto& v : vertices_) {
    const bool is_target = (target_ == v.get());
    VectorXd v_x = VectorXd::Zero(v->size());
    if (is_target) {
      for (const auto& e : incoming_edges[v.get()]) {
        v_x += result.GetSolution(e->z_);
      }
    } else {
      for (const auto& e : outgoing_edges[v.get()]) {
        v_x += result.GetSolution(e->y_);
      }
    }
    for (int i = 0; i < v->size(); ++i) {
      decision_variable_index.emplace(v->x()[i].get_id(), count);
      x_val[count++] = v_x[i];
    }
  }
  result.set_decision_variable_index(decision_variable_index);
  result.set_x_val(x_val);
  return result;
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
