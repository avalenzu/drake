#include "drake/geometry/optimization/graph_of_convex_sets.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/hyperellipsoid.h"
#include "drake/geometry/optimization/point.h"
#include "drake/geometry/optimization/vpolytope.h"

namespace drake {
namespace geometry {
namespace optimization {

using Eigen::Matrix;
using Eigen::Matrix2d;
using Eigen::Matrix4d;
using Eigen::RowVector2d;
using Eigen::Vector2d;
using Eigen::Vector4d;
using symbolic::Environment;
using symbolic::Expression;
using symbolic::Substitution;
typedef ShortestPathProblem::Vertex Vertex;
typedef ShortestPathProblem::Edge Edge;

/*
Let's me test with one edge defintely on the optimal path, and one definitely
off it.
┌──────┐         ┌──────┐
│source├──e_on──►│target│
└───┬──┘         └──────┘
    │e_off
    │
┌───▼──┐
│ sink │
└──────┘
*/
class ThreePoints : public ::testing::Test {
 protected:
  void SetUp() {
    auto p1 = std::make_unique<Point>(Vector2d(3., 5.));
    p_source_ = p1.get();
    auto source = spp_.AddVertex(std::move(p1));
    auto p2 = std::make_unique<Point>(Vector2d(-2., 4.));
    p_target_ = p2.get();
    auto target = spp_.AddVertex(std::move(p2));
    e_on_ = spp_.AddEdge(*source, *target);
    auto p3 = std::make_unique<Point>(Vector2d(5., -2.3));
    p_sink_ = p3.get();
    auto sink = spp_.AddVertex(std::move(p3));
    e_off_ = spp_.AddEdge(*source, *sink);
    spp_.set_source(*source);
    spp_.set_target(*target);

    subs_on_off_.emplace(e_on_->xv()[0], e_off_->xv()[0]);
    subs_on_off_.emplace(e_on_->xv()[1], e_off_->xv()[1]);
  }

  ShortestPathProblem spp_;
  Point* p_source_{nullptr};
  Point* p_target_{nullptr};
  Point* p_sink_{nullptr};
  ShortestPathProblem::Edge* e_on_{nullptr};
  ShortestPathProblem::Edge* e_off_{nullptr};
  Substitution subs_on_off_{};
};

TEST_F(ThreePoints, LinearCost1) {
  e_on_->AddCost(1.0);
  e_off_->AddCost(1.0);
  auto result = spp_.Solve(true);
  ASSERT_TRUE(result.is_success());
  EXPECT_NEAR(e_on_->GetSolutionCost(result), 1.0, 1e-6);
  EXPECT_NEAR(e_off_->GetSolutionCost(result), 0.0, 1e-6);
}

TEST_F(ThreePoints, LinearCost2) {
  const Vector2d a{2.3, 4.5}, b{5.6, 7.8};
  const double c = 1.23;
  e_on_->AddCost(a.dot(e_on_->xu()) + b.dot(e_on_->xv()) + c);
  e_off_->AddCost(a.dot(e_off_->xu()) + b.dot(e_off_->xv()) + c);
  auto result = spp_.Solve(true);
  ASSERT_TRUE(result.is_success());
  EXPECT_NEAR(e_on_->GetSolutionCost(result),
              a.dot(p_source_->x()) + b.dot(p_target_->x()) + c, 1e-6);
  EXPECT_NEAR(e_off_->GetSolutionCost(result), 0.0, 1e-6);
}

TEST_F(ThreePoints, LinearCost3) {
  const Vector4d a{2.3, 4.5, 5.6, 7.8};
  const double b = 1.23;
  auto cost = std::make_shared<solvers::LinearCost>(a, b);
  e_on_->AddCost(solvers::Binding(cost, {e_on_->xu(), e_on_->xv()}));
  e_off_->AddCost(solvers::Binding(cost, {e_off_->xu(), e_off_->xv()}));
  auto result = spp_.Solve(true);
  ASSERT_TRUE(result.is_success());
  EXPECT_NEAR(e_on_->GetSolutionCost(result),
              a.head(2).dot(p_source_->x()) + a.tail(2).dot(p_target_->x()) + b,
              1e-6);
  EXPECT_NEAR(e_off_->GetSolutionCost(result), 0.0, 1e-6);
}

TEST_F(ThreePoints, QuadraticCost) {
  e_on_->AddCost((e_on_->xu() - e_on_->xv()).squaredNorm());
  e_off_->AddCost((e_off_->xu() - e_off_->xv()).squaredNorm());
  auto result = spp_.Solve(true);
  ASSERT_TRUE(result.is_success());
  EXPECT_NEAR(e_on_->GetSolutionCost(result),
              (p_source_->x() - p_target_->x()).squaredNorm(), 1e-6);
  EXPECT_NEAR(e_off_->GetSolutionCost(result), 0.0, 1e-6);
}

TEST_F(ThreePoints, QuadraticCost2) {
  Matrix2d A;
  A << 4.3, .5, -.4, 1.2;
  // Make an arbitrary non-negative quadratic form, which goes to zero at a
  // point.
  Expression cost = (A * (e_on_->xu() - Vector2d{.54, -.23}) +
                     A * (e_on_->xv() - Vector2d{7.2, -.73}))
                        .squaredNorm();
  e_on_->AddCost(cost);
  e_off_->AddCost(cost.Substitute(subs_on_off_));
  auto result = spp_.Solve(true);
  ASSERT_TRUE(result.is_success());
  Environment env{};
  env.insert(e_on_->xu(), p_source_->x());
  env.insert(e_on_->xv(), p_target_->x());
  EXPECT_NEAR(e_on_->GetSolutionCost(result), cost.Evaluate(env), 1e-5);
  EXPECT_NEAR(e_off_->GetSolutionCost(result), 0.0, 1e-6);
}

TEST_F(ThreePoints, QuadraticCost3) {
  Matrix2d A;
  A << 4.3, .5, -.4, 1.2;
  // Make an arbitrary positive quadratic form (by adding a constant).
  Expression cost = (A * (e_on_->xu() - Vector2d{.54, -.23}) +
                     A * (e_on_->xv() - Vector2d{7.2, -.73}))
                        .squaredNorm() +
                    4.2;
  e_on_->AddCost(cost);
  e_off_->AddCost(cost.Substitute(subs_on_off_));
  auto result = spp_.Solve(true);
  ASSERT_TRUE(result.is_success());
  Environment env{};
  env.insert(e_on_->xu(), p_source_->x());
  env.insert(e_on_->xv(), p_target_->x());
  EXPECT_NEAR(e_on_->GetSolutionCost(result), cost.Evaluate(env), 1e-5);
  EXPECT_NEAR(e_off_->GetSolutionCost(result), 0.0, 1e-6);
}

TEST_F(ThreePoints, QuadraticCost4) {
  const Matrix4d R = Vector4d(3.2, 4.3, 6.4, 7.1).asDiagonal();
  const Vector4d d = Vector4d(0.1, 0.2, 0.3, 0.4);
  auto cost = std::make_shared<solvers::QuadraticCost>(
      2.0 * R.transpose() * R, 2.0 * R.transpose() * d, d.dot(d));
  e_on_->AddCost(solvers::Binding(cost, {e_on_->xu(), e_on_->xv()}));
  e_off_->AddCost(solvers::Binding(cost, {e_off_->xu(), e_off_->xv()}));
  auto result = spp_.Solve(true);
  ASSERT_TRUE(result.is_success());
  Vector4d x;
  x << p_source_->x(), p_target_->x();
  EXPECT_NEAR(e_on_->GetSolutionCost(result), (R * x + d).squaredNorm(), 1e-6);
  EXPECT_NEAR(e_off_->GetSolutionCost(result), 0.0, 1e-6);
}

TEST_F(ThreePoints, L2NormCost) {
  // |xu - xv|₂
  Matrix<double, 2, 4> A;
  A.leftCols(2) = Matrix2d::Identity();
  A.rightCols(2) = -Matrix2d::Identity();
  auto cost = std::make_shared<solvers::L2NormCost>(A, Vector2d::Zero());
  e_on_->AddCost(solvers::Binding(cost, {e_on_->xu(), e_on_->xv()}));
  e_off_->AddCost(solvers::Binding(cost, {e_off_->xu(), e_off_->xv()}));
  auto result = spp_.Solve(true);
  ASSERT_TRUE(result.is_success());
  EXPECT_NEAR(e_on_->GetSolutionCost(result),
              (p_source_->x() - p_target_->x()).norm(), 1e-6);
  EXPECT_NEAR(e_off_->GetSolutionCost(result), 0.0, 1e-6);
}

TEST_F(ThreePoints, L2NormCost2) {
  // L2-norm of an arbitrary transformation of xu and xv.
  Matrix<double, 2, 4> A;
  // clang-format off
  A << 4.3, .5, -.4, 1.2,
       0.1, .2, -2., -.34;
  // clang-format on
  const Vector2d b{.5, .3};
  auto cost = std::make_shared<solvers::L2NormCost>(A, b);
  e_on_->AddCost(solvers::Binding(cost, {e_on_->xu(), e_on_->xv()}));
  e_off_->AddCost(solvers::Binding(cost, {e_off_->xu(), e_off_->xv()}));
  auto result = spp_.Solve(true);
  ASSERT_TRUE(result.is_success());
  EXPECT_NEAR(
      e_on_->GetSolutionCost(result),
      (A.leftCols(2) * p_source_->x() + A.rightCols(2) * p_target_->x() + b)
          .norm(),
      1e-6);
  EXPECT_NEAR(e_off_->GetSolutionCost(result), 0.0, 1e-6);
}

// Like the ThreePoints, but with boxes for each vertex instead of points.
class ThreeBoxes : public ::testing::Test {
 protected:
  void SetUp() {
    auto box = HPolyhedron::MakeUnitBox(2);
    source_ = spp_.AddVertex(box, "source");
    target_ = spp_.AddVertex(box, "target");
    sink_ = spp_.AddVertex(box, "sink");
    e_on_ = spp_.AddEdge(*source_, *target_);
    e_off_ = spp_.AddEdge(*source_, *sink_);
    spp_.set_source(*source_);
    spp_.set_target(*target_);

    subs_on_off_.emplace(e_on_->xv()[0], e_off_->xv()[0]);
    subs_on_off_.emplace(e_on_->xv()[1], e_off_->xv()[1]);
  }

  ShortestPathProblem spp_;
  ShortestPathProblem::Edge* e_on_{nullptr};
  ShortestPathProblem::Edge* e_off_{nullptr};
  ShortestPathProblem::Vertex* source_{nullptr};
  ShortestPathProblem::Vertex* target_{nullptr};
  ShortestPathProblem::Vertex* sink_{nullptr};
  Substitution subs_on_off_{};
};

TEST_F(ThreeBoxes, LinearEqualityConstraint) {
  const Vector2d b{.5, .3};
  e_on_->AddConstraint(e_on_->xv() == b);
  e_off_->AddConstraint(e_off_->xv() == b);
  auto result = spp_.Solve(true);
  ASSERT_TRUE(result.is_success());
  EXPECT_TRUE(CompareMatrices(target_->GetSolution(result), b, 1e-6));
  EXPECT_TRUE(CompareMatrices(sink_->GetSolution(result), 0 * b, 1e-6));
}

TEST_F(ThreeBoxes, LinearEqualityConstraint2) {
  Matrix<double, 2, 4> Aeq;
  // clang-format off
  Aeq << 4.3, .5, -.4, 1.2,
       0.1, .2, -2., -.34;
  // clang-format on
  const Vector2d beq{.5, .3};
  auto constraint =
      std::make_shared<solvers::LinearEqualityConstraint>(Aeq, beq);
  e_on_->AddConstraint(
      solvers::Binding(constraint, {e_on_->xu(), e_on_->xv()}));
  e_off_->AddConstraint(
      solvers::Binding(constraint, {e_off_->xu(), e_off_->xv()}));
  auto result = spp_.Solve(true);
  ASSERT_TRUE(result.is_success());
  EXPECT_TRUE(
      CompareMatrices(Aeq.leftCols(2) * source_->GetSolution(result) +
                          Aeq.rightCols(2) * target_->GetSolution(result),
                      beq, 1e-6));
  EXPECT_TRUE(
      CompareMatrices(sink_->GetSolution(result), Vector2d::Zero(), 1e-6));
}

TEST_F(ThreeBoxes, LinearConstraint) {
  const Vector2d b{.5, .3};
  e_on_->AddConstraint(e_on_->xv() >= b);
  e_off_->AddConstraint(e_off_->xv() >= b);
  auto result = spp_.Solve(true);
  ASSERT_TRUE(result.is_success());
  EXPECT_TRUE((target_->GetSolution(result).array() >= b.array() - 1e-6).all());
  EXPECT_TRUE(
      CompareMatrices(sink_->GetSolution(result), Vector2d::Zero(), 1e-6));
}

TEST_F(ThreeBoxes, LinearConstraint2) {
  Matrix<double, 2, 4> A;
  // clang-format off
  A << 4.3, .5, -.4, 1.2,
       0.1, .2, -2., -.34;
  // clang-format on
  const Vector2d lb{.5, .3}, ub{1.0, .4};
  auto constraint = std::make_shared<solvers::LinearConstraint>(A, lb, ub);
  e_on_->AddConstraint(
      solvers::Binding(constraint, {e_on_->xu(), e_on_->xv()}));
  e_off_->AddConstraint(
      solvers::Binding(constraint, {e_off_->xu(), e_off_->xv()}));
  auto result = spp_.Solve(true);
  ASSERT_TRUE(result.is_success());
  EXPECT_TRUE(((A.leftCols(2) * source_->GetSolution(result) +
                A.rightCols(2) * target_->GetSolution(result))
                   .array() <= ub.array() + 1e-6)
                  .all());
  EXPECT_TRUE(((A.leftCols(2) * source_->GetSolution(result) +
                A.rightCols(2) * target_->GetSolution(result))
                   .array() >= lb.array() - 1e-6)
                  .all());
  EXPECT_TRUE(
      CompareMatrices(sink_->GetSolution(result), Vector2d::Zero(), 1e-6));

  // Confirm that my linear constraint resulted in a non-trivial solution.
  EXPECT_FALSE(
      CompareMatrices(source_->GetSolution(result), Vector2d::Zero(), 1e-6));
  EXPECT_FALSE(
      CompareMatrices(target_->GetSolution(result), Vector2d::Zero(), 1e-6));
}

// A simple shortest-path problem where there are no continuous variables.  The
// ShortestPathProblem class should still solve the problem, and the convex
// relaxation should be optimal.
GTEST_TEST(ShortestPathTest, ClassicalShortestPath) {
  ShortestPathProblem spp;

  for (int i = 0; i < 5; ++i) {
    spp.AddVertex(Point(Vector1d{0.0}));
  }

  spp.AddEdge(0, 1)->AddCost(3.0);
  spp.AddEdge(1, 0)->AddCost(1.0);
  spp.AddEdge(0, 2)->AddCost(4.0);
  spp.AddEdge(1, 2)->AddCost(1.0);
  spp.AddEdge(0, 3)->AddCost(1.0);
  spp.AddEdge(3, 2)->AddCost(1.0);
  spp.AddEdge(1, 4)->AddCost(2.5);  // Updated from original to break symmetry.
  spp.AddEdge(2, 4)->AddCost(3.0);
  spp.AddEdge(0, 4)->AddCost(6.0);

  spp.set_source(0);
  spp.set_target(4);

  auto result = spp.Solve();
  ASSERT_TRUE(result.is_success());

  Vector<double, 9> expected_edge_costs;
  expected_edge_costs << 0, 0, 0, 0, 1, 1, 0, 3, 0;
  for (int i = 0; i < static_cast<int>(spp.edges().size()); ++i) {
    const auto& e = spp.edges()[i];
    EXPECT_NEAR(e->GetSolutionCost(result), expected_edge_costs[i], 1e-6);
  }
}

// Example/Figure ?? from the paper.
GTEST_TEST(ShortestPathTest, TobiasToyExample) {
  ShortestPathProblem spp;

  Vertex* source = spp.AddVertex(Point(Vector2d(0, 0)), "source");
  Eigen::MatrixXd vertices(2, 4);
  // clang-format off
  vertices << 1,  1,  3,  3,
              0, -2, -2, -1;
  // clang-format on
  Vertex* p1 = spp.AddVertex(VPolytope(vertices), "p1");
  // clang-format off
  vertices <<  4,  5,  3,  2,
              -2, -4, -4, -3;
  // clang-format on
  Vertex* p2 = spp.AddVertex(VPolytope(vertices), "p2");
  vertices.resize(2, 5);
  // clang-format off
  vertices <<  2, 1, 2, 4, 4,
               2, 3, 4, 4, 3;
  // clang-format on
  Vertex* p3 = spp.AddVertex(VPolytope(vertices), "p3");
  Vertex* e1 =
      spp.AddVertex(Hyperellipsoid(Matrix2d::Identity(), Vector2d(4, 1)), "e1");
  Vertex* e2 = spp.AddVertex(
      Hyperellipsoid(Matrix2d(Vector2d(.25, 1).asDiagonal()), Vector2d(7, -2)),
      "e2");
  // clang-format off
  vertices.resize(2, 3);
  vertices <<  5, 7, 6,
               4, 4, 3;
  // clang-format on
  Vertex* p4 = spp.AddVertex(VPolytope(vertices), "p4");
  vertices.resize(2, 4);
  // clang-format off
  vertices <<  7, 8, 9, 8,
               2, 2, 3, 4;
  // clang-format on
  Vertex* p5 = spp.AddVertex(VPolytope(vertices), "p5");
  Vertex* target = spp.AddVertex(Point(Vector2d(9, 0)), "target");

  spp.set_source(*source);
  spp.set_target(*target);

  spp.AddEdge(*source, *p1);
  spp.AddEdge(*source, *p2);
  spp.AddEdge(*source, *p3);
  spp.AddEdge(*p1, *e2);
  spp.AddEdge(*p2, *p3);
  spp.AddEdge(*p2, *e1);
  spp.AddEdge(*p2, *e2);
  spp.AddEdge(*p3, *p2);  // removing this changes the asymptotic behavior.
  spp.AddEdge(*p3, *e1);
  spp.AddEdge(*p3, *p4);
  spp.AddEdge(*e1, *e2);
  spp.AddEdge(*e1, *p4);
  spp.AddEdge(*e1, *p5);
  spp.AddEdge(*e2, *e1);
  spp.AddEdge(*e2, *p5);
  spp.AddEdge(*e2, *target);
  spp.AddEdge(*p4, *p3);
  spp.AddEdge(*p4, *e2);
  spp.AddEdge(*p4, *p5);
  spp.AddEdge(*p4, *target);
  spp.AddEdge(*p5, *e1);
  spp.AddEdge(*p5, *target);

  // |xu - xv|₂
  Matrix<double, 2, 4> A;
  A.leftCols(2) = Matrix2d::Identity();
  A.rightCols(2) = -Matrix2d::Identity();
  auto cost = std::make_shared<solvers::L2NormCost>(A, Vector2d::Zero());
  for (const auto& e : spp.edges()) {
    e->AddCost(solvers::Binding(cost, {e->xu(), e->xv()}));
  }

  auto result = spp.Solve();
  ASSERT_TRUE(result.is_success());

  // TODO(russt): Test that the values are optimal.
  for (const auto& e : spp.edges()) {
    std::cout << "Edge " << e->name() << " from " << e->u().name() << " to "
              << e->v().name() << " has cost " << e->GetSolutionCost(result)
              << std::endl;
  }
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
