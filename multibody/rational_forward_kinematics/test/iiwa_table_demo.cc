#include <functional>
#include <memory>
#include <random>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/common/symbolic.h"
#include "drake/common/text_logging.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/manipulation/dev/remote_tree_viewer_wrapper.h"
#include "drake/math/bspline_basis.h"
#include "drake/math/bspline_curve.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/rational_forward_kinematics/configuration_space_collision_free_region.h"
#include "drake/multibody/rational_forward_kinematics/test/rational_forward_kinematics_test_utilities.h"
#include "drake/solvers/bspline_curve_optimization_utilities.h"
#include "drake/solvers/solve.h"

using drake::math::BsplineBasis;
using drake::math::BsplineCurve;
using drake::solvers::MathematicalProgram;
using drake::solvers::MathematicalProgramResult;
using drake::solvers::VectorXDecisionVariable;
using drake::symbolic::Expression;
using drake::symbolic::Formula;
using drake::symbolic::Variable;

DEFINE_int32(num_boxes, 3, "Number of C-space regions.");
DEFINE_int32(order, 4, "Spline order.");
DEFINE_int32(num_control_points, 16, "Number of spline control points.");
DEFINE_int32(penalized_derivative_order, 1,
             "Attempt to minimize this order of derivative.");
DEFINE_bool(clamped, false, "If true, use clamped knot vector.");
DEFINE_double(goal_sampling_probability, 0.05,
              "Fraction of the time that we sample the goal.");
DEFINE_double(binary_search_tolerance, 0.1, "");

namespace drake {
namespace multibody {
namespace {
class ConfigurationSpaceBox {
 public:
  ConfigurationSpaceBox(Eigen::VectorXd q_min, Eigen::VectorXd q_max)
      : num_positions_(q_min.size()),
        q_min_(q_min), q_max_(q_max) {
    DRAKE_ASSERT(q_max_.size() == num_positions_);
    DRAKE_ASSERT(((q_max_ - q_min_).array() >= 0).all());
  }

  ConfigurationSpaceBox(Eigen::VectorXd q_mid, double rho)
      : ConfigurationSpaceBox(
            q_mid - Eigen::VectorXd::Constant(q_mid.size(), rho),
            q_mid + Eigen::VectorXd::Constant(q_mid.size(), rho)) {}

  const Eigen::VectorXd& lower_bounds() const {
    return q_min_;
  }

  const Eigen::VectorXd& upper_bounds() const {
    return q_max_;
  }

  const Eigen::VectorXd q_mid() const {
    return 0.5 * (lower_bounds() + upper_bounds());
  }

  template <typename Generator>
  Eigen::VectorXd Sample(Generator* generator) {
    DRAKE_DEMAND(generator);
    std::uniform_real_distribution<double> distribution{};
    Eigen::VectorXd sample{q_min_};
    for (int j = 0; j < num_positions_; ++j) {
      sample(j) += (q_max_(j) - q_min_(j)) * distribution(*generator);
    }
    return sample;
  }

  Formula ToFormula(const VectorXDecisionVariable& x,
                    const Variable& indicator) const {
    const int n = x.size();
    // clang-format off
    const auto A =
        (MatrixX<double>(2 * n, n) <<
          MatrixX<double>::Identity(n, n),
         -MatrixX<double>::Identity(n, n))
        .finished();
    const auto b =
        (MatrixX<double>(2 * n, 1) <<
          upper_bounds(),
         -lower_bounds())
        .finished();
    // clang-format on
    return {A * x <= indicator * b};
  }

 private:
  const int num_positions_{};
  Eigen::VectorXd q_min_;
  Eigen::VectorXd q_max_;
};
}  // namespace

int DoMain() {
  // weld the schunk gripper to iiwa link 7.
  Eigen::Isometry3d X_7S =
      Eigen::Translation3d(0, 0, 0.1) *
      Eigen::AngleAxisd(-21.0 / 180 * M_PI, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX()) *
      Eigen::Isometry3d::Identity();
  auto plant = std::make_unique<MultibodyPlant<double>>();
  auto scene_graph = std::make_unique<geometry::SceneGraph<double>>();
  plant->RegisterAsSourceForSceneGraph(scene_graph.get());
  AddIiwaWithSchunk(X_7S, plant.get());
  plant->Finalize(scene_graph.get());
  auto plant_collision = std::make_unique<MultibodyPlant<double>>();
  auto scene_graph_collision = std::make_unique<geometry::SceneGraph<double>>();
  plant_collision->RegisterAsSourceForSceneGraph(scene_graph_collision.get());
  AddIiwaWithSchunk(X_7S, plant_collision.get());

  MultibodyPlantVisualizer visualizer(*plant, std::move(scene_graph));
  Eigen::Matrix<double, 7, 1> q_start =
      (Eigen::Matrix<double, 7, 1>() << 60, 0, 0, -90, 0, 0, 0).finished() *
      M_PI / 180.;
  Eigen::Matrix<double, 7, 1> q_goal =
      (Eigen::Matrix<double, 7, 1>() << -60, 45, 0, -90, 0, 0, 0).finished() *
      M_PI / 180.;
  Eigen::Matrix<double, 7, 1> q = q_start;
  // This is only for visualizing a sampled configuration in the verified
  // collision free box in the configuration space.
  // double delta = 0.271484;
  // q(0) -= delta;
  // q(1) += delta;
  // q(2) -= delta;
  // q(3) -= delta;
  // q(4) -= delta;
  // q(5) += delta;

  // Now add the link points to represent collision.
  auto context = plant->CreateDefaultContext();
  plant->SetPositions(context.get(), q);
  manipulation::dev::RemoteTreeViewerWrapper viewer;

  std::array<BodyIndex, 8> iiwa_link;
  for (int i = 0; i < 8; ++i) {
    iiwa_link[i] =
        plant->GetBodyByName("iiwa_link_" + std::to_string(i)).index();
  }
  // Schunk gripper points.
  Eigen::Matrix<double, 3, 8> p_SV;
  p_SV.col(0) << -0.065, -0.035, 0.02;
  p_SV.col(1) << 0.065, -0.035, 0.02;
  p_SV.col(2) << -0.065, -0.035, -0.02;
  p_SV.col(3) << 0.065, -0.035, -0.02;
  p_SV.col(4) << -0.065, 0.105, 0.02;
  p_SV.col(5) << 0.065, 0.105, 0.02;
  p_SV.col(6) << -0.065, 0.105, -0.02;
  p_SV.col(7) << 0.065, 0.105, -0.02;
  Eigen::Matrix<double, 3, 8> p_7V = X_7S * p_SV;
  for (int i = 0; i < p_7V.cols(); ++i) {
    VisualizeBodyPoint(&viewer, *plant, *context, iiwa_link[7], p_7V.col(i),
                       0.01, {1, 0, 0, 1}, "gripper_point" + std::to_string(i));
  }
  std::vector<std::shared_ptr<const ConvexPolytope>> link_polytopes;

  auto add_link_collision_polytope = [&link_polytopes](
      MultibodyPlant<double>* plant_collision_in,
      geometry::SceneGraph<double>* scene_graph_collision_in,
      BodyIndex body_index, const Eigen::Ref<const Eigen::Matrix3Xd>& p_BV,
      const std::string& name) {
    // The proximity engine does not support mesh yet. So we add the vertices as
    // tiny spheres to the collision geometry. If these spheres are in collision
    // with the environment, we know the robot has to be in collision.
    geometry::Sphere vertex_sphere(1e-6);
    for (int i = 0; i < p_BV.cols(); ++i) {
      Eigen::Isometry3d X_BS = Eigen::Isometry3d::Identity();
      X_BS.translation() = p_BV.col(i);
      plant_collision_in->RegisterCollisionGeometry(
          plant_collision_in->get_body(body_index), X_BS, vertex_sphere,
          name + std::to_string(i), CoulombFriction<double>(1.0, 1.0),
          scene_graph_collision_in);
    }
    link_polytopes.push_back(
        std::make_shared<ConvexPolytope>(body_index, p_BV));
  };

  add_link_collision_polytope(plant_collision.get(),
                              scene_graph_collision.get(), iiwa_link[7], p_7V,
                              "schunk");

  // iiwa_link6 points.
  Eigen::Matrix<double, 3, 14> p_6V;
  p_6V.col(0) << 0.03, -0.05, 0.06;
  p_6V.col(1) << -0.03, -0.05, 0.06;
  p_6V.col(2) << 0.04, -0.09, 0.02;
  p_6V.col(3) << -0.04, -0.09, 0.02;
  p_6V.col(4) << -0.03, -0.05, -0.09;
  p_6V.col(5) << 0.03, -0.05, -0.09;
  p_6V.col(6) << 0.03, 0.05, -0.09;
  p_6V.col(7) << -0.03, 0.05, -0.09;
  p_6V.col(8) << -0.07, 0, 0;
  p_6V.col(9) << 0.07, 0, 0;
  p_6V.col(10) << 0.03, 0.12, 0.03;
  p_6V.col(11) << 0.03, 0.12, -0.03;
  p_6V.col(12) << -0.03, 0.12, 0.03;
  p_6V.col(13) << -0.03, 0.12, -0.03;
  for (int i = 0; i < p_6V.cols(); ++i) {
    VisualizeBodyPoint(&viewer, *plant, *context, iiwa_link[6], p_6V.col(i),
                       0.01, {1, 0, 0, 1}, "link6_pt" + std::to_string(i));
  }
  add_link_collision_polytope(plant_collision.get(),
                              scene_graph_collision.get(), iiwa_link[6], p_6V,
                              "link6_pt");

  Eigen::Matrix<double, 3, 10> p_5V1;
  p_5V1.col(0) << 0.05, 0.07, -0.05;
  p_5V1.col(1) << 0.05, -0.05, -0.05;
  p_5V1.col(2) << -0.05, 0.07, -0.05;
  p_5V1.col(3) << -0.05, -0.05, -0.05;
  p_5V1.col(4) << 0.05, 0.05, 0.08;
  p_5V1.col(5) << 0.05, -0.05, 0.07;
  p_5V1.col(6) << -0.05, 0.05, 0.08;
  p_5V1.col(7) << -0.05, -0.05, 0.07;
  p_5V1.col(8) << 0.04, 0.08, 0.15;
  p_5V1.col(9) << -0.04, 0.08, 0.15;
  for (int i = 0; i < p_5V1.cols(); ++i) {
    VisualizeBodyPoint(&viewer, *plant, *context, iiwa_link[5], p_5V1.col(i),
                       0.01, {1, 0, 0, 1}, "link5_V1_pt" + std::to_string(i));
  }
  add_link_collision_polytope(plant_collision.get(),
                              scene_graph_collision.get(), iiwa_link[5], p_5V1,
                              "link5_V1_pt");

  Eigen::Matrix<double, 3, 8> p_4V;
  p_4V.col(0) << 0.04, -0.02, 0.11;
  p_4V.col(1) << -0.04, -0.02, 0.11;
  p_4V.col(2) << 0.06, -0.05, 0;
  p_4V.col(3) << -0.06, -0.05, 0;
  p_4V.col(4) << 0.06, 0.12, 0;
  p_4V.col(5) << -0.06, 0.12, 0;
  p_4V.col(6) << 0.05, 0.12, 0.09;
  p_4V.col(7) << -0.05, 0.12, 0.09;
  for (int i = 0; i < p_4V.cols(); ++i) {
    VisualizeBodyPoint(&viewer, *plant, *context, iiwa_link[4], p_4V.col(i),
                       0.01, {1, 0, 0, 1}, "link4_pt" + std::to_string(i));
  }
  add_link_collision_polytope(plant_collision.get(),
                              scene_graph_collision.get(), iiwa_link[4], p_4V,
                              "link4_pt");

  std::vector<std::shared_ptr<const ConvexPolytope>> obstacles;
  auto add_box_to_obstacle = [&obstacles](
      MultibodyPlant<double>* plant_collision_in,
      geometry::SceneGraph<double>* scene_graph_collision_in,
      const Eigen::Isometry3d& X_WB, const Eigen::Vector3d box_size,
      const std::string& name) {
    plant_collision_in->RegisterCollisionGeometry(
        plant_collision_in->world_body(), X_WB,
        geometry::Box(box_size(0), box_size(1), box_size(2)), name,
        CoulombFriction<double>(1, 1), scene_graph_collision_in);
    obstacles.push_back(std::make_shared<ConvexPolytope>(
        plant_collision_in->world_body().index(),
        GenerateBoxVertices(box_size, X_WB)));
  };
  // Add a table.
  Eigen::Isometry3d X_WT = Eigen::Isometry3d::Identity();
  X_WT.translation() << 0.5, 0, 0.25;
  Eigen::Vector3d table_size(0.3, 0.6, 0.5);
  viewer.PublishGeometry(DrakeShapes::Box(table_size), X_WT, {0, 0, 1, 1},
                         {"table"});

  add_box_to_obstacle(plant_collision.get(), scene_graph_collision.get(), X_WT,
                      table_size, "table");


  plant_collision->Finalize(scene_graph_collision.get());

  ConfigurationSpaceCollisionFreeRegion dut(*plant, link_polytopes, obstacles,
                                            SeparatingPlaneOrder::kAffine);

  std::mt19937_64 generator{1234};
  std::vector<ConfigurationSpaceBox> boxes{};
  ConfigurationSpaceBox full_configuration_space{
      plant_collision->GetPositionLowerLimits(),
      plant_collision->GetPositionUpperLimits()};
  double goal_sampling_probability = FLAGS_goal_sampling_probability;
  auto sampling_function = [&q_goal, &goal_sampling_probability,
                            &full_configuration_space,
                            &generator]() -> Eigen::VectorXd {
    std::uniform_real_distribution<double> uniform_zero_to_one{0.0, 1.0};
    if (uniform_zero_to_one(generator) < goal_sampling_probability) {
      return q_goal;
    } else {
      return full_configuration_space.Sample(&generator);
    }
  };
  const int num_boxes{FLAGS_num_boxes};
  std::vector<Formula> regions;
  VectorXDecisionVariable q_placeholder(q.size());
  for (int i = 0; i < q.size(); ++i) {
    q_placeholder(i) = Variable(fmt::format("q_placeholder_{}", i));
  }
  Variable indicator_placeholder("b");
  while (static_cast<int>(boxes.size()) < num_boxes) {
    visualizer.VisualizePosture(q);
    double rho = dut.FindLargestBoxThroughBinarySearch(
        q, {}, Eigen::VectorXd::Constant(7, -1),
        Eigen::VectorXd::Constant(7, 1), 0, 1.0, FLAGS_binary_search_tolerance);
    drake::log()->info("q_mid = {}", q.transpose());
    drake::log()->info("rho = {}, corresponding to angle (deg): {}\n", rho,
                       rho / M_PI * 180.0);
    if (rho > 0) {
      plant->SetPositions(context.get(), q);
      VisualizeBodyPoint(&viewer, *plant, *context, iiwa_link[7],
                         p_7V.rowwise().mean(), 0.1 * rho, {0, 1, 1, 1},
                         "gripper_mean" + std::to_string(boxes.size()));
      boxes.emplace_back(q, rho);
      regions.push_back(
          boxes.back().ToFormula(q_placeholder, indicator_placeholder));
      if (solvers::ClosestPointInRegions({regions.back()}, q_goal,
                                         q_placeholder, indicator_placeholder)
              .isApprox(q_goal)) {
        // In this case, the goal is contained in the existing regions. We're
        // done!
        break;
      }
    }

    bool found_new_q = false;
    while (!found_new_q) {
      Eigen::VectorXd q_sample = sampling_function();
      drake::log()->info("q_sample = {}", q_sample.transpose());

      Eigen::VectorXd q_nearest = solvers::ClosestPointInRegions(
          regions, q_sample, q_placeholder, indicator_placeholder);
      // If sample point is outside the existing regions, then q_nearest is the
      // new q.
      found_new_q = (q_sample - q_nearest).squaredNorm() >
                    std::numeric_limits<double>::epsilon();
      drake::log()->info("q_nearest = {}", q_nearest.transpose());
      if (found_new_q) {
        q = q_nearest;
      } else if (q_sample.isApprox(q_goal)) {
        // In this case, we sampled the goal, and it was contained in the
        // existing regions. We're done!
        break;
      }
    }
  }

  std::transform(boxes.begin(), boxes.end(), std::back_inserter(regions),
                 [&q_placeholder,
                  &indicator_placeholder](const ConfigurationSpaceBox& box) {
                   return box.ToFormula(q_placeholder, indicator_placeholder);
                 });

  // Create a mathematical program for optimizing a B-spline curve through the
  // regions.
  MathematicalProgram program;
  BsplineBasis<double> basis{FLAGS_order, FLAGS_num_control_points,
        FLAGS_clamped
        ? math::KnotVectorType::kClampedUniform
        : math::KnotVectorType::kUniform};
  BsplineCurve<Expression> q_curve_symbolic = solvers::AddCurveThroughRegions(
      regions, q_placeholder, indicator_placeholder, basis, &program);
  // Curve should start at q_start.
  program.AddLinearEqualityConstraint(q_curve_symbolic.InitialValue() ==
                                      q_start);
  // Curve should end at the closest point to q_goal.
  program.AddLinearEqualityConstraint(
      q_curve_symbolic.FinalValue() ==
      solvers::ClosestPointInRegions(regions, q_goal, q_placeholder,
                                     indicator_placeholder));
  // Curve should start and end with zero velocity.
  program.AddLinearEqualityConstraint(
      q_curve_symbolic.Derivative().InitialValue() ==
      Eigen::VectorXd::Zero(q.size()));
  program.AddLinearEqualityConstraint(
      q_curve_symbolic.Derivative().FinalValue() ==
      Eigen::VectorXd::Zero(q.size()));
  BsplineCurve<Expression> penalized_derivative_curve_symbolic =
      q_curve_symbolic.Derivative(FLAGS_penalized_derivative_order);
  for (const auto& control_point :
       penalized_derivative_curve_symbolic.control_points()) {
    program.AddQuadraticCost((control_point.transpose() * control_point)(0, 0));
  }

  drake::log()->info("Calling Solve ...");
  MathematicalProgramResult result = solvers::Solve(program);
  drake::log()->info("Done.");

  while (true) {
    visualizer.VisualizeTrajectory(
        solvers::GetSolutionCurve(result, q_curve_symbolic));
  }

  return 0;
}

}  // namespace multibody
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::multibody::DoMain();
}
