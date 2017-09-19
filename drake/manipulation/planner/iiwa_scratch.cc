#include <random>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/common/text_logging.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/manipulation/planner/kinematic_trajectory_optimization.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/linear_system.h"
#include "drake/systems/primitives/signal_logger.h"
#include "drake/systems/primitives/trajectory_source.h"

DEFINE_double(duration, 1, "Duration of trajectory.");
DEFINE_double(spatial_velocity_weight, 1,
              "Relative weight of end-effector spatial velocity cost");
DEFINE_double(orientation_tolerance, 0.0, "Orientation tolerance (degrees)");
DEFINE_double(position_tolerance, 0.0, "Position tolerance");
DEFINE_double(realtime_rate, 1.0, "Playback speed relative to real-time");
DEFINE_double(collision_avoidance_threshold, 0.05, "Minimum distance to obstacles at knot points");
DEFINE_string(initial_ee_position, "0.5 0.5 0.5", "Initial end-effector position");
DEFINE_string(final_ee_position, "0.5 -0.5 0.5", "Final end-effector position");
DEFINE_string(initial_ee_orientation, "0.0 0.0 0.0", "Initial end-effector orientation (RPY in degrees)");
DEFINE_string(final_ee_orientation, "0.0 0.0 0.0", "Final end-effector position (RPY in degrees)");
DEFINE_string(obstacle_position, "0.5 0.0 0.0", "Dimensions of obstacle (m)");
DEFINE_string(obstacle_size, "0.2 0.2 0.5", "Dimensions of obstacle (m)");
DEFINE_bool(animate_with_zoh, false, "If true, use a zero-order hold to display trajectory");
DEFINE_bool(loop_animation, true, "If true, repeat playback indefinitely");

using drake::solvers::SolutionResult;
using drake::systems::trajectory_optimization::MultipleShooting;
using drake::systems::Diagram;
using drake::systems::DiagramBuilder;
using drake::systems::SignalLogger;
using drake::systems::TrajectorySource;
using drake::systems::DrakeVisualizer;
using drake::systems::Simulator;

namespace drake {
namespace manipulation {
namespace planner {
int DoMain() {
  const std::string kModelPath = FindResourceOrThrow(
      "drake/manipulation/models/iiwa_description/urdf/"
      "iiwa14_mesh_collision.urdf");
  auto iiwa = std::make_unique<RigidBodyTree<double>>();
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      kModelPath, multibody::joints::kFixed, nullptr, iiwa.get());
  drake::multibody::AddFlatTerrainToWorld(iiwa.get());

  // Add an obstacle to the world
  std::istringstream iss_obstacle_position{FLAGS_obstacle_position};
  std::istringstream iss_obstacle_size{FLAGS_obstacle_size};
  Vector3<double> obstacle_size;
  Isometry3<double> X_WO{Isometry3<double>::Identity()};
  for (int i = 0; i < 3; ++i) {
    iss_obstacle_size >> obstacle_size(i);
    DRAKE_THROW_UNLESS(!iss_obstacle_size.fail());
    iss_obstacle_position >> X_WO.translation()(i);
    DRAKE_THROW_UNLESS(!iss_obstacle_position.fail());
  }
  // Move the obstacle up to sit on the ground plane
  X_WO.translation().z() += obstacle_size.z() / 2;
  DrakeShapes::Box geom(obstacle_size);
  RigidBody<double>& world = iiwa->world();
  Eigen::Vector4d color;
  color << 0.5, 0.5, 0.5, 1;
  world.AddVisualElement(DrakeShapes::VisualElement(geom, X_WO, color));
  iiwa->addCollisionElement(
      drake::multibody::collision::Element(geom, X_WO, &world), world,
      "terrain");
  iiwa->compile();

  const int kNumKnots = std::ceil(FLAGS_duration / 0.1) + 1;
  drake::log()->info("Number of knots: {}", kNumKnots);

  KinematicTrajectoryOptimization kin_traj_opt{std::move(iiwa), kNumKnots};
  MultipleShooting* prog = kin_traj_opt.mutable_prog();
  prog->SetSolverOption(drake::solvers::SnoptSolver::id(),
                        "Major iterations limit", 1e5);

  const int kNumPositions = kin_traj_opt.tree().get_num_positions();

  // v[0] = 0.
  VectorX<double> v0{VectorX<double>::Zero(kin_traj_opt.num_velocities())};
  prog->AddLinearConstraint(
      prog->initial_state().tail(kin_traj_opt.num_velocities()) == v0);

  // v[tf] = 0.
  VectorX<double> vf{VectorX<double>::Zero(kin_traj_opt.num_velocities())};
  prog->AddLinearConstraint(
      prog->final_state().tail(kin_traj_opt.num_velocities()) == vf);

  kin_traj_opt.AddRunningCost(kin_traj_opt.input().transpose() *
                              kin_traj_opt.input());
  kin_traj_opt.AddSpatialVelocityCost("iiwa_link_ee",
                                      FLAGS_spatial_velocity_weight);

  // Add initial and final pose constraints
  Isometry3<double> X_WF0{Isometry3<double>::Identity()};
  Isometry3<double> X_WFf{Isometry3<double>::Identity()};
  Vector3<double> rpy_WF0;
  Vector3<double> rpy_WFf;
  std::istringstream iss_initial_ee_position{FLAGS_initial_ee_position};
  std::istringstream iss_final_ee_position{FLAGS_final_ee_position};
  std::istringstream iss_initial_ee_orientation{FLAGS_initial_ee_orientation};
  std::istringstream iss_final_ee_orientation{FLAGS_final_ee_orientation};
  for (int i = 0; i < 3; ++i) {
    iss_initial_ee_position >> X_WF0.translation()(i);
    DRAKE_THROW_UNLESS(!iss_initial_ee_position.fail());
    iss_final_ee_position >> X_WFf.translation()(i);
    DRAKE_THROW_UNLESS(!iss_final_ee_position.fail());
    iss_initial_ee_orientation >> rpy_WF0(i);
    DRAKE_THROW_UNLESS(!iss_initial_ee_orientation.fail());
    iss_final_ee_orientation >> rpy_WFf(i);
    DRAKE_THROW_UNLESS(!iss_final_ee_orientation.fail());
  }
  X_WF0.linear() = drake::math::rpy2rotmat(M_PI/180*rpy_WF0);
  X_WFf.linear() = drake::math::rpy2rotmat(M_PI/180*rpy_WFf);

  const double kOrientationTolerance{FLAGS_orientation_tolerance*M_PI/180};

  kin_traj_opt.AddBodyPoseConstraint(0, "iiwa_link_ee", X_WF0,
                                     kOrientationTolerance,
                                     FLAGS_position_tolerance);
  kin_traj_opt.AddBodyPoseConstraint(kNumKnots - 1, "iiwa_link_ee", X_WFf,
                                     kOrientationTolerance,
                                     FLAGS_position_tolerance);
  // Add collision avoidance constraints
  kin_traj_opt.AddCollisionAvoidanceConstraint(
      FLAGS_collision_avoidance_threshold);

  SolutionResult result{prog->Solve()};
  drake::log()->info("Solver returns {}.", result);

  auto x_sol = prog->ReconstructStateTrajectory();
  std::vector<double> breaks{
      x_sol.get_piecewise_polynomial().getSegmentTimes()};
  std::vector<MatrixX<double>> knots;
  std::vector<MatrixX<double>> knots_dot;
  knots.reserve(breaks.size());
  knots_dot.reserve(breaks.size());
  drake::log()->info("Number of knots in solution: {}", breaks.size());
  for (int i = 0; i < static_cast<int>(breaks.size()); ++i) {
    const double t{breaks[i]};
    knots.push_back(x_sol.value(t).topRows(2 * kNumPositions));
    knots_dot.push_back(FLAGS_realtime_rate * x_sol.value(t).bottomRows(2 * kNumPositions));
    breaks[i] /= FLAGS_realtime_rate;
  }
  std::unique_ptr<PiecewisePolynomialTrajectory> q_and_v_traj;
  if (FLAGS_animate_with_zoh) {
    q_and_v_traj = std::make_unique<PiecewisePolynomialTrajectory>(
        PiecewisePolynomial<double>::ZeroOrderHold(breaks, knots));
  } else {
    q_and_v_traj = std::make_unique<PiecewisePolynomialTrajectory>(
        PiecewisePolynomial<double>::Cubic(breaks, knots, knots_dot));
  }

  lcm::DrakeLcm lcm;
  lcm.StartReceiveThread();
  DrakeVisualizer visualizer{kin_traj_opt.tree(), &lcm, true};
  while (true) {
    visualizer.PlaybackTrajectory(q_and_v_traj->get_piecewise_polynomial());
  }

  //DiagramBuilder<double> builder;

  // auto logger = builder.AddSystem<SignalLogger<double>>(
  // iiwa.get_num_positions() + iiwa.get_num_velocities());

  //auto trajectory_source =
      //builder.AddSystem<TrajectorySource<double>>(*q_and_v_traj);
  // builder.Connect(trajectory_source->get_output_port(),
  // logger->get_input_port(0));

  //auto visualizer = builder.AddSystem<DrakeVisualizer>(kin_traj_opt.tree(), &lcm, true);

  //auto gain = builder.AddSystem<systems::LinearSystem<double>>(
      //MatrixX<double>::Zero(0, 0), MatrixX<double>::Zero(0, 2 * kNumPositions),
      //MatrixX<double>::Zero(2 * kNumPositions, 0),
      //MatrixX<double>::Identity(2 * kNumPositions, 2 * kNumPositions), 0.02);
  //builder.Connect(trajectory_source->get_output_port(), gain->get_input_port());
  //builder.Connect(gain->get_output_port(), visualizer->get_input_port(0));
  //drake::log()->debug("Added visualizer.");

  //auto sys = builder.Build();

  //Simulator<double> simulator{*sys};
  //simulator.set_target_realtime_rate(FLAGS_realtime_rate);
  //simulator.Initialize();

  //drake::log()->info("Stepping to t = {} s.", x_sol.get_end_time());
  //simulator.StepTo(x_sol.get_end_time());
  //while (true) {
    //visualizer->ReplayCachedSimulation();
  //}
  return result;
}
}  // namespace drake
}  // namespace manipulation
}  // namespace planner

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();
  return drake::manipulation::planner::DoMain();
}
