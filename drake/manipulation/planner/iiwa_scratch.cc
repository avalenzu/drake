#include <random>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/common/text_logging.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/manipulation/planner/kinematic_trajectory_optimization.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/linear_system.h"
#include "drake/systems/primitives/signal_logger.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/systems/analysis/simulator.h"

DEFINE_double(duration, 1, "Duration of trajectory.");

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
        "iiwa14_polytope_collision.urdf");
    RigidBodyTree<double> iiwa{};
    drake::parsers::urdf::AddModelInstanceFromUrdfFile(
        kModelPath, multibody::joints::kFixed, nullptr, &iiwa);

    const int kNumKnots = std::ceil(FLAGS_duration / 0.1) + 1;
    drake::log()->info("Number of knots: {}", kNumKnots);

    KinematicTrajectoryOptimization kin_traj_opt{iiwa, kNumKnots};
    MultipleShooting* prog = kin_traj_opt.mutable_prog();

    const int kNumPositions = iiwa.get_num_positions();

    // x[0] = 0.
    VectorX<double> x0{VectorX<double>::Zero(prog->initial_state().size())};
    prog->AddLinearConstraint(prog->initial_state() == x0);

    // x[t_final] = random
    std::default_random_engine rand_generator{1234};
    VectorX<double> xf{VectorX<double>::Zero(prog->initial_state().size())};
    xf.head(kNumPositions) = iiwa.getRandomConfiguration(rand_generator);
    drake::log()->debug("xf = [{}]", xf.transpose());
    prog->AddLinearConstraint(prog->final_state() == xf);

    SolutionResult result{prog->Solve()};
    drake::log()->info("Solver returns {}.", result);

    auto x_sol = prog->ReconstructStateTrajectory();
    const std::vector<double> breaks{x_sol.get_piecewise_polynomial().getSegmentTimes()};
    std::vector<MatrixX<double>> knots;
    std::vector<MatrixX<double>> knots_dot;
    knots.reserve(breaks.size());
    knots_dot.reserve(breaks.size());
    for (double t : breaks) {
      drake::log()->debug("t = {} s, x = [{}]", t, x_sol.value(t).transpose()); 
      knots.push_back(x_sol.value(t).topRows(2*kNumPositions));
      knots_dot.push_back(x_sol.value(t).bottomRows(2*kNumPositions));
    }
    PiecewisePolynomialTrajectory q_and_v_traj{
        PiecewisePolynomial<double>::Cubic(breaks, knots, knots_dot)};

    DiagramBuilder<double> builder;

    //auto logger = builder.AddSystem<SignalLogger<double>>(
        //iiwa.get_num_positions() + iiwa.get_num_velocities());

    auto trajectory_source = builder.AddSystem<TrajectorySource<double>>(q_and_v_traj);
    //builder.Connect(trajectory_source->get_output_port(),
                    //logger->get_input_port(0));

    lcm::DrakeLcm lcm;
    auto visualizer = builder.AddSystem<DrakeVisualizer>(iiwa, &lcm, true);

    auto gain = builder.AddSystem<systems::LinearSystem<double>>(
        MatrixX<double>::Zero(0, 0), MatrixX<double>::Zero(0, 2*kNumPositions),
        MatrixX<double>::Zero(2 * kNumPositions, 0),
        MatrixX<double>::Identity(2 * kNumPositions, 2 * kNumPositions), 0.05);
    builder.Connect(trajectory_source->get_output_port(),
                    gain->get_input_port());
    builder.Connect(gain->get_output_port(),
                    visualizer->get_input_port(0));
    drake::log()->debug("Added visualizer.");

    auto sys = builder.Build();

    Simulator<double> simulator{*sys};
    simulator.set_target_realtime_rate(1);
    simulator.Initialize();

    drake::log()->info("Stepping to t = {} s.", x_sol.get_end_time());
    simulator.StepTo(x_sol.get_end_time());
    while (true) {
      visualizer->ReplayCachedSimulation();
    }
    return result;
  }
} // namespace drake
} // namespace manipulation
} // namespace planner

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();
  return drake::manipulation::planner::DoMain();
}
