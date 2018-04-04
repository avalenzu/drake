#include <limits>

#include <gflags/gflags.h>

#include "drake/common/text_logging.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/manipulation/util/model_tree/protobuf_converter.h"
#include "drake/manipulation/util/sim_diagram_builder.h"
#include "drake/manipulation/util/world_sim_tree_builder.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/primitives/constant_vector_source.h"

using drake::manipulation::util::SimDiagramBuilder;
using drake::manipulation::util::WorldSimTreeBuilder;
using drake::manipulation::util::model_tree::ProtobufConverter;
using drake::systems::RigidBodyPlant;
using drake::systems::Simulator;
using drake::systems::ConstantVectorSource;

namespace drake {
namespace manipulation {
namespace util {
namespace {

DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(),
              "Number of seconds to simulate.");
DEFINE_double(target_realtime_rate, 1.0,
              "Playback speed.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");
DEFINE_string(model_tree_file, "",
              "Path (absolute or Drake resource) to .model_tree file.");
int DoMain() {
  // Get the flag values.
  const std::string& model_tree_file = FLAGS_model_tree_file;
  const double& simulation_sec = FLAGS_simulation_sec;
  const double& target_realtime_rate = FLAGS_target_realtime_rate;

  if (model_tree_file.empty()) {
    drake::log()->error("You must specify --model_tree_file=<filename>");
    return 1;
  }
  WorldSimTreeBuilder<double> tree_builder{};
  tree_builder.AddGround();
  ProtobufConverter protobuf_converter{};
  tree_builder.AddModelInstancesFromModelTree(
      protobuf_converter.ParseModelTreeFromFile(model_tree_file));

  SimDiagramBuilder<double> sim_diagram_builder;
  sim_diagram_builder.AddPlant(tree_builder.Build());
  drake::lcm::DrakeLcm lcm{};
  sim_diagram_builder.AddVisualizer(&lcm);
  auto plant = sim_diagram_builder.get_plant();
  const int num_model_instances = plant->get_num_model_instances();
  for (int i = 0; i < num_model_instances; ++i) {
    if (plant->model_instance_has_actuators(i)) {
      const int num_actuators =
          plant->model_instance_actuator_command_input_port(i).size();
      auto zero_source = sim_diagram_builder.get_mutable_builder()
                             ->AddSystem<ConstantVectorSource<double>>(
                                 VectorX<double>::Zero(num_actuators));
      sim_diagram_builder.get_mutable_builder()->Connect(
          zero_source->get_output_port(),
          plant->model_instance_actuator_command_input_port(i));
    }
  }

  auto sys = sim_diagram_builder.Build();

  Simulator<double> simulator{*sys};
  simulator.set_target_realtime_rate(target_realtime_rate);

  lcm.StartReceiveThread();
  simulator.StepTo(simulation_sec);

  return 0;
}
}  // namespace
}  // namespace util
}  // namespace manipulation
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();
  return drake::manipulation::util::DoMain();
}
