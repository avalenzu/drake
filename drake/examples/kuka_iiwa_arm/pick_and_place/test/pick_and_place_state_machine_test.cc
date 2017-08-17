#include "drake/examples/kuka_iiwa_arm/pick_and_place/pick_and_place_state_machine.h"

#include <fstream>

#include <gtest/gtest.h>
#include "bot_core/robot_state_t.hpp"
#include "robotlocomotion/robot_plan_t.hpp"

#include "drake/common/find_resource.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/examples/kuka_iiwa_arm/robot_plan_interpolator.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_schunk_wsg_command.hpp"
#include "drake/lcmt_schunk_wsg_status.hpp"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/zero_order_hold.h"
#include "drake/systems/primitives/signal_logger.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace pick_and_place {
namespace {

const char* const kIiwaUrdf =
    "drake/manipulation/models/iiwa_description/urdf/"
    "iiwa14_polytope_collision.urdf";


struct TestStep {
  int iiwa_plan_count_expected;
  int wsg_command_count_expected;
  PickAndPlaceState state_expected;
};

class PickAndPlaceInitialConditionTest
    : public ::testing::TestWithParam<
          std::tuple<double, double, double, bool>> {
 public:
  virtual void SetUp() {
    x_ += std::get<0>(GetParam());
    y_ += std::get<1>(GetParam());
    theta_ += std::get<2>(GetParam());
    visualize_ =std::get<3>(GetParam());

    systems::DiagramBuilder<double> builder;

    iiwa_ = std::make_unique<RigidBodyTree<double>>();
    parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
        FindResourceOrThrow(kIiwaUrdf), multibody::joints::kFixed, iiwa_.get());

    iiwa_trajectory_generator_ = builder.AddSystem<RobotPlanInterpolator>(
        FindResourceOrThrow(kIiwaUrdf));
    auto hold = builder.AddSystem<systems::ZeroOrderHold<double>>(
        0.1, iiwa_->get_num_positions() + iiwa_->get_num_velocities());
    logger_ = builder.AddSystem<systems::SignalLogger<double>>(
        iiwa_->get_num_positions() + iiwa_->get_num_velocities());

    builder.Connect(iiwa_trajectory_generator_->get_state_output_port(),
                    hold->get_input_port());
    builder.Connect(hold->get_output_port(),
                    iiwa_trajectory_generator_->get_state_input_port());
    builder.Connect(iiwa_trajectory_generator_->get_state_output_port(),
                    logger_->get_input_port(0));

    if (visualize()) {
      auto visualizer =
          builder.AddSystem<systems::DrakeVisualizer>(*iiwa_, &lcm_);
      builder.Connect(iiwa_trajectory_generator_->get_state_output_port(),
                      visualizer->get_input_port(0));
    }

    sys_ = builder.Build();

    simulator_ = std::make_unique<systems::Simulator<double>>(*sys_);
    if (visualize()) {
      simulator_->set_target_realtime_rate(1);
    }
    simulator_->get_mutable_integrator()->set_maximum_step_size(0.1);

    auto& plan_source_context = sys_->GetMutableSubsystemContext(
        *iiwa_trajectory_generator_, simulator_->get_mutable_context());
    iiwa_trajectory_generator_->Initialize(
        plan_source_context.get_time(), Eigen::VectorXd::Zero(7),
        plan_source_context.get_mutable_state());
  }

  void SimulatePlan(const robotlocomotion::robot_plan_t& iiwa_plan) {
    auto& plan_source_context = sys_->GetMutableSubsystemContext(
        *iiwa_trajectory_generator_, simulator_->get_mutable_context());
    plan_source_context.FixInputPort(
        0, std::make_unique<systems::Value<robotlocomotion::robot_plan_t>>(
               iiwa_plan));
    simulator_->StepTo(simulator_->get_context().get_time() +
                     iiwa_plan.plan.back().utime / 1e6 + 0.5);
  }

  Eigen::Block<const MatrixX<double>, Eigen::Dynamic, Eigen::Dynamic, true>
  joint_positions() const {
    return logger_->data();
  }

  void WriteToResultsFile(bool success) {
    if (!visualize()) {
      std::ofstream status_file{"pick_and_place_status.csv",
                                std::ofstream::app};
      Eigen::IOFormat FlattenedCsvFormat(Eigen::FullPrecision,
                                         Eigen::DontAlignCols, ", ", ", ", "",
                                         "", "", "");
      Eigen::Vector4d x_y_z_theta{x(), y(), z(), theta()};
      status_file << x_y_z_theta.format(FlattenedCsvFormat) << ", " << success
                  << '\n';
      status_file.close();
      if (success) {
        std::ofstream trajectory_file{"pick_and_place_trajectories.csv",
                                      std::ofstream::app};
        trajectory_file << x_y_z_theta.format(FlattenedCsvFormat) << ", "
                        << joint_positions().format(FlattenedCsvFormat) << '\n';
        trajectory_file.close();
      }
    }
  }

  double x() const { return x_; }
  double y() const { return y_; }
  double z() const { return z_; }
  double theta() const { return theta_; }
  double visualize() const { return visualize_; }

 private:
  bool visualize_;
  lcm::DrakeLcm lcm_;
  std::unique_ptr<RigidBodyTree<double>> iiwa_;
  std::unique_ptr<systems::Simulator<double>> simulator_;
  RobotPlanInterpolator* iiwa_trajectory_generator_;
  systems::SignalLogger<double>* logger_;
  std::unique_ptr<systems::Diagram<double>> sys_;
  double x_{0.80};
  double y_{-0.36};
  double z_{0.27};
  double theta_;
};

// Create a test scenario where the iiwa picks up an object 80cm in
// front of it and moves it 72cm to the right (symmetric about the
// center of the robot).  The test uses a single place location and
// does not loop.  The choice of the pick/place location is arbitrary.
TEST_P(PickAndPlaceInitialConditionTest, InitialConditionTest) {
  drake::log()->set_level(spdlog::level::err);
  Isometry3<double> place_location;
  place_location.translation() = Eigen::Vector3d(0.80, 0.36, 0);
  place_location.linear().setIdentity();
  std::vector<Isometry3<double>> place_locations;
  place_locations.push_back(place_location);


  // Test the non-looping configuration.
  PickAndPlaceStateMachine dut(place_locations, false);

  // Create world state and initialize with a trivial configuration.
  WorldState world_state(FindResourceOrThrow(kIiwaUrdf), "iiwa_link_ee");

  bot_core::robot_state_t iiwa_msg{};
  iiwa_msg.utime = 1000;
  iiwa_msg.pose.translation.x = 0;
  iiwa_msg.pose.translation.y = 0;
  iiwa_msg.pose.translation.z = 0;
  iiwa_msg.pose.rotation.w = 1;
  iiwa_msg.pose.rotation.x = 0;
  iiwa_msg.pose.rotation.y = 0;
  iiwa_msg.pose.rotation.z = 0;
  iiwa_msg.num_joints = kIiwaArmNumJoints;
  iiwa_msg.joint_name.push_back("iiwa_joint_1");
  iiwa_msg.joint_name.push_back("iiwa_joint_2");
  iiwa_msg.joint_name.push_back("iiwa_joint_3");
  iiwa_msg.joint_name.push_back("iiwa_joint_4");
  iiwa_msg.joint_name.push_back("iiwa_joint_5");
  iiwa_msg.joint_name.push_back("iiwa_joint_6");
  iiwa_msg.joint_name.push_back("iiwa_joint_7");

  iiwa_msg.joint_position.resize(kIiwaArmNumJoints, 0);
  iiwa_msg.joint_velocity.resize(kIiwaArmNumJoints, 0);
  world_state.HandleIiwaStatus(iiwa_msg);

  lcmt_schunk_wsg_status wsg_msg;
  wsg_msg.utime = iiwa_msg.utime;
  wsg_msg.actual_position_mm = 0;
  wsg_msg.actual_force = 0;
  world_state.HandleWsgStatus(wsg_msg);

  bot_core::robot_state_t object_msg{};
  object_msg.utime = 1000;
  object_msg.pose.translation.x = x();
  object_msg.pose.translation.y = y();
  object_msg.pose.translation.z = z();
  object_msg.pose.rotation.w = cos(theta()/2);
  object_msg.pose.rotation.x = 0;
  object_msg.pose.rotation.y = 0;
  object_msg.pose.rotation.z = sin(theta()/2);
  world_state.HandleObjectStatus(object_msg);

  int iiwa_plan_count = 0;
  robotlocomotion::robot_plan_t iiwa_plan{};
  PickAndPlaceStateMachine::IiwaPublishCallback iiwa_callback =
      [&iiwa_plan_count, &iiwa_plan](
          const robotlocomotion::robot_plan_t* plan) {
    iiwa_plan_count++;
    iiwa_plan = *plan;
  };

  int wsg_command_count = 0;
  lcmt_schunk_wsg_command wsg_command{};
  PickAndPlaceStateMachine::WsgPublishCallback wsg_callback =
      [&wsg_command_count, &wsg_command](
          const lcmt_schunk_wsg_command* command) {
    wsg_command_count++;
    wsg_command = *command;
  };

  manipulation::planner::ConstraintRelaxingIk planner(
      FindResourceOrThrow(kIiwaUrdf), "iiwa_link_ee",
      Isometry3<double>::Identity());

  dut.Update(world_state, iiwa_callback, wsg_callback, &planner);

  PiecewisePolynomial<double> pp;
  PiecewisePolynomial<double> pp_deriv;
  PiecewisePolynomial<double> pp_traj;

  while (dut.state() != kDone) {
    // Steps are long (5 seconds) so actions always complete in a
    // small number of steps.
    iiwa_msg.utime += 1000000;
    if (!iiwa_plan.plan.empty()) {
      SimulatePlan(iiwa_plan);
      iiwa_msg.joint_position = iiwa_plan.plan.back().joint_position;
      iiwa_plan.plan.clear();
    }
    world_state.HandleIiwaStatus(iiwa_msg);

    wsg_msg.utime = iiwa_msg.utime;
    wsg_msg.actual_position_mm = wsg_command.target_position_mm;
    world_state.HandleWsgStatus(wsg_msg);

    // Warp the object to the target y position when we expect to be
    // transitioning to kApproachPlace (this is the y value it would
    // have had after kApproachPlacePregrasp completed successfully).
    if (dut.state() == kApproachPlace) {
      object_msg.pose.translation.y = place_location.translation()(1);
      world_state.HandleObjectStatus(object_msg);
    }

    try {
        dut.Update(world_state, iiwa_callback, wsg_callback, &planner);
    } catch (...) {
      WriteToResultsFile(false);
      ASSERT_TRUE(false);
    }
  }
  WriteToResultsFile(true);
}

INSTANTIATE_TEST_CASE_P(WithoutVisualization, PickAndPlaceInitialConditionTest,
                        ::testing::Combine(::testing::Range(-0.1, 0.1, 0.01),
                                           ::testing::Range(-0.1, 0.1, 0.01),
                                           ::testing::Range(-M_PI_2, M_PI_2, 0.5*M_PI_4),
                                           ::testing::Values(false)));
INSTANTIATE_TEST_CASE_P(WithVisualization, PickAndPlaceInitialConditionTest,
                        ::testing::Combine(::testing::Range(-0.1, 0.1, 0.01),
                                           ::testing::Range(-0.1, 0.1, 0.01),
                                           ::testing::Range(-M_PI_2, M_PI_2, 0.5*M_PI_4),
                                           ::testing::Values(true)));
}  // namespace
}  // namespace pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
