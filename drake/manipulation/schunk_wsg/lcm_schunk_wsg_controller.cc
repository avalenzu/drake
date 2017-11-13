#include "drake/manipulation/schunk_wsg/lcm_schunk_wsg_controller.h"

#include "drake/manipulation/schunk_wsg/schunk_wsg_constants.h"
#include "drake/manipulation/schunk_wsg/schunk_wsg_controller.h"
#include "drake/manipulation/schunk_wsg/schunk_wsg_lcm.h"
#include "drake/systems/controllers/pid_controller.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/matrix_gain.h"
#include "drake/systems/primitives/pass_through.h"
#include "drake/systems/primitives/saturation.h"

namespace drake {
namespace manipulation {
namespace schunk_wsg {

LcmSchunkWsgController::LcmSchunkWsgController() {
  systems::DiagramBuilder<double> builder;

  auto wsg_trajectory_generator =
      builder.AddSystem<SchunkWsgTrajectoryGenerator>(
          kSchunkWsgNumPositions + kSchunkWsgNumVelocities,
          kSchunkWsgPositionIndex);
  command_input_port_ =
      builder.ExportInput(wsg_trajectory_generator->get_command_input_port());

  auto state_pass_through = builder.AddSystem<systems::PassThrough<double>>(
      kSchunkWsgNumPositions + kSchunkWsgNumVelocities);

  state_input_port_ = builder.ExportInput(state_pass_through->get_input_port());
  builder.Connect(state_pass_through->get_output_port(),
                  wsg_trajectory_generator->get_state_input_port());

  // The p gain here is somewhat arbitrary.  The goal is to make sure
  // that the maximum force is generated except when very close to the
  // target.
  const Eigen::VectorXd wsg_kp = Eigen::VectorXd::Constant(2, 2000.0);
  const Eigen::VectorXd wsg_ki = Eigen::VectorXd::Constant(2, 0.0);
  const Eigen::VectorXd wsg_kd = Eigen::VectorXd::Constant(2, 5.0);
  MatrixX<double> P_q(2, 2);
  P_q << -1, 1, 0.5, 0.5;
  const MatrixX<double> zero_size_P_q{
      MatrixX<double>::Zero(P_q.rows(), P_q.cols())};
  MatrixX<double> P_x(2, 4);
  P_x << P_q, zero_size_P_q, zero_size_P_q, P_q;
  const MatrixX<double> P_y{P_q.transpose()};
  auto wsg_controller =
      builder.AddSystem<systems::controllers::PidController<double>>(
          P_x, P_y, wsg_kp, wsg_ki, wsg_kd);

  builder.Connect(state_pass_through->get_output_port(),
                  wsg_controller->get_input_port_estimated_state());
  builder.Connect(wsg_trajectory_generator->get_target_output_port(),
                  wsg_controller->get_input_port_desired_state());

  // Create a gain block to negate the max force (to produce a minimum
  // force).
  auto positive_gain =
      builder.AddSystem<systems::MatrixGain<double>>(Vector2<double>(1, 1));
  auto negative_gain =
      builder.AddSystem<systems::MatrixGain<double>>(Vector2<double>(-1, -1));
  builder.Connect(wsg_trajectory_generator->get_max_force_output_port(),
                  positive_gain->get_input_port());
  builder.Connect(wsg_trajectory_generator->get_max_force_output_port(),
                  negative_gain->get_input_port());

  auto finger_controller = builder.AddSystem<SchunkWsgController<double>>();
  builder.Connect(wsg_controller->get_output_port_control(),
                  finger_controller->get_gripper_force_input_port());
  builder.Connect(state_pass_through->get_output_port(),
                  finger_controller->get_state_input_port());

  auto saturation = builder.AddSystem<systems::Saturation<double>>(2);
  builder.Connect(finger_controller->get_output_port(),
                  saturation->get_input_port());
  builder.Connect(positive_gain->get_output_port(),
                  saturation->get_max_value_port());
  builder.Connect(negative_gain->get_output_port(),
                  saturation->get_min_value_port());

  builder.ExportOutput(saturation->get_output_port());

  builder.BuildInto(this);
}

}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake
