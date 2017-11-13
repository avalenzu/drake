#include "drake/manipulation/schunk_wsg/schunk_wsg_controller.h"

#include "drake/common/default_scalars.h"
#include "drake/manipulation/schunk_wsg/schunk_wsg_constants.h"
#include "drake/systems/controllers/pid_controller.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/adder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/matrix_gain.h"
#include "drake/systems/primitives/pass_through.h"
#include "drake/systems/primitives/saturation.h"

namespace drake {
namespace manipulation {
namespace schunk_wsg {

template <typename T>
SchunkWsgController<T>::SchunkWsgController() {
  systems::DiagramBuilder<T> builder;

  // x = (q, v),  x̃ = (q̃, ṽ)
  //
  // q̃ = (q₀ + q₁)/2
  //
  // ṽ = (v₀ + v₁)/2
  MatrixX<double> J_q(1, 2);
  J_q << 0.5, 0.5;
  MatrixX<double> J_x(2, 4);
  J_x << J_q, MatrixX<double>::Zero(1, 2), MatrixX<double>::Zero(1, 2), J_q;

  auto convert_to_x_tilde =
      builder.template AddSystem<systems::MatrixGain<T>>(J_x);
  state_input_port_ = builder.ExportInput(convert_to_x_tilde->get_input_port());

  // The mean finger position should be zero.
  auto x_tilde_desired =
      builder.template AddSystem<systems::ConstantVectorSource<T>>(
          Vector2<T>::Zero());

  // The p gain here is somewhat arbitrary.  The goal is to make sure
  // that the maximum force is generated except when very close to the
  // target.
  const Eigen::VectorXd wsg_kp = Eigen::VectorXd::Constant(1, 2000.0);
  const Eigen::VectorXd wsg_ki = Eigen::VectorXd::Constant(1, 0.0);
  const Eigen::VectorXd wsg_kd = Eigen::VectorXd::Constant(1, 5.0);

  auto pid_controller =
      builder.template AddSystem<systems::controllers::PidController<T>>(
          wsg_kp, wsg_ki, wsg_kd);

  builder.Connect(convert_to_x_tilde->get_output_port(),
                  pid_controller->get_input_port_estimated_state());
  builder.Connect(x_tilde_desired->get_output_port(),
                  pid_controller->get_input_port_desired_state());

  // The PID controller outputs u_tilde. Add a matrix gain block to convert to
  // u.
  auto convert_u_tilde_to_u =
      builder.template AddSystem<systems::MatrixGain<T>>(J_q.transpose());
  builder.Connect(pid_controller->get_output_port_control(),
                  convert_u_tilde_to_u->get_input_port());

  auto convert_gripper_force_to_u =
      builder.template AddSystem<systems::MatrixGain<T>>(
          Vector2<double>(-1, 1));
  gripper_force_input_port_ =
      builder.ExportInput(convert_gripper_force_to_u->get_input_port());

  auto add_u_terms = builder.template AddSystem<systems::Adder<T>>(2, 2);
  builder.Connect(convert_u_tilde_to_u->get_output_port(),
                  add_u_terms->get_input_port(0));
  builder.Connect(convert_gripper_force_to_u->get_output_port(),
                  add_u_terms->get_input_port(1));

  builder.ExportOutput(add_u_terms->get_output_port());

  // Add the commanded gripper force
  builder.BuildInto(this);
}

}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::manipulation::schunk_wsg::SchunkWsgController)
