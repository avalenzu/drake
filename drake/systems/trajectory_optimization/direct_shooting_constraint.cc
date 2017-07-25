#include "drake/systems/trajectory_optimization/direct_shooting_constraint.h"

#include <utility>

#include "drake/common/drake_throw.h"
#include "drake/math/autodiff.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace systems {
// namespace {
// Eigen::MatrixXd ExtractDerivativesMatrix(const AutoDiffVecXd& vec_in) {
// if (!vec_in.size()) {
// return Eigen::MatrixXd();
//}

// Eigen::MatrixXd ret(vec_in.size(), vec_in(0).derivatives().size());
// for (int i = 0; i < ret.rows(); i++) {
// ret.row(i) = vec_in(i).derivatives();
//}
// return ret;
//}
//}  // namespace

DirectShootingConstraint::DirectShootingConstraint(int num_states,
                                                   int num_inputs)
    : Constraint(num_states, 1 + (2 * num_states) + (2 * num_inputs),
                 Eigen::VectorXd::Zero(num_states),
                 Eigen::VectorXd::Zero(num_states)),
      num_states_(num_states),
      num_inputs_(num_inputs) {}

DirectShootingConstraint::~DirectShootingConstraint() {}

void DirectShootingConstraint::DoEval(
    const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd& y) const {
  AutoDiffVecXd y_t;
  Eval(math::initializeAutoDiff(x), y_t);
  y = math::autoDiffToValueMatrix(y_t);
}

void DirectShootingConstraint::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                                      AutoDiffVecXd& y) const {
  DRAKE_ASSERT(x.size() == 1 + (2 * num_states_) + (2 * num_inputs_));

  // Extract our input variables:
  // h - current time (knot) value
  // x0, x1 state vector at time steps k, k+1
  // u0, u1 input vector at time steps k, k+1
  const AutoDiffXd h = x(0);
  const auto x0 = x.segment(1, num_states_);
  const auto x1 = x.segment(1 + num_states_, num_states_);
  const auto u0 = x.segment(1 + (2 * num_states_), num_inputs_);
  // const auto u1 = x.segment(1 + (2 * num_states_) + num_inputs_,
  // num_inputs_);

  AutoDiffVecXd x1_shoot;
  shoot(x0, u0, h, &x1_shoot);
  y = x1 - x1_shoot;
}

SystemDirectShootingConstraint::SystemDirectShootingConstraint(
    const systems::System<double>& system,
    const systems::Context<double>& context)
    : DirectShootingConstraint(context.get_continuous_state()->size(),
                               context.get_num_input_ports() > 0
                                   ? system.get_input_port(0).size()
                                   : 0) {
  DRAKE_THROW_UNLESS(system.get_num_input_ports() <= 1);
  DRAKE_THROW_UNLESS(context.has_only_continuous_state());

  DiagramBuilder<AutoDiffXd> builder;
  system_ = builder.AddSystem<systems::System<AutoDiffXd>>(
      systems::System<double>::ToAutoDiffXd(system));
  if (num_inputs() > 0) {
    control_input_ =
        builder.AddSystem<systems::ConstantVectorSource<AutoDiffXd>>(
            VectorX<AutoDiffXd>::Zero(num_inputs(), 1));
    builder.Cascade(*control_input_, *system_);
  }
  diagram_ = builder.Build();
  simulator_ = std::make_unique<Simulator<AutoDiffXd>>(*diagram_);
  context_ = simulator_->get_mutable_context();
  system_context_ = &(diagram_->GetMutableSubsystemContext(*system_, context_));
  control_input_context_ =
      &(diagram_->GetMutableSubsystemContext(*control_input_, context_));
  system_context_->SetTimeStateAndParametersFrom(context);

  // Set derivatives of all parameters in the SYSTEM context to zero (but with
  // the correct size).
  int num_gradients = 1 + 2 * num_states() + 2 * num_inputs();
  for (int i = 0; i < context_->get_parameters().num_numeric_parameters();
       i++) {
    auto params = system_context_->get_mutable_parameters()
                      .get_mutable_numeric_parameter(i)
                      ->get_mutable_value();
    for (int j = 0; j < params.size(); j++) {
      auto& derivs = params(j).derivatives();
      if (derivs.size() == 0) {
        derivs.resize(num_gradients);
        derivs.setZero();
      }
    }
  }
}

SystemDirectShootingConstraint::~SystemDirectShootingConstraint() {}

void SystemDirectShootingConstraint::shoot(const AutoDiffVecXd& state_0,
                                           const AutoDiffVecXd& input_0,
                                           const AutoDiffXd& duration,
                                           AutoDiffVecXd* state_1) const {
  if (context_->get_num_input_ports() > 0) {
    control_input_->get_mutable_source_value(control_input_context_)
        ->SetFromVector(input_0);
  }
  system_context_->get_mutable_continuous_state()->SetFromVector(state_0);
  context_->set_time(0.0);
  simulator_->StepTo(duration);
  *state_1 = system_context_->get_continuous_state()->CopyToVector();
}

}  // namespace systems
}  // namespace drake
