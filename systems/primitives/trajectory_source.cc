#include "drake/systems/primitives/trajectory_source.h"

#include "drake/common/drake_assert.h"
#include "drake/common/copyable_unique_ptr.h"
#include "drake/systems/framework/abstract_values.h"

namespace drake {
namespace systems {

using trajectories::Trajectory;

template <typename T>
TrajectorySource<T>::TrajectorySource(const Trajectory<T>& trajectory,
                                      int output_derivative_order,
                                      bool zero_derivatives_beyond_limits)
    : SingleOutputVectorSource<T>(trajectory.rows() *
                                  (1 + output_derivative_order)),
      // Make a copy of the input trajectory.
      clamp_derivatives_(zero_derivatives_beyond_limits) {
  // This class does not currently support trajectories which output
  // more complicated matrices.
  DRAKE_DEMAND(trajectory.cols() == 1);
  DRAKE_DEMAND(output_derivative_order >= 0);

  trajectory_index_ =
      this->DeclareAbstractParameter(Value<Trajectory<T>>(trajectory));
  for (int i = 0; i < output_derivative_order; i++) {
    derivative_indices_.push_back(this->DeclareAbstractParameter(
        Value<Trajectory<T>>(*trajectory.MakeDerivative(i))));
  }
}

template <typename T>
void TrajectorySource<T>::SetSourceTrajectoryInContext(
    Context<T>* context, const Trajectory<T>& trajectory) const {
  context->get_mutable_abstract_parameter(trajectory_index_)
      .template SetValue<Trajectory<T>>(trajectory);
  for (size_t i = 0; i < derivative_indices_.size(); ++i) {
    context->get_mutable_abstract_parameter(derivative_indices_.at(i))
        .template SetValue<Trajectory<T>>(*trajectory.MakeDerivative(i));
  }
}

template <typename T>
void TrajectorySource<T>::DoCalcVectorOutput(
    const Context<T>& context, Eigen::VectorBlock<VectorX<T>>* output) const {
  const Trajectory<T>& trajectory =
      context.get_abstract_parameter(trajectory_index_)
           .template GetValue<Trajectory<T>>();
  int len = trajectory.rows();
  double time = context.get_time();
  output->head(len) = trajectory.value(time);

  bool set_zero = clamp_derivatives_ && (time > trajectory.end_time() ||
                                         time < trajectory.start_time());

  for (size_t i = 0; i < derivative_indices_.size(); ++i) {
    if (set_zero) {
      output->segment(len * (i + 1), len).setZero();
    } else {
      const Trajectory<T>& derivative =
          context.get_abstract_parameter(derivative_indices_.at(i))
              .template GetValue<Trajectory<T>>();
      output->segment(len * (i + 1), len) = derivative.value(time);
    }
  }
}

template class TrajectorySource<double>;

}  // namespace systems
}  // namespace drake
