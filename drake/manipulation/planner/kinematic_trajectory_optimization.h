#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"

namespace drake {
namespace manipulation {
namespace planner {

/// This class implements a plant with the following dynamics:
/// 
/// ⌈ qdot ⌉ = ⌈ 0 I 0 ⌉ ⌈ q ⌉   ⌈ 0 ⌉
/// | vdot | = | 0 0 I | | v | + | 0 | u
/// ⌊ adot ⌋ = ⌊ 0 0 0 ⌋ ⌊ a ⌋   ⌊ I ⌋
/// 
/// 
class KinematicTrajectoryOptimization {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(KinematicTrajectoryOptimization)

  KinematicTrajectoryOptimization(const multibody::MultibodyTree<double>& tree,
                                  int num_time_samples);
};

}  // namespace planner
}  // namespace manipulation
}  // namespace drake
