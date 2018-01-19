#include "drake/manipulation/planner/differential_inverse_kinematics_system.h"

namespace drake {
namespace manipulation {
namespace planner {

using systems::Parameters;
DifferentialInverseKinematicsSystem::DifferentialInverseKinematicsSystem(
      std::unique_ptr<RigidBodyTree<double>> robot,
      const std::string& end_effector_frame_name) {
}

/// Reserves the parameters as required by CreateDefaultContext.
virtual std::unique_ptr<Parameters<double>> AllocateParameters() const {

}

}  // namespace planner
}  // namespace manipulation
}  // namespace drake
