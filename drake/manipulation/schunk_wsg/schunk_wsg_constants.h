/**
 * @file
 *
 * Constants defined in this file are for the Schunk WSG gripper modeled in
 * models/schunk_wsg_50.sdf. Although the gripper only has one actuator, the
 * model has a number of constrained linkages to mimic two fingers moving
 * in a coordinated manner. This results in more states than actuators, and
 * a need for a selection matrix for state feedback control.
 */

#pragma once

#include "drake/common/eigen_types.h"

namespace drake {
namespace manipulation {
namespace schunk_wsg {

constexpr int kSchunkWsgNumActuators = 2;
constexpr int kSchunkWsgNumPositions = 2;
constexpr int kSchunkWsgNumVelocities = 2;

constexpr int kSchunkWsgPositionIndex = 0;
constexpr int kSchunkWsgVelocityIndex =
    kSchunkWsgNumPositions + kSchunkWsgPositionIndex;

// TODO(sammy): need to double check this.
constexpr double kSchunkWsgLcmStatusPeriod = 0.05;

}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake
