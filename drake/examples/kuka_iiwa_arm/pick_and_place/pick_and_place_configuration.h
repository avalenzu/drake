#pragma once

#include <string>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/common/type_safe_index.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace pick_and_place {

using RobotBaseIndex = TypeSafeIndex<class RobotBaseTag>;
using TargetIndex = TypeSafeIndex<class TargetTag>;

struct OptitrackInfo {
  int id{0};
  Isometry3<double> X_MF{Isometry3<double>::Identity()};
};

struct PlannerConfiguration {
  std::string model_path;
  std::string end_effector_name;
  RobotBaseIndex robot_base_index{0};
  TargetIndex target_index{0};
  Vector3<double> target_dimensions;
  double period_sec{0.01};
  int num_tables{0};
};

struct SimulatedPlantConfiguration {
  std::vector<Isometry3<double>> robot_base_poses;
  std::vector<std::string> table_models;
  std::vector<Isometry3<double>> table_poses;
  std::vector<std::string> object_models;
  std::vector<Isometry3<double>> object_poses;
  double static_friction_coef{0.9};
  double dynamic_friction_coef{0.5};
  double v_stiction_tolerance{0.01};
  double stiffness{10000};
  double dissipation{2};
};

struct OptitrackConfiguration {
  std::vector<int> robot_base_optitrack_ids{};
  std::vector<int> object_optitrack_ids{};
  std::vector<OptitrackInfo> table_optitrack_info;
};

}  // namespace pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
