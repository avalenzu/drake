#pragma once

#include <string>

#include "drake/common/eigen_types.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace monolithic_pick_and_place {

class OptitrackConfiguration {
 public:
  struct Target {
    std::string model_name;
    Eigen::Vector3d dimensions;
    int object_id;
  };

  struct Table {
    int object_id;
  };

  struct IiwaBase {
    int object_id;
  };

  const Target& target(int index) const { return targets_.at(index); };

  int num_targets() const { return targets_.size(); };

  const Table& table(int index) const { return tables_.at(index); };

  int num_tables() const { return tables_.size(); };

  const IiwaBase& iiwa_base(int index) const { return iiwa_bases_.at(index); };

  const Isometry3<double>& optitrack_frame() const { return X_WO_; };

 private:
  const std::vector<Target> targets_{
      {"block_for_pick_and_place.urdf", Eigen::Vector3d(0.06, 0.06, 0.2), 6},
      {"simple_cuboid.urdf", Eigen::Vector3d(0.06, 0.06, 0.06), 8},
      {"simple_cylinder.urdf", Eigen::Vector3d(0.065, 0.065, 0.13), 9},
      // These are hacky dimensions for the big robot toy.
      {"big_robot_toy.urdf", Eigen::Vector3d(0.1, 0.035, 0.18), 10}};
  const std::vector<Table> tables_{{2}, {3}, {4}, {5}};
  const std::vector<IiwaBase> iiwa_bases_{{0}, {1}};
  const Isometry3<double> X_WO_{
      AngleAxis<double>(M_PI_2, Vector3<double>::UnitX())};
};

}  // namespace monolithic_pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
