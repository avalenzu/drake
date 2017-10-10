#pragma once

#include <string>
#include <unordered_map>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace monolithic_pick_and_place {

class OptitrackConfiguration {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(OptitrackConfiguration);

  OptitrackConfiguration() = default;

  struct Object {
    int object_id;
    std::string model_name;
    Eigen::Vector3d dimensions;
  };

  const optional<Object> object(std::string object_name) const {
    const auto& itr = objects_.find(object_name);
    return (itr == objects_.cend()) ? optional<Object>{objects_.at(object_name)}
                                    : optional<Object>{};
  };

  int num_objects() const { return objects_.size(); };

  const Isometry3<double>& optitrack_frame() const { return X_WO_; };

  void AddObject(const std::string& object_name, int object_id,
                 const std::string& model_path = "",
                 const Vector3<double>& dimensions = Vector3<double>::Zero());

 private:
  std::unordered_map<std::string, Object> objects_{};
  Isometry3<double> X_WO_{Isometry3<double>::Identity()};
};

}  // namespace monolithic_pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
