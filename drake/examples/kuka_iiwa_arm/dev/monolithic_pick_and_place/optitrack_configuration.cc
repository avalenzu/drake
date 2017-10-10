#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/optitrack_configuration.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace monolithic_pick_and_place {

const OptitrackConfiguration::Object& OptitrackConfiguration::GetObjectOrThrow(
    const std::string object_name) const {
  auto itr = objects_.find(object_name);
  if (itr != objects_.end()) {
    return itr->second;
  }
  throw std::logic_error("Unknown object name: " + object_name + ".");
};

void OptitrackConfiguration::AddObject(const std::string& object_name,
                                       int object_id,
                                       const std::string& model_path,
                                       const Vector3<double>& dimensions) {
  objects_.emplace(object_name, Object{object_id, model_path, dimensions});
}

}  // namespace monolithic_pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
