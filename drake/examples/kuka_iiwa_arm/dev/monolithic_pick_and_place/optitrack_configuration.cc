#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/optitrack_configuration.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace monolithic_pick_and_place {

void OptitrackConfiguration::AddObject(const std::string& object_name,
                                       int object_id,
                                       const std::string& model_path) {
  objects_.emplace(object_name, Object{object_id, model_path});
}

void OptitrackConfiguration::AddObject(const std::string& object_name,
                                       int object_id,
                                       const std::string& model_path,
                                       const Vector3<double>&) {
  AddObject(object_name, object_id, model_path);
}

}  // namespace monolithic_pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
