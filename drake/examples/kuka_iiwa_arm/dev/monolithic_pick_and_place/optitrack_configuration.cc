#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/optitrack_configuration.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace monolithic_pick_and_place {
namespace {
// TODO(avalenzu): Move this copy-pasta somewhere common. The original is in
// drake/geometry/geometry_state.cc

// Helper method for consistently determining the presence of a key in a
// container and throwing a consistent exception type if absent.
// Searches for a key value in a "findable" object. To be findable, the source
// must have find(const Key&) and end() methods that return types that can
// be equality compared, such that if they are equal, the key is *not* present
// in the source. The exception message is produced by the given functor,
// make_message().
template <class Key, class Findable>
void FindOrThrow(const Key& key, const Findable& source,
                 const std::function<std::string()>& make_message) {
  if (source.find(key) == source.end()) throw std::logic_error(make_message());
}
}

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
