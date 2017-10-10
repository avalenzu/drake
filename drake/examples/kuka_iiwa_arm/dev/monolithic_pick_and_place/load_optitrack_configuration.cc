#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/load_optitrack_configuration.h"

#include "google/protobuf/text_format.h"

#include "drake/common/proto/protobuf.h"
#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/optitrack_configuration.pb.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace monolithic_pick_and_place {
OptitrackConfiguration LoadOptitrackConfigurationFromFile(
    const std::string& path) {
  OptitrackConfiguration optitrack_configuration;
  OptitrackRigidBodies rigid_bodies;
  auto istream = drake::MakeFileInputStreamOrThrow(path);
  google::protobuf::TextFormat::Parse(istream.get(), &rigid_bodies);

  for (const auto& body : rigid_bodies.rigid_body()) {
    optitrack_configuration.AddObject(
        body.name(), body.id(), body.model_path());
  }
  return optitrack_configuration;
}
}  // namespace monolithic_pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
