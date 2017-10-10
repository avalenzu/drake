#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/optitrack_configuration.h"
#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/optitrack_configuration.pb.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace monolithic_pick_and_place {
OptitrackConfiguration LoadOptitrackConfigurationFromFile(
    const std::string& path);
}  // namespace monolithic_pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
