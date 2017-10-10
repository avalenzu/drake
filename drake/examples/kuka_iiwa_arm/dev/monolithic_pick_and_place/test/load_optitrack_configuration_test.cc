#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/load_optitrack_configuration.h"

#include <string>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace monolithic_pick_and_place {

const std::string kFilename{
    "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/"
    "optitrack_configurations/default.optitrack_rigid_bodies"};

GTEST_TEST(LoadOptitrackConfigurationTest,
           LoadOptitrackConfigurationFromFileTest) {
  OptitrackConfiguration optitrack_configuration{
      LoadOptitrackConfigurationFromFile(FindResourceOrThrow(kFilename))};
  const OptitrackConfiguration::Object object{
      optitrack_configuration.object("cube")};
  EXPECT_EQ(object.object_id, 3);
  EXPECT_EQ(object.model_path,
            "drake/examples/kuka_iiwa_arm/models/objects/simple_cuboid.urdf");
}

}  // namespace monolithic_pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
