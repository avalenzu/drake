#include "drake/manipulation/util/model_tree/protobuf_converter.h"

#include <vector>

#include <gtest/gtest.h>

#include "drake/common/drake_optional.h"
#include "drake/common/find_resource.h"
#include "drake/math/transform.h"

using drake::nullopt;
using drake::optional;
using drake::math::RotationMatrix;
using drake::math::Transform;
using drake::multibody::joints::FloatingBaseType;

namespace drake {
namespace manipulation {
namespace util {
namespace model_tree {
namespace {

// This model tree should match the one described by top_level.model_tree and
// nested.model_tree.
ModelTree MakeModelTree() {
  return ModelTree(
      "", nullopt, nullopt, {} /*X_PM*/, FloatingBaseType::kFixed,
      {ModelTreeNode(
           "iiwa",
           ModelFile(
               FindResourceOrThrow("drake/manipulation/models/iiwa_description/"
                                   "urdf/iiwa14_spheres_collision.urdf"),
               ModelFileType::kUrdf),
           nullopt, {} /*X_PM*/, FloatingBaseType::kFixed, {} /*children*/),
       ModelTreeNode(
           "gripper_with_camera", nullopt,
           AttachmentInfo("iiwa", "iiwa_frame_ee"),
           Transform<double>(RotationMatrix<double>::MakeSpaceXYZRotation(
                                 {0.0, -0.39269908, 0.0}),
                             {0.0, 0.04, 0.0}),
           FloatingBaseType::kFixed,
           {ModelTreeNode(
                "wsg",
                ModelFile(FindResourceOrThrow(
                              "drake/manipulation/models/wsg_50_description/"
                              "sdf/schunk_wsg_50_ball_contact.sdf"),
                          ModelFileType::kSdf),
                nullopt, {} /*X_PM*/, FloatingBaseType::kFixed,
                {} /*children*/),
            ModelTreeNode(
                "xtion_wsg_fixture",
                ModelFile(FindResourceOrThrow(
                              "drake/manipulation/models/xtion_description/"
                              "urdf/xtion_wsg_fixture.urdf"),
                          ModelFileType::kUrdf),
                AttachmentInfo("wsg", "body_frame"), {} /*X_PM*/,
                FloatingBaseType::kFixed, {}),
            ModelTreeNode(
                "xtion",
                ModelFile(FindResourceOrThrow(
                              "drake/manipulation/models/xtion_description/"
                              "urdf/xtion.urdf"),
                          ModelFileType::kUrdf),
                AttachmentInfo("xtion_wsg_fixture", "xtion_wsg_fixture"),
                {} /*X_PM*/, FloatingBaseType::kFixed, {})})});
}

GTEST_TEST(ProtobufConverterTests, ParseModelTreeFromFileOrThrowTest) {
  ProtobufConverter converter{};
  ModelTree actual_tree = converter.ParseModelTreeFromFileOrThrow(
      "drake/manipulation/util/model_tree/test/top_level.model_tree");
  ModelTree expected_tree = MakeModelTree();
  EXPECT_EQ(actual_tree, expected_tree);
}

}  // namespace
}  // namespace model_tree
}  // namespace util
}  // namespace manipulation
}  // namespace drake
