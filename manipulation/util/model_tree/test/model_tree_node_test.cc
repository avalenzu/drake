#include "drake/manipulation/util/model_tree/model_tree_node.h"

#include <vector>

#include <gtest/gtest.h>

#include "drake/common/drake_optional.h"
#include "drake/multibody/joints/floating_base_types.h"

using drake::nullopt;
using drake::math::RotationMatrix;
using drake::math::Transform;
using drake::multibody::joints::FloatingBaseType;

namespace drake {
namespace manipulation {
namespace util {
namespace model_tree {
namespace {

constexpr char kModelName[] = "my_model";
constexpr char kModelPath[] = "my/model/path";
constexpr ModelFileType kModelFileType = ModelFileType::kUrdf;
constexpr char kParentName[] = "my_parent";
constexpr char kParentBodyName[] = "my_parent_body";
constexpr FloatingBaseType kBaseJointType = FloatingBaseType::kQuaternion;

std::vector<ModelTreeNode> MakeModelTreeNodes(int num_nodes) {
  std::vector<ModelTreeNode> nodes{};
  for (int i = 0; i < num_nodes; ++i) {
    std::string model_name = "model_" + std::to_string(i);
    AttachmentInfo attachment_info{kModelName, "my_model_body_0"};
    ModelFile model_file{"/path/to/" + model_name, kModelFileType};
    nodes.emplace_back(model_name, model_file, attachment_info,
                       Transform<double>(), FloatingBaseType::kFixed,
                       std::vector<ModelTreeNode>());
  }
  return nodes;
}

Transform<double> MakeX_PM() {
  return {RotationMatrix<double>::MakeSpaceXYZRotation({1., 2., 3.}),
          {1., 2., 3}};
}

class ModelTreeNodeTest : public ::testing::Test {
 public:
  ModelTreeNodeTest()
      : model_(kModelName, ModelFile(kModelPath, kModelFileType),
               AttachmentInfo(kParentName, kParentBodyName), MakeX_PM(),
               kBaseJointType, MakeModelTreeNodes(5)) {}

 protected:
  ModelTreeNode model_;
};

TEST_F(ModelTreeNodeTest, ConstructorTest) {
  EXPECT_EQ(model_.name(), kModelName);
  ASSERT_TRUE(model_.has_model_file());
  EXPECT_EQ(model_.model_absolute_path(), kModelPath);
  EXPECT_EQ(model_.model_file_type(), kModelFileType);
  EXPECT_EQ(*model_.parent_model_instance_name(),
            kParentName);
  EXPECT_EQ(model_.parent_body_or_frame_name(), kParentBodyName);
  EXPECT_EQ(*model_.attached_to_frame(), false);
  EXPECT_TRUE(model_.X_PM().IsNearlyEqualTo(MakeX_PM(), 0.0));
  EXPECT_EQ(model_.base_joint_type(), kBaseJointType);
  EXPECT_EQ(model_.children(), MakeModelTreeNodes(5));
}

TEST_F(ModelTreeNodeTest, OperatorEqualsTest) {
  // Add parent.
  // Construct comparison models.
  ModelTreeNode same_model{kModelName,
                           ModelFile(kModelPath, kModelFileType),
                           AttachmentInfo(kParentName, kParentBodyName),
                           MakeX_PM(),
                           kBaseJointType,
                           MakeModelTreeNodes(5)};

  ModelTreeNode different_name{"my_other_model",
                               ModelFile(kModelPath, kModelFileType),
                               AttachmentInfo(kParentName, kParentBodyName),
                               MakeX_PM(),
                               kBaseJointType,
                               MakeModelTreeNodes(5)};

  ModelTreeNode different_model_file{
      kModelName,
      ModelFile(kModelPath, ModelFileType::kSdf),
      AttachmentInfo(kParentName, kParentBodyName),
      MakeX_PM(),
      kBaseJointType,
      MakeModelTreeNodes(5)};

  ModelTreeNode different_attachment_info{
      kModelName,
      ModelFile(kModelPath, kModelFileType),
      AttachmentInfo("foo", kParentBodyName),
      MakeX_PM(),
      kBaseJointType,
      MakeModelTreeNodes(5)};

  ModelTreeNode different_X_PM{kModelName,
                               ModelFile(kModelPath, kModelFileType),
                               AttachmentInfo(kParentName, kParentBodyName),
                               {},
                               kBaseJointType,
                               MakeModelTreeNodes(5)};

  ModelTreeNode different_base_joint_type{
      kModelName,
      ModelFile(kModelPath, kModelFileType),
      AttachmentInfo(kParentName, kParentBodyName),
      MakeX_PM(),
      FloatingBaseType::kFixed,
      MakeModelTreeNodes(5)};

  ModelTreeNode different_children{kModelName,
                                   ModelFile(kModelPath, ModelFileType::kSdf),
                                   AttachmentInfo(kParentName, kParentBodyName),
                                   MakeX_PM(),
                                   kBaseJointType,
                                   MakeModelTreeNodes(3)};

  EXPECT_TRUE(model_ == same_model);
  EXPECT_FALSE(model_ != same_model);

  EXPECT_FALSE(model_ == different_name);
  EXPECT_TRUE(model_ != different_name);

  EXPECT_FALSE(model_ == different_model_file);
  EXPECT_TRUE(model_ != different_model_file);

  EXPECT_FALSE(model_ == different_attachment_info);
  EXPECT_TRUE(model_ != different_attachment_info);

  EXPECT_FALSE(model_ == different_X_PM);
  EXPECT_TRUE(model_ != different_X_PM);

  EXPECT_FALSE(model_ == different_base_joint_type);
  EXPECT_TRUE(model_ != different_base_joint_type);

  EXPECT_FALSE(model_ == different_children);
  EXPECT_TRUE(model_ != different_children);
}

}  // namespace
}  // namespace model_tree
}  // namespace util
}  // namespace manipulation
}  // namespace drake
