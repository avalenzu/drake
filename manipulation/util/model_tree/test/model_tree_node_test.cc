#include "drake/manipulation/util/model_tree/model_tree_node.h"

#include <vector>

#include <gtest/gtest.h>

#include "drake/common/drake_optional.h"

using drake::nullopt;

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

std::vector<ModelTreeNode> MakeModelTreeNodes(int num_nodes) {
  std::vector<ModelTreeNode> nodes{};
  for (int i = 0; i < num_nodes; ++i) {
    std::string model_name = "model_" + std::to_string(i);
    AttachmentInfo attachment_info{kModelName, "my_model_body_0"};
    ModelFile model_file{"/path/to/" + model_name, kModelFileType};
    nodes.emplace_back(model_name, attachment_info, model_file,
                       std::vector<ModelTreeNode>());
  }
  return nodes;
}

class ModelTreeNodeTest : public ::testing::Test {
 public:
  ModelTreeNodeTest()
      : model_(kModelName, AttachmentInfo(kParentName, kParentBodyName),
               ModelFile(kModelPath, kModelFileType), MakeModelTreeNodes(5)) {}

 protected:
  ModelTreeNode model_;
};

TEST_F(ModelTreeNodeTest, ConstructorTest) {
  EXPECT_EQ(model_.name(), kModelName);
  EXPECT_EQ(model_.attachment_info(),
            AttachmentInfo(kParentName, kParentBodyName));
  EXPECT_EQ(model_.model_file(), ModelFile(kModelPath, kModelFileType));
  EXPECT_EQ(model_.children(), MakeModelTreeNodes(5));
}

TEST_F(ModelTreeNodeTest, OperatorEqualsTest) {
  // Add parent.
  // Construct comparison models.
  ModelTreeNode same_model{
      kModelName, AttachmentInfo(kParentName, kParentBodyName),
      ModelFile(kModelPath, kModelFileType), MakeModelTreeNodes(5)};

  ModelTreeNode different_name{
      "my_other_model", AttachmentInfo(kParentName, kParentBodyName),
      ModelFile(kModelPath, kModelFileType), MakeModelTreeNodes(5)};

  ModelTreeNode different_attachment_info{
      kModelName, AttachmentInfo("foo", kParentBodyName),
      ModelFile(kModelPath, kModelFileType), MakeModelTreeNodes(5)};

  ModelTreeNode different_model_file{
      kModelName, AttachmentInfo(kParentName, kParentBodyName),
      ModelFile(kModelPath, ModelFileType::kSdf), MakeModelTreeNodes(5)};

  ModelTreeNode different_children{
      kModelName, AttachmentInfo(kParentName, kParentBodyName),
      ModelFile(kModelPath, ModelFileType::kSdf), MakeModelTreeNodes(3)};

  EXPECT_TRUE(model_ == same_model);
  EXPECT_FALSE(model_ != same_model);

  for (const auto& different_model :
       {different_name, different_attachment_info, different_model_file,
        different_children}) {
    EXPECT_FALSE(model_ == different_model);
    EXPECT_TRUE(model_ != different_model);
  }
}

}  // namespace
}  // namespace model_tree
}  // namespace util
}  // namespace manipulation
}  // namespace drake
