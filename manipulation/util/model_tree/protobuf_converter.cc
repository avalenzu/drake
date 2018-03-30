#include "drake/manipulation/util/model_tree/protobuf_converter.h"

#include <google/protobuf/text_format.h>
#include <spruce.hh>

#include "drake/common/proto/protobuf.h"

using drake::optional;
using std::string;

namespace drake {
namespace manipulation {
namespace util {
namespace model_tree {

namespace {

proto::ModelTreeNode::ModelFileType GuessFileType(string filename) {
  spruce::path p(filename);
  // Converts the file extension to be lower case.
  auto extension = p.extension();
  std::transform(extension.begin(), extension.end(), extension.begin(),
                 ::tolower);
  if (extension == ".sdf") {
    return proto::ModelTreeNode::ModelFileType::
        ModelTreeNode_ModelFileType_kSdf;
  } else if (extension == ".model_tree") {
    return proto::ModelTreeNode::ModelFileType::
        ModelTreeNode_ModelFileType_kModelTree;
  } else {  // Default to URDF
    return proto::ModelTreeNode::ModelFileType::
        ModelTreeNode_ModelFileType_kUrdf;
  }
}

proto::ModelTreeNode::ModelFileType GetModelFileType(
    const proto::ModelTreeNode& proto_node) {
  return proto_node.has_model_file_type()
             ? proto_node.model_file_type()
             : GuessFileType(proto_node.model_file_path());
}
}  // namespace

void ProtobufConverter::ParseModelTreeFromFileOrThrow(
    const string& filename, ModelTree* model_tree) const {
  string absolute_path = FindAbsoluteFilePathOrThrow(filename);
  auto istream = drake::MakeFileInputStreamOrThrow(absolute_path);
  proto::ModelTree proto_model_tree;
  google::protobuf::TextFormat::Parse(istream.get(), &proto_model_tree);
  ConvertModelTree(proto_model_tree, model_tree);
}

void ProtobufConverter::ConvertModelTree(
    const proto::ModelTree& proto_model_tree, ModelTree* model_tree) const {
  for (int i = 0; i < proto_model_tree.model_tree_node_size(); ++i) {
    std::unique_ptr<ModelTreeNode> node{new ModelTreeNode()};
    ConvertModelTreeNode(proto_model_tree.model_tree_node(i), node.get());
    model_tree->AddChild(std::move(node));
  }
}

void ProtobufConverter::ConvertModelTreeNode(
    const proto::ModelTreeNode& proto_node, ModelTreeNode* node) const {
  string absolute_path =
      FindAbsoluteFilePathOrThrow(proto_node.model_file_path());
  proto::ModelTreeNode::ModelFileType proto_model_file_type =
      GetModelFileType(proto_node);
  if (proto_model_file_type == proto::ModelTreeNode::ModelFileType::
                                   ModelTreeNode_ModelFileType_kModelTree) {
    ParseModelTreeFromFileOrThrow(absolute_path, node);
  } else {
	  node->set_name(proto_node.name());
  }
}

}  // namespace model_tree
}  // namespace util
}  // namespace manipulation
}  // namespace drake
