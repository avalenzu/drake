#include "drake/manipulation/util/model_tree/protobuf_converter.h"

#include <google/protobuf/text_format.h>
#include <spruce.hh>

#include "drake/common/drake_optional.h"
#include "drake/common/proto/protobuf.h"

using drake::optional;
using drake::nullopt;
using std::string;

namespace drake {
namespace manipulation {
namespace util {
namespace model_tree {

namespace {

// Returns true iff the path is relative (not absolute).
// Copied from drake/common/find_resource.h
bool IsRelativePath(const string& path) {
  // TODO(jwnimmer-tri) Prevent .. escape?
  return !path.empty() && (path[0] != '/');
}

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

AttachmentInfo ConvertAttachmentInfo(
    const proto::AttachmentInfo& proto_attachment_info) {
  return {proto_attachment_info.parent_model_instance_name(),
          proto_attachment_info.parent_body_or_frame_name()};
}

optional<AttachmentInfo> GetAttachmentInfo(
    const proto::ModelTreeNode& proto_node) {
  return proto_node.has_attachment_info()
             ? optional<AttachmentInfo>(
                   ConvertAttachmentInfo(proto_node.attachment_info()))
             : nullopt;
}

ModelFileType ConvertModelFileType(
    proto::ModelTreeNode::ModelFileType proto_model_file_type) {
  DRAKE_ASSERT(proto_model_file_type !=
               proto::ModelTreeNode::ModelFileType::
                   ModelTreeNode_ModelFileType_kModelTree);
  if (proto_model_file_type ==
      proto::ModelTreeNode::ModelFileType::ModelTreeNode_ModelFileType_kSdf) {
    return ModelFileType::kSdf;
  } else {
    return ModelFileType::kUrdf;
  }
}

}  // namespace

string ProtobufConverter::FindAbsoluteFilePathOrThrow(
    const string& filename) const {
  if (IsRelativePath(filename)) {
    return DoResolveRelativePathOrThrow(filename);
  } else {
    return filename;
  }
}

string ProtobufConverter::DoResolveRelativePathOrThrow(
    const string& relative_path) const {
  return FindResourceOrThrow(relative_path);
}

proto::ModelTree ProtobufConverter::ParseProtoModelTreeFromFileOrThrow(
    const string& filename) const {
  string absolute_path = FindAbsoluteFilePathOrThrow(filename);
  auto istream = drake::MakeFileInputStreamOrThrow(absolute_path);
  proto::ModelTree proto_model_tree;
  google::protobuf::TextFormat::Parse(istream.get(), &proto_model_tree);
  return proto_model_tree;
}

ModelTree ProtobufConverter::ParseModelTreeFromFileOrThrow(
    const string& filename) const {
  return ConvertModelTree(ParseProtoModelTreeFromFileOrThrow(filename));
}

ModelTree ProtobufConverter::ConvertModelTree(
    const proto::ModelTree& proto_model_tree,
    const drake::optional<proto::ModelTreeNode>& corresponding_proto_node)
    const {
  std::vector<ModelTreeNode> children;
  for (int i = 0; i < proto_model_tree.model_tree_node_size(); ++i) {
    children.push_back(
        ConvertModelTreeNode(proto_model_tree.model_tree_node(i)));
  }
  if (corresponding_proto_node) {
    // This is a nested model tree.
    return ModelTree(corresponding_proto_node->name(),
                     GetAttachmentInfo(*corresponding_proto_node), nullopt,
                     children);
  } else {
    // This is a top-level model tree.
    return ModelTree("", nullopt, nullopt, children);
  }
}

ModelTreeNode ProtobufConverter::ConvertModelTreeNode(
    const proto::ModelTreeNode& proto_node) const {
  string absolute_path =
      FindAbsoluteFilePathOrThrow(proto_node.model_file_path());
  proto::ModelTreeNode::ModelFileType proto_model_file_type =
      GetModelFileType(proto_node);
  if (proto_model_file_type == proto::ModelTreeNode::ModelFileType::
                                   ModelTreeNode_ModelFileType_kModelTree) {
    return ConvertModelTree(ParseProtoModelTreeFromFileOrThrow(absolute_path),
                            proto_node);
  } else {
    return ModelTreeNode(
        proto_node.name(), GetAttachmentInfo(proto_node),
        ModelFile(absolute_path, ConvertModelFileType(proto_model_file_type)),
        {});
  }
}

}  // namespace model_tree
}  // namespace util
}  // namespace manipulation
}  // namespace drake
