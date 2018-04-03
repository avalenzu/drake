#include "drake/manipulation/util/model_tree/protobuf_converter.h"

#include <algorithm>
#include <vector>

#include <google/protobuf/text_format.h>
#include <spruce.hh>

#include "drake/common/drake_optional.h"
#include "drake/common/proto/protobuf.h"
#include "drake/math/transform.h"
#include "drake/multibody/joints/floating_base_types.h"

using drake::nullopt;
using drake::optional;
using drake::math::RotationMatrix;
using drake::math::Transform;
using std::string;
using BaseJointType = drake::multibody::joints::FloatingBaseType;

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

// Attempts to determine file type from file extension. Defaults to URDF.
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

// The following functions return the values stored in optional fields of
// Protobuf messages or default values.
proto::ModelTreeNode::ModelFileType GetModelFileType(
    const proto::ModelTreeNode& proto_node) {
  if (proto_node.has_model_file_type()) {
    return proto_node.model_file_type();
  }
  return GuessFileType(proto_node.model_file_path());
}

proto::Transform GetX_PM(const proto::ModelTreeNode& proto_node) {
  if (proto_node.has_x_pb()) {
    return proto_node.x_pb();
  }
  return proto::Transform();
}

optional<proto::PredecessorInfo> GetPredecessorInfo(
    const proto::ModelTreeNode& proto_node) {
  if (proto_node.has_predecessor_info()) {
    return optional<proto::PredecessorInfo>(proto_node.predecessor_info());
  }
  return nullopt;
}

bool GetIsFrame(const proto::PredecessorInfo& proto_predecessor_info) {
  if (proto_predecessor_info.has_is_frame()) {
    return proto_predecessor_info.is_frame();
  }
  return false;
}

// The following functions convert instances of proto::Foo to instances of Foo.
Vector3<double> ConvertVector3(const proto::Vector3& proto_vector) {
  return {proto_vector.x(), proto_vector.y(), proto_vector.z()};
}

RotationMatrix<double> ConvertSpaceXYZRotation(
    const proto::SpaceXYZRotation& proto_rpy) {
  return RotationMatrix<double>::MakeSpaceXYZRotation(
      {proto_rpy.roll(), proto_rpy.pitch(), proto_rpy.yaw()});
}

Transform<double> ConvertTransform(const proto::Transform& proto_X) {
  Transform<double> X{};
  if (proto_X.has_translation()) {
    X.set_translation(ConvertVector3(proto_X.translation()));
  }
  if (proto_X.has_rotation_rpy()) {
    X.set_rotation(ConvertSpaceXYZRotation(proto_X.rotation_rpy()));
  }
  return X;
}

optional<PredecessorInfo> ConvertPredecessorInfo(
    const optional<proto::PredecessorInfo>& proto_predecessor_info) {
  if (proto_predecessor_info) {
    return PredecessorInfo(proto_predecessor_info->model_instance_name(),
                           proto_predecessor_info->body_or_frame_name(),
                           GetIsFrame(*proto_predecessor_info));
  }
  return nullopt;
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

BaseJointType ConvertBaseJointType(
    const proto::ModelTreeNode::BaseJointType& proto_base_joint_type) {
  switch (proto_base_joint_type) {
    case proto::ModelTreeNode_BaseJointType_kFixed:
      return BaseJointType::kFixed;
    case proto::ModelTreeNode_BaseJointType_kFloating:
      return BaseJointType::kQuaternion;
  }
}

}  // namespace

string ProtobufConverter::FindAbsoluteFilePath(const string& filename) const {
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

proto::ModelTree ProtobufConverter::ParseProtoModelTreeFromFile(
    const string& filename) const {
  string absolute_path = FindAbsoluteFilePath(filename);
  auto istream = drake::MakeFileInputStreamOrThrow(absolute_path);
  proto::ModelTree proto_model_tree;
  google::protobuf::TextFormat::Parse(istream.get(), &proto_model_tree);
  return proto_model_tree;
}

ModelTree ProtobufConverter::ParseModelTreeFromFile(
    const string& filename) const {
  return ConvertModelTree(ParseProtoModelTreeFromFile(filename));
}

// Converts a proto::ModelTree into a ModelTree.
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
    return ModelTree(
        corresponding_proto_node->name(), nullopt,
        ConvertPredecessorInfo(GetPredecessorInfo(*corresponding_proto_node)),
        ConvertTransform(GetX_PM(*corresponding_proto_node)),
        ConvertBaseJointType(corresponding_proto_node->base_joint_type()),
        children);
  } else {
    // This is a top-level model tree.
    return ModelTree("", nullopt, nullopt, {}, BaseJointType::kFixed, children);
  }
}

// Converts a proto::ModelTreeNode into a ModelTreeNode.
ModelTreeNode ProtobufConverter::ConvertModelTreeNode(
    const proto::ModelTreeNode& proto_node) const {
  string absolute_path = FindAbsoluteFilePath(proto_node.model_file_path());
  proto::ModelTreeNode::ModelFileType proto_model_file_type =
      GetModelFileType(proto_node);
  if (proto_model_file_type == proto::ModelTreeNode::ModelFileType::
                                   ModelTreeNode_ModelFileType_kModelTree) {
    // This is a nested model tree node.
    return ConvertModelTree(ParseProtoModelTreeFromFile(absolute_path),
                            proto_node);
  } else {
    // This is a leaf model tree node.
    return ModelTreeNode(
        proto_node.name(),
        ModelFile(absolute_path, ConvertModelFileType(proto_model_file_type)),
        ConvertPredecessorInfo(GetPredecessorInfo(proto_node)),
        ConvertTransform(GetX_PM(proto_node)),
        ConvertBaseJointType(proto_node.base_joint_type()), {});
  }
}

}  // namespace model_tree
}  // namespace util
}  // namespace manipulation
}  // namespace drake
