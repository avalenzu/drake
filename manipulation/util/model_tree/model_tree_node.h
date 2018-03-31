#pragma once

#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"
#include "drake/math/transform.h"

namespace drake {
namespace manipulation {
namespace util {
namespace model_tree {

/** Describes how a model tree node is attached to the tree.
 */
struct AttachmentInfo {
  AttachmentInfo(const std::string& parent_model_instance_name_in,
                 const std::string& parent_body_or_frame_name_in)
      : parent_model_instance_name(parent_model_instance_name_in),
        parent_body_or_frame_name(parent_body_or_frame_name_in) {}
  std::string parent_model_instance_name{};
  std::string parent_body_or_frame_name{};
};

/** Enumerates supported model file types.
 */
enum class ModelFileType { kUrdf, kSdf };

/** Describes a model file
 */
struct ModelFile {
  ModelFile(const std::string& absolute_path_in, ModelFileType type_in)
      : absolute_path(absolute_path_in), type(type_in){};
  std::string absolute_path{};
  ModelFileType type{};
};

bool operator==(const AttachmentInfo& info_0, const AttachmentInfo& info_1) {
  return (info_0.parent_model_instance_name ==
          info_1.parent_model_instance_name) &&
         (info_0.parent_body_or_frame_name == info_1.parent_body_or_frame_name);
}

bool operator==(const ModelFile& file_0, const ModelFile& file_1) {
  return (file_0.absolute_path == file_1.absolute_path) &&
         (file_0.type == file_1.type);
}

/** Represents a node in a model tree.
 */
class ModelTreeNode {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ModelTreeNode);
  ModelTreeNode(const std::string& name,
                const drake::optional<AttachmentInfo>& attachment_info,
                const drake::optional<ModelFile>& model_file,
                const std::vector<ModelTreeNode>& children)
      : name_(name),
        attachment_info_(attachment_info),
        model_file_(model_file),
        children_(children) {}

  const std::string& name() const { return name_; }

  drake::optional<AttachmentInfo> attachment_info() const {
    return attachment_info_;
  }

  std::vector<ModelTreeNode> children() const { return children_; }

  drake::optional<ModelFile> model_file() const { return model_file_; }

  bool operator==(const ModelTreeNode& other) const {
    return (name_ == other.name_) &&
           (attachment_info_ == other.attachment_info_) &&
           (model_file_ == other.model_file_) && (children_ == other.children_);
  }

  bool operator!=(const ModelTreeNode& other) const {
    return !operator==(other);
  }
 private:
  std::string name_;
  drake::optional<AttachmentInfo> attachment_info_{};
  drake::optional<ModelFile> model_file_{};
  std::vector<ModelTreeNode> children_{};
};

typedef ModelTreeNode ModelTree;

}  // namespace model_tree
}  // namespace util
}  // namespace manipulation
}  // namespace drake
