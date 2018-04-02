#pragma once

#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"
#include "drake/math/transform.h"
#include "drake/multibody/joints/floating_base_types.h"

namespace drake {
namespace manipulation {
namespace util {
namespace model_tree {

/** Describes how a model tree node is attached to the tree.
 */
struct AttachmentInfo {
  AttachmentInfo(const std::string& parent_model_instance_name_in,
                 const std::string& parent_body_or_frame_name_in,
                 bool attached_to_frame_in = false)
      : parent_model_instance_name(parent_model_instance_name_in),
        parent_body_or_frame_name(parent_body_or_frame_name_in),
        attached_to_frame(attached_to_frame_in) {}
  std::string parent_model_instance_name{};
  std::string parent_body_or_frame_name{};
  bool attached_to_frame{false};
};

/** Enumerates supported model file types.
 */
enum class ModelFileType { kUrdf, kSdf };

/** Describes a model file
 */
struct ModelFile {
  ModelFile(const std::string& absolute_path_in, ModelFileType type_in)
      : absolute_path(absolute_path_in), type(type_in) {}
  std::string absolute_path{};
  ModelFileType type{};
};

bool operator==(const AttachmentInfo& info_0, const AttachmentInfo& info_1);

bool operator==(const ModelFile& file_0, const ModelFile& file_1);

/** Represents a node in a model tree.
 */
class ModelTreeNode {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ModelTreeNode);
  ModelTreeNode(const std::string& name,
                const drake::optional<ModelFile>& model_file,
                const drake::optional<AttachmentInfo>& attachment_info,
                const drake::math::Transform<double>& X_PM,
                drake::multibody::joints::FloatingBaseType base_joint_type,
                const std::vector<ModelTreeNode>& children)
      : name_(name),
        model_file_(model_file),
        attachment_info_(attachment_info),
        X_PM_(X_PM),
        base_joint_type_(base_joint_type),
        children_(children) {
    for (auto& child : children_) {
      child.parent_ = this;
    }
  }

  std::string name() const;

  bool has_model_file() const { return static_cast<bool>(model_file_); }

  bool has_attachment_info() const { return static_cast<bool>(attachment_info_); }

  drake::optional<std::string> model_absolute_path() const;

  drake::optional<ModelFileType> model_file_type() const;

  drake::optional<std::string> parent_model_instance_name() const;

  drake::optional<std::string> parent_body_or_frame_name() const;

  drake::optional<bool> attached_to_frame() const;

  const drake::math::Transform<double>& X_PM() const { return X_PM_; }

  drake::multibody::joints::FloatingBaseType base_joint_type() const {
    return base_joint_type_;
  }

  std::vector<ModelTreeNode> children() const { return children_; }

  bool operator==(const ModelTreeNode& other) const;

  bool operator!=(const ModelTreeNode& other) const {
    return !operator==(other);
  }

 private:
  std::string name_;
  drake::optional<ModelFile> model_file_{};
  drake::optional<AttachmentInfo> attachment_info_{};
  drake::math::Transform<double> X_PM_{};
  drake::multibody::joints::FloatingBaseType base_joint_type_{};
  std::vector<ModelTreeNode> children_{};
  const ModelTreeNode* parent_{};
};

typedef ModelTreeNode ModelTree;

}  // namespace model_tree
}  // namespace util
}  // namespace manipulation
}  // namespace drake
