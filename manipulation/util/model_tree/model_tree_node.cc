#include "drake/manipulation/util/model_tree/model_tree_node.h"

#include "drake/common/text_logging.h"

using drake::nullopt;
using drake::optional;

namespace drake {
namespace manipulation {
namespace util {
namespace model_tree {

bool operator==(const AttachmentInfo& info_0, const AttachmentInfo& info_1) {
  DRAKE_SPDLOG_TRACE(
      drake::log(),
      "info_0.parent_model_instance_name == info_1.parent_model_instance_name: "
      "{}\n"
      "        info_0.parent_model_instance_name: {}\n"
      "        info_1.parent_model_instance_name: {}",
      info_0.parent_model_instance_name == info_1.parent_model_instance_name,
      info_0.parent_model_instance_name, info_1.parent_model_instance_name);
  DRAKE_SPDLOG_TRACE(
      drake::log(),
      "info_0.parent_body_or_frame_name == info_1.parent_body_or_frame_name: "
      "{}\n"
      "        info_0.parent_body_or_frame_name: {}\n"
      "        info_1.parent_body_or_frame_name: {}",
      info_0.parent_body_or_frame_name == info_1.parent_body_or_frame_name,
      info_0.parent_body_or_frame_name, info_1.parent_body_or_frame_name);
  return (info_0.parent_model_instance_name ==
          info_1.parent_model_instance_name) &&
         (info_0.parent_body_or_frame_name ==
          info_1.parent_body_or_frame_name) &&
         (info_0.attached_to_frame == info_1.attached_to_frame);
}

bool operator==(const ModelFile& file_0, const ModelFile& file_1) {
  DRAKE_SPDLOG_TRACE(drake::log(),
                     "file_0.absolute_path == file_1.absolute_path: {}\n"
                     "        file_0.absolute_path: {}\n"
                     "        file_1.absolute_path: {}",
                     file_0.absolute_path == file_1.absolute_path,
                     file_0.absolute_path, file_1.absolute_path);
  DRAKE_SPDLOG_TRACE(
      drake::log(),
      "file_0.type == file_1.type: {}\nfile_0.type: {}, file_1.type: {}",
      static_cast<int>(file_0.type), static_cast<int>(file_1.type));
  return (file_0.absolute_path == file_1.absolute_path) &&
         (file_0.type == file_1.type);
}

bool ModelTreeNode::operator==(const ModelTreeNode& other) const {
  DRAKE_SPDLOG_TRACE(drake::log(), "name_ == other.name_: {}",
                     name_ == other.name_);
  DRAKE_SPDLOG_TRACE(drake::log(), "model_file_ == other.model_file_: {}",
                     model_file_ == other.model_file_);
  DRAKE_SPDLOG_TRACE(drake::log(),
                     "attachment_info_ == other.attachment_info_: {}",
                     attachment_info_ == other.attachment_info_);
  DRAKE_SPDLOG_TRACE(drake::log(),
                     "X_PM.IsNearlyEqualTo(info_1.X_PM, 0.0): {}\n"
                     "X_PM:\n"
                     "{}\n"
                     "other.X_PM:\n"
                     "{}\n",
                     X_PM().IsNearlyEqualTo(other.X_PM(), 0.0),
                     X_PM().GetAsMatrix4(), other.X_PM().GetAsMatrix4());
  DRAKE_SPDLOG_TRACE(drake::log(),
                     "base_joint_type == other.base_joint_type_: {}",
                     base_joint_type_ == other.base_joint_type_);
  return (name_ == other.name_) && (model_file_ == other.model_file_) &&
         (attachment_info_ == other.attachment_info_) &&
         X_PM_.IsNearlyEqualTo(other.X_PM_, 0.0) &&
         (base_joint_type_ == other.base_joint_type_) &&
         (children_ == other.children_);
}

std::string ModelTreeNode::ParentNamePrefix() const {
  return parent_name_.empty() ? "" : parent_name_ + "/";
}

std::string ModelTreeNode::name() const { return ParentNamePrefix() + name_; }

optional<AttachmentInfo> ModelTreeNode::attachment_info() const {
  if (attachment_info_) {
    return AttachmentInfo(
        ParentNamePrefix() + attachment_info_->parent_model_instance_name,
        attachment_info_->parent_body_or_frame_name,
        attachment_info_->attached_to_frame);
  } else {
    return parent_attachment_info_;
  }
}

void ModelTreeNode::UpdateChildren() {
  for (ModelTreeNode& child : children_) {
    child.parent_name_ = name();
    child.parent_attachment_info_ = attachment_info();
    child.UpdateChildren();
  }
}

}  // namespace model_tree
}  // namespace util
}  // namespace manipulation
}  // namespace drake
