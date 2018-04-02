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
                     "info_0.X_P:\n"
                     "{}\n"
                     "info_1.X_P:\n"
                     "{}\n",
                     info_0.X_PM.IsNearlyEqualTo(info_1.X_PM, 0.0),
                     info_0.X_PM.GetAsMatrix4(), info_1.X_PM.GetAsMatrix4());
  DRAKE_SPDLOG_TRACE(drake::log(),
                     "base_joint_type == other.base_joint_type_: {}",
                     base_joint_type_ == other.base_joint_type_);
  return (name_ == other.name_) && (model_file_ == other.model_file_) &&
         (attachment_info_ == other.attachment_info_) &&
         X_PM_.IsNearlyEqualTo(other.X_PM_, 0.0) &&
         (base_joint_type_ == other.base_joint_type_) &&
         (children_ == other.children_);
}

std::string ModelTreeNode::name() const {
  return parent_ ? parent_->name() + name_ : name_;
}

optional<std::string> ModelTreeNode::model_absolute_path() const {
  return model_file_ ? optional<std::string>(model_file_->absolute_path)
                     : nullopt;
}

optional<ModelFileType> ModelTreeNode::model_file_type() const {
  return model_file_ ? optional<ModelFileType>(model_file_->type) : nullopt;
}

optional<std::string> ModelTreeNode::parent_model_instance_name() const {
  if (attachment_info_) {
    if (parent_) {
      return parent_->name() + attachment_info_->parent_model_instance_name;
    } else {
      return attachment_info_->parent_model_instance_name;
    }
  } else {
    if (parent_) {
      return parent_->parent_model_instance_name();
    } else {
      return nullopt;
    }
  }
}

optional<std::string> ModelTreeNode::parent_body_or_frame_name() const {
  if (attachment_info_) {
    return attachment_info_->parent_body_or_frame_name;
  } else if (parent_) {
    return parent_->parent_body_or_frame_name();
  } else {
    return nullopt;
  }
}

optional<bool> ModelTreeNode::attached_to_frame() const {
  return attachment_info_ ? optional<bool>(attachment_info_->attached_to_frame)
                          : nullopt;
}

}  // namespace model_tree
}  // namespace util
}  // namespace manipulation
}  // namespace drake
