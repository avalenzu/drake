#pragma once

#include <memory>
#include <string>
#include <vector>

#include "drake/common/drake_optional.h"
#include "drake/math/transform.h"

namespace drake {
namespace manipulation {
namespace util {
namespace model_tree {

/** Describes how a model tree node is attached to the tree.
 */
struct AttachmentInfo {
  std::string parent_model_instance_name{};
  std::string parent_body_or_frame_name{};
};

/** Enumerates supported model file types.
 */
enum class ModelFileType { kUrdf, kSdf };

/** Describes a model file
 */
struct ModelFile {
  std::string absolute_path{};
  ModelFileType type{};
};

/** Represents a node in a model tree.
 */
class ModelTreeNode {
 public:
  const std::string& name() const { return name_; }

  void set_name(const std::string& name) { name_ = name; }

  const drake::optional<AttachmentInfo>& attachmentInfo() const {
    return attachment_info_;
  }

  void set_attachment_info(const AttachmentInfo& attachment_info) {
    attachment_info_ = attachment_info;
  }

  const std::vector<std::unique_ptr<const ModelTreeNode>>& children()
      const {
    return children_;
  }

  const drake::optional<const ModelFile>& model_file() const {
    return model_file_;
  }

  void set_model_file(const ModelFile model_file) {
    model_file_.emplace(model_file);
  }

  void AddChild(std::unique_ptr<const ModelTreeNode> child) {
    children_.emplace_back(std::move(child));
  }

 private:
  std::string name_;
  drake::optional<AttachmentInfo> attachment_info_{};
  drake::optional<const ModelFile> model_file_{};
  std::vector<std::unique_ptr<const ModelTreeNode>> children_{};
};

typedef ModelTreeNode ModelTree;

}  // namespace model_tree
}  // namespace util
}  // namespace manipulation
}  // namespace drake
