#include "drake/manipulation/util/model_tree/model.h"

namespace drake {
namespace manipulation {
namespace util {
namespace model_tree {

bool operator==(const ModelFile& file_0, const ModelFile& file_1) {
  drake::log()->trace(
      "file_0.absolute_path == file_1.absolute_path: {}\n"
      "        file_0.absolute_path: {}\n"
      "        file_1.absolute_path: {}",
      file_0.absolute_path == file_1.absolute_path, file_0.absolute_path,
      file_1.absolute_path);
  drake::log()->trace(
      "file_0.type == file_1.type: {}\n"
      "        file_0.type: {}\n"
      "        file_1.type: {}",
      file_0.type == file_1.type, static_cast<int>(file_0.type),
      static_cast<int>(file_1.type));
  return (file_0.absolute_path == file_1.absolute_path) &&
         (file_0.type == file_1.type);
}

void Model::AddChild(Model* child, BaseJoint* base_joint) {
  DRAKE_ASSERT(child && base_joint)
  base_joint->set_parent_model(this);
  base_joint->set_child_model(child);
  child.base_joint_ = base_joint;
  if (first_child_joint_) {
    MutableLastChild()->next_sibling_ = child;
  } else {
    first_child_joint_ = base_joint;
  }
}

const Model* parent() const {
  if (base_joint_) {
    return base_joint_->parent_model();
  }
  return nullptr;
}

}  // namespace model_tree
}  // namespace util
}  // namespace manipulation
}  // namespace drake
