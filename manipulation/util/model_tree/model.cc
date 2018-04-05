#include "drake/manipulation/util/model_tree/model.h"

#include "drake/common/text_logging.h"

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

void Model::AddChild(std::unique_ptr<Model> child,
                     std::unique_ptr<BaseJoint> base_joint) {
  DRAKE_ASSERT(child && base_joint)
  base_joint->set_parent_model(this);
  base_joint->set_child_model(child.get());
  child->base_joint_ = std::move(base_joint);
  if (first_child_) {
    Model* last_child = first_child_.get();
    while (last_child->next_sibling()) {
      last_child = last_child->next_sibling_.get();
    }
    last_child->next_sibling_ = std::move(child);
  } else {
    first_child_ = std::move(child);
  }
}

const Model* Model::parent() const {
  if (base_joint_) {
    return base_joint_->parent_model();
  }
  return nullptr;
}

}  // namespace model_tree
}  // namespace util
}  // namespace manipulation
}  // namespace drake
