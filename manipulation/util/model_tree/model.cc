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

Model* Model::AddChild(std::unique_ptr<Model> child) {
  DRAKE_ASSERT(child != nullptr)
  if (first_child_) {
    Model* last_child = first_child_.get();
    while (last_child->next_sibling_) {
      last_child = last_child->next_sibling_.get();
    }
    last_child->next_sibling_ = std::move(child);
    return last_child->next_sibling_.get();
  } else {
    first_child_ = std::move(child);
    return first_child_.get();
  }
}

Model* Model::AddChild(std::unique_ptr<Model> child,
                       std::unique_ptr<BaseJoint> base_joint) {
  DRAKE_ASSERT(child && base_joint)
  base_joint->set_parent_model(this);
  base_joint->set_child_model(child.get());
  child->base_joint_ = std::move(base_joint);
  return AddChild(std::move(child));
}

const Model* Model::parent() const {
  if (base_joint_) {
    return base_joint_->parent_model();
  }
  return nullptr;
}

Model* Model::mutable_parent() {
  if (base_joint_) {
    return base_joint_->mutable_parent_model();
  }
  return nullptr;
}

void Model::add_name_prefix(const std::string& prefix) {
  Model* model = this;
  while (model) {
    model->name_ = prefix + model->name_;
    model = model->mutable_next_model();
  }
}

Model* Model::mutable_next_model() {
  if (first_child_) {
    return first_child_.get();
  }
  if (next_sibling_) {
    return next_sibling_.get();
  }
  if (base_joint_) {
    return base_joint_->mutable_parent_model();
  }
  return nullptr;
}

const Model* Model::next_model() const {
  if (first_child_) {
    return first_child_.get();
  }
  if (next_sibling_) {
    return next_sibling_.get();
  }
  if (parent()) {
    return parent()->next_model();
  }
  return nullptr;
}

}  // namespace model_tree
}  // namespace util
}  // namespace manipulation
}  // namespace drake
