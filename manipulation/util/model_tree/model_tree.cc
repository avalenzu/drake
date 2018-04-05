#include "drake/manipulation/util/model_tree/model_tree.h"

#include <set>

#include "drake/common/text_logging.h"

using drake::nullopt;
using drake::optional;
using drake::multibody::joints::FloatingBaseType;

namespace drake {
namespace manipulation {
namespace util {
namespace model_tree {

void ModelTree::add_to_models_map(Model* model) {
  DRAKE_ASSERT(model);
  if (models_.insert(std::make_pair(model->name(), model)).second) {
    throw std::runtime_error("Duplicate model name: " + model->name());
  }
}

Model* ModelTree::AddModel(const std::string& name,
                           const ModelFile& model_file) {
  return AddModel(std::make_unique<Model>(name, model_file));
}

Model* ModelTree::AddModel(std::unique_ptr<Model> model) {
  // Since we're just adding this model, it cannot yet be a child of another
  // model. Therefore, we add it to free_models_ here.
  std::string name = model->name();
  std::map<std::string, std::unique_ptr<Model>>::iterator free_model_itr;
  bool already_in_free_models;
  std::tie(free_model_itr, already_in_free_models) =
      free_models_.insert(std::make_pair(name, std::move(model)));
  std::map<std::string, std::unique_ptr<Model>>::iterator model_itr;
  // Now we add the non-owning pointer to the full map in models_.
  if (already_in_free_models) {
    throw std::runtime_error("Duplicate model name: " + name);
  }
  add_to_models_map(free_model_itr->second.get());
  return free_model_itr->second.get();
}

void ModelTree::AddBaseJoint(const std::string& parent_model_name,
                             const std::string& child_model_name,
                             const BodyOrFrameName& parent_body_or_frame_name,
                             const multibody::joints::FloatingBaseType& type,
                             const drake::math::Transform<double>& X_PJ) {
  // Find the parent model in the full map of models.
  auto parent_itr = models_.find(parent_model_name);
  if (parent_itr == models_.end()) {
    throw std::runtime_error("Invalid model name \"" + parent_model_name +
                             "\" for parent. No such model has been added");
  }
  // Find the child model in the map of free models - each model can only be a
  // child of one other model.
  auto child_itr = free_models_.find(child_model_name);
  if (child_itr == free_models_.end()) {
    throw std::runtime_error("Invalid model name \"" + child_model_name +
                             "\" for child.");
  }
  parent_itr->second->AddChild(
      std::move(child_itr->second),
      std::make_unique<BaseJoint>(parent_body_or_frame_name, type, X_PJ));

  // Remove the entry for the child from the map of free models.
  free_models_.erase(child_itr);
}

void ModelTree::AddSubTree(const std::string& name,
                           std::unique_ptr<ModelTree> sub_tree) {
  for (auto& name_and_model : sub_tree->free_models_) {
    name_and_model.second->add_name_prefix(name + "/");
    Model* model =
        AddModel(std::move(name_and_model.second))->mutable_next_model();
    while (model) {
      add_to_models_map(model);
      model = model->mutable_next_model();
    }
  }
}

}  // namespace model_tree
}  // namespace util
}  // namespace manipulation
}  // namespace drake
