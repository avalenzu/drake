#pragma once

#include <string>
#include <map>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"
#include "drake/manipulation/util/model_tree/base_joint.h"
#include "drake/manipulation/util/model_tree/model.h"
#include "drake/math/transform.h"
#include "drake/multibody/joints/floating_base_types.h"

namespace drake {
namespace manipulation {
namespace util {
namespace model_tree {

class ModelTree {
 public:
  Model* AddModel(const std::string& name, const ModelFile& model_file);
  void AddBaseJoint(const std::string& parent_model_name,
                    const std::string& child_model_name,
                    const BodyOrFrameName& parent_body_or_frame_name,
                    const multibody::joints::FloatingBaseType& type,
                    const drake::math::Transform<double>& X_PJ);
  void AddSubTree(const std::string& name, std::unique_ptr<ModelTree> sub_tree);
  const Model* first_root_model() const;
 private:
  void add_to_models_map(Model* model);
  Model* AddModel(std::unique_ptr<Model> model);
  std::map<std::string, Model*> models_{};
  std::unique_ptr<Model> root_{new Model()};
  std::map<std::string, std::unique_ptr<Model>> free_models_;
};

}  // namespace model_tree
}  // namespace util
}  // namespace manipulation
}  // namespace drake
