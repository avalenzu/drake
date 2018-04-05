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
  void AddModel(const std::string& name, const ModelFile& model_file);
  void AddBaseJoint(const std::string& parent_model_name,
                    const std::string& child_model_name,
                    const BodyOrFrameName& parent_body_or_frame_name,
                    const multibody::joints::FloatingBaseType& type,
                    const drake::math::Transform<double>& X_PJ);
  void AddSubTree(std::unique_ptr<ModelTree> sub_tree);
  const Model* first_root_model() const;
 private:
  std::map<std::string, Model*> models_{};
  std::unique_ptr<Model> first_root_model_{};
};

}  // namespace model_tree
}  // namespace util
}  // namespace manipulation
}  // namespace drake
