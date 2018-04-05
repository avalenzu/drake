#pragma once

#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"
#include "drake/manipulation/util/model_tree/base_joint.h"
#include "drake/math/transform.h"
#include "drake/multibody/joints/floating_base_types.h"

namespace drake {
namespace manipulation {
namespace util {
namespace model_tree {

/** Enumerates supported model file types.
 */
enum class ModelFileType { kUrdf, kSdf };

/** Describes a model file
 */
struct ModelFile {
  // ModelFile(const std::string& absolute_path_in, ModelFileType type_in)
  //: absolute_path(absolute_path_in), type(type_in) {}
  std::string absolute_path{};
  ModelFileType type{};
};

bool operator==(const ModelFile& file_0, const ModelFile& file_1);

class Model {
 public:
  Model(const std::string& name, const ModelFile& model_file);

  const ModelFile& model_file() const { return model_file_; }
  const Model* parent() const;
  const Model* next_sibling() const { return next_sibling_; }
  const Model* first_child() const;
  const BaseJoint* base_joint() const { return base_joint_; }

  void AddChild(Model* child, BaseJoint* base_joint);
  Model* MutableLastChild();

 private:
  ModelFile model_file_;
  Model* parent_;
  Model* next_sibling_;
  BaseJoint* first_child_joint;
  BaseJoint* base_joint_;
};

}  // namespace model_tree
}  // namespace util
}  // namespace manipulation
}  // namespace drake
