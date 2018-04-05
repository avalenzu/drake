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
  Model(){};
  Model(const std::string name, const ModelFile& model_file)
      : name_(name), model_file_(model_file) {}

  const std::string& name() const { return name_; }
  const ModelFile& model_file() const { return model_file_; }
  const BaseJoint* base_joint() const { return base_joint_.get(); }
  const Model* next_model() const;

  void add_name_prefix(const std::string& name);
  Model* mutable_next_model();

  Model* AddChild(std::unique_ptr<Model> child,
                  std::unique_ptr<BaseJoint> base_joint);

  Model* AddChild(std::unique_ptr<Model> child);

 private:
  std::string name_{};
  ModelFile model_file_{};
  std::unique_ptr<Model> next_sibling_{};
  std::unique_ptr<Model> first_child_{};
  std::unique_ptr<BaseJoint> base_joint_{};
};

}  // namespace model_tree
}  // namespace util
}  // namespace manipulation
}  // namespace drake
