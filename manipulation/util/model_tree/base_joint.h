#pragma once
#pragma once

#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"
#include "drake/manipulation/util/model_tree/model.h"
#include "drake/math/transform.h"
#include "drake/multibody/joints/floating_base_types.h"

namespace drake {
namespace manipulation {
namespace util {
namespace model_tree {

/** Describes how a model tree node is attached to the tree.
 */
// TODO(avalenzu): Add a FindBodyOrFrame() method to RigidBodyTree and get rid
// of this nonsense.
struct BodyOrFrameName {
  std::string body_or_frame_name{};
  bool is_frame{false};
};

bool operator==(const BodyOrFrameName& name_0, const BodyOrFrameName& name_1);

class BaseJoint {
 public:
  BaseJoint(const BodyOrFrameName& parent_body_or_frame_name,
            const multibody::joints::FloatingBaseType& type,
            const Transform<double>& X_PJ);

  const Model* parent_model() const { return parent_model_; }
  const Model* child_model() const { return child_model_; }
  const BodyOrFrameName& parent_body_or_frame_name() const {
    return parent_body_or_frame_name_;
  }
  const multibody::joints::FloatingBaseType type() const { return type_; }
  const Transform<double>& X_PJ() const { return X_PJ_; }
  void set_parent_model(const Model* parent_model) {
    parent_model_ = parent_model;
  };
  void set_child_model(const Model* child_model) { child_model_ = child_model; };
 private
  const Model* parent_model_{};
  const Model* child_model_{};
  const BodyOrFrameName parent_body_or_frame_name_;
  const multibody::joints::FloatingBaseType type_;
  const Transform<double> X_PJ_;
};

}  // namespace model_tree
}  // namespace util
}  // namespace manipulation
}  // namespace drake
