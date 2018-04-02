#pragma once

#include <functional>
#include <string>

#include "drake/common/find_resource.h"
#include "drake/manipulation/util/model_tree/model_tree.pb.h"
#include "drake/manipulation/util/model_tree/model_tree_node.h"

namespace drake {
namespace manipulation {
namespace util {
namespace model_tree {

class ProtobufConverter {
 public:
  virtual ~ProtobufConverter(){};

  ModelTree ParseModelTreeFromFileOrThrow(const std::string& filename) const;

 protected:
  virtual std::string DoResolveRelativePathOrThrow(
      const std::string& relative_path) const;

 private:
  proto::ModelTree ParseProtoModelTreeFromFileOrThrow(
      const std::string& filename) const;

  std::string FindAbsoluteFilePathOrThrow(const std::string& filename) const;

  ModelTree ConvertModelTree(
      const proto::ModelTree& proto_model_tree,
      const drake::optional<proto::ModelTreeNode>& corresponding_proto_node =
          drake::nullopt) const;

  ModelTreeNode ConvertModelTreeNode(
      const proto::ModelTreeNode& proto_node) const;
};

}  // namespace model_tree
}  // namespace util
}  // namespace manipulation
}  // namespace drake
