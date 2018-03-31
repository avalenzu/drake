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
  ProtobufConverter(
      std::function<std::string(std::string)> resolve_relative_path_callback =
          drake::FindResourceOrThrow)
      : resolve_relative_path_callback_(resolve_relative_path_callback){};

  ModelTree ParseModelTreeFromFileOrThrow(const std::string& filename) const;

 private:
  proto::ModelTree ParseProtoModelTreeFromFileOrThrow(
      const std::string& filename) const;

  std::string FindAbsoluteFilePathOrThrow(std::string filename) const;

  ModelTree ConvertModelTree(
      const proto::ModelTree& proto_model_tree,
      const drake::optional<proto::ModelTreeNode>& corresponding_proto_node =
          drake::nullopt) const;

  ModelTreeNode ConvertModelTreeNode(
      const proto::ModelTreeNode& proto_node) const;

  std::function<std::string(std::string)> resolve_relative_path_callback_{};
};

}  // namespace model_tree
}  // namespace util
}  // namespace manipulation
}  // namespace drake
