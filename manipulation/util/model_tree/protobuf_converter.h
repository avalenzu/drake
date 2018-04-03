#pragma once

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
  virtual ~ProtobufConverter() {}

  /** Instantiate a ModelTree from a text-format Protobuf file containing a
   * proto::ModelTree message.
   * @throws std::runtime_error if any of the referenced model files cannot be
   *         found.
   */
  ModelTree ParseModelTreeFromFile(const std::string& filename) const;

 protected:
  /** Returns the absolute path corresponding to a given relative path. The
   * default implementation calls FindResourceOrThrow(). Child classes should
   * override this method if they wish to use other methods for resolving
   * relative paths.
   * @throws std::runtime_error if `relative_path` cannot be resolved to an
   *         absolute path.
   */
  virtual std::string DoResolveRelativePathOrThrow(
      const std::string& relative_path) const;

 private:
  proto::ModelTree ParseProtoModelTreeFromFile(
      const std::string& filename) const;

  std::string FindAbsoluteFilePath(const std::string& filename) const;

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
