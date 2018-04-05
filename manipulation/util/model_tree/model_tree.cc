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

void ModelTree::AddModel(const std::string& name, const ModelFile& model_file) {
  if (first_root_model_) {
  } else {
  }
}

}  // namespace model_tree
}  // namespace util
}  // namespace manipulation
}  // namespace drake
