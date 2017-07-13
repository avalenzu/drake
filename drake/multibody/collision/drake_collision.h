#pragma once

#include <memory>

#include "drake/multibody/collision/model.h"

namespace DrakeCollision {

enum ModelType {
  kUnusable = 0,
  kFcl = 1,
#ifdef BULLET_COLLISION
  kBullet = 2
#endif
};

#ifdef BULLET_COLLISION
std::unique_ptr<Model> newModel(ModelType type = kBullet);
#else
std::unique_ptr<Model> newModel(ModelType type = kUnusable);
#endif
}  // namespace DrakeCollision
