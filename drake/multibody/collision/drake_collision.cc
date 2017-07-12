#include "drake/multibody/collision/drake_collision.h"

#include "drake/multibody/collision/collision_filter.h"

#ifdef BULLET_COLLISION
#include "drake/multibody/collision/bullet_model.h"
#endif

#include "drake/common/drake_assert.h"
#include "drake/multibody/collision/unusable_model.h"
#include "drake/multibody/collision/fcl_model.h"

using std::unique_ptr;

namespace DrakeCollision {

enum ModelType {
  kUnusable = 0,
  kFcl = 1,
#ifdef BULLET_COLLISION
  kBullet = 2
#endif
};

unique_ptr<Model> newModel(ModelType type) {
  switch (type) {
    case (kUnusable): {
      return unique_ptr<Model>(new UnusableModel());
      break;
    }
    case (kFcl): {
      return unique_ptr<Model>(new FCLModel());
      break;
    }
#ifdef BULLET_COLLISION
    case kBullet: {
      return unique_ptr<Model>(new BulletModel());
      break;
    }
#endif
    default:
      DRAKE_ABORT_MSG("Unexpected collision model type.");
  }
}

}  // namespace DrakeCollision
