#include "drake/multibody/collision/fcl_model.h"

#include <Eigen/Dense>
#include <fcl/fcl.h>

#include "drake/common/drake_assert.h"
#include "drake/common/unused.h"

using Eigen::Isometry3d;
using Eigen::Vector3d;

namespace drake {
namespace multibody {
namespace collision {

/// @brief Distance data stores the distance request and the result given by distance algorithm.
struct CollisionData
{
  /// @brief Collision request
  fcl::CollisionRequestd request;

  /// @brief Vector of distance results
  std::vector<PointPair>* closest_points{nullptr};
};

bool collisionPointsFunction(fcl::CollisionObjectd* fcl_object_A,
    fcl::CollisionObjectd* fcl_object_B, void* callback_data)
{
  auto element_A = static_cast<Element*>(fcl_object_A->getUserData());
  auto element_B = static_cast<Element*>(fcl_object_B->getUserData());
  if (element_A && element_B && element_A->CanCollideWith(element_B)) {
      // Unpack the callback data
      auto* collision_data = static_cast<CollisionData*>(callback_data);
      const fcl::CollisionRequestd& request = collision_data->request;
      fcl::CollisionResultd result;

      // Perform nearphase collision detection
      fcl::collide(fcl_object_A, fcl_object_B, request, result);

      // Process the contact points
      std::vector<fcl::Contactd> contacts;
      result.getContacts(contacts);

      for (auto contact : contacts) {
        // Signed distance is negative when penetration depth is positive
        double d_QP = -contact.penetration_depth;
        // Define the normal as the unit vector from Q to P (opposite
        // convention from FCL)
        Vector3d n_QP = -contact.normal;
        if (element_B->getShape() == DrakeShapes::MESH) {
          if (element_A->getShape() == DrakeShapes::SPHERE) {
            // Penetration depth sign convention is reversed for sphere-mesh contact???
            d_QP = -d_QP;
          }
          n_QP *= -1;
        }

        // FCL returns a single contact point, but PointPair expects two
        const Vector3d p_WP{contact.pos + 0.5*d_QP*n_QP};
        const Vector3d p_WQ{contact.pos - 0.5*d_QP*n_QP};

        // Transform the closest points to their respective body frames.
        // Let element A be on body C and element B
        //const Isometry3d X_CA = element_A->getLocalTransform();
        //const Isometry3d X_DB = element_B->getLocalTransform();
        //const Isometry3d X_AW = element_A->getWorldTransform().inverse();
        //const Isometry3d X_BW = element_B->getWorldTransform().inverse();
        //const Vector3d p_CP = X_CA * X_AW * p_WP;
        //const Vector3d p_DQ = X_DB * X_BW * p_WQ;

        collision_data->closest_points->emplace_back(element_A, element_B, 
            p_WP, p_WQ, n_QP, d_QP);
      }
  }
  return false; // Check all pairs provided by the broadphase
}

void FclModel::DoAddElement(const Element& element) {
  ElementId id = element.getId();

  if (id != 0) {
    // Create the fcl::CollisionObject
    std::shared_ptr<fcl::CollisionGeometryd> fcl_geometry;

    switch (element.getShape()) {
      case DrakeShapes::SPHERE: {
        const auto sphere =
            static_cast<const DrakeShapes::Sphere&>(element.getGeometry());
        fcl_geometry = std::shared_ptr<fcl::CollisionGeometryd>(new fcl::Sphered(sphere.radius));
      } break;
      default:
        DRAKE_ABORT_MSG("Not implemented.");
        break;
    }
    
    if (fcl_geometry != nullptr) {
      std::unique_ptr<fcl::CollisionObjectd> fcl_object{new fcl::CollisionObjectd(fcl_geometry)};

      // Store a pointer to the Element in the fcl collision object
      fcl_object->setUserData(FindMutableElement(id));

      // NOTE: The FCL collision object is assigned the Drake element's
      // world transform.  This will be the *only* time that anchored collision
      // objects will have their world transform set.  This code assumes that
      // the world transform on the corresponding input Drake element has
      // already been properly set. (See RigidBodyTree::CompileCollisionState.)
      fcl_object->setTransform(element.getWorldTransform());

      // Register the object with the collision manager
      broadphase_manager_.registerObject(fcl_object.get());
      // Take ownership of FCL collision object
      fcl_collision_objects_.insert(std::make_pair(id, move(fcl_object)));
    }
  }
}

void FclModel::UpdateModel() {
  broadphase_manager_.update();
}

bool FclModel::UpdateElementWorldTransform(
    ElementId id, const Isometry3d& T_local_to_world) {
  const bool element_exists(
      Model::UpdateElementWorldTransform(id, T_local_to_world));
  if (element_exists) {
    fcl_collision_objects_[id]->setTransform(FindElement(id)->getWorldTransform());
  }
  return element_exists;
}

bool FclModel::ClosestPointsAllToAll(const std::vector<ElementId>& ids_to_check,
                                     bool use_margins,
                                     std::vector<PointPair>* closest_points) {
  drake::unused(ids_to_check, use_margins, closest_points);
  DRAKE_ABORT_MSG("Not implemented.");
  return false;
}

bool FclModel::ComputeMaximumDepthCollisionPoints(
    bool use_margins, std::vector<PointPair>* points) {
  CollisionData collision_data;
  collision_data.closest_points = points;
  collision_data.request.enable_contact = true;
  collision_data.request.num_max_contacts = 1e3;
  broadphase_manager_.collide(static_cast<void*>(&collision_data), collisionPointsFunction);
  return true;
}

bool FclModel::ClosestPointsPairwise(const std::vector<ElementIdPair>& id_pairs,
                                     bool use_margins,
                                     std::vector<PointPair>* closest_points) {
  drake::unused(id_pairs, use_margins, closest_points);
  DRAKE_ABORT_MSG("Not implemented.");
  return false;
}

void FclModel::CollisionDetectFromPoints(
    const Eigen::Matrix3Xd& points, bool use_margins,
    std::vector<PointPair>* closest_points) {
  drake::unused(points, use_margins, closest_points);
  DRAKE_ABORT_MSG("Not implemented.");
}

void FclModel::ClearCachedResults(bool use_margins) {
  drake::unused(use_margins);
  DRAKE_ABORT_MSG("Not implemented.");
}

bool FclModel::CollisionRaycast(const Eigen::Matrix3Xd& origins,
                                const Eigen::Matrix3Xd& ray_endpoints,
                                bool use_margins, Eigen::VectorXd* distances,
                                Eigen::Matrix3Xd* normals) {
  drake::unused(origins, ray_endpoints, use_margins, distances, normals);
  DRAKE_ABORT_MSG("Not implemented.");
  return false;
}

bool FclModel::CollidingPointsCheckOnly(
    const std::vector<Eigen::Vector3d>& input_points,
    double collision_threshold) {
  drake::unused(input_points, collision_threshold);
  DRAKE_ABORT_MSG("Not implemented.");
  return false;
}

std::vector<size_t> FclModel::CollidingPoints(
    const std::vector<Eigen::Vector3d>& input_points,
    double collision_threshold) {
  drake::unused(input_points, collision_threshold);
  DRAKE_ABORT_MSG("Not implemented.");
  return std::vector<size_t>();
}

}  // namespace collision
}  // namespace multibody
}  // namespace drake
