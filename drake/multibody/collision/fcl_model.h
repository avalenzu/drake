#pragma once

#include <vector>

#include <Eigen/Dense>
#include <fcl/fcl.h>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/collision/element.h"
#include "drake/multibody/collision/model.h"
#include "drake/multibody/collision/point_pair.h"

namespace drake {
namespace multibody {
namespace collision {

typedef std::unordered_map<ElementId,
                           std::unique_ptr<fcl::CollisionObject<double>>>
    ElementToFclObjMap;

class FclModel : public Model {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FclModel)

  FclModel() {}
  virtual ~FclModel() {}

  void DoAddElement(const Element& element) override;
  bool ClosestPointsAllToAll(const std::vector<ElementId>& ids_to_check,
                             bool use_margins,
                             std::vector<PointPair>* closest_points) override;
  bool ClosestPointsPairwise(const std::vector<ElementIdPair>& id_pairs,
                             bool use_margins,
                             std::vector<PointPair>* closest_points) override;
  bool CollidingPointsCheckOnly(
      const std::vector<Eigen::Vector3d>& input_points,
      double collision_threshold) override;
  bool CollisionRaycast(const Eigen::Matrix3Xd& origins,
                        const Eigen::Matrix3Xd& ray_endpoints, bool use_margins,
                        Eigen::VectorXd* distances,
                        Eigen::Matrix3Xd* normals) override;
  bool ComputeMaximumDepthCollisionPoints(
      bool use_margins, std::vector<PointPair>* points) override;
  std::vector<size_t> CollidingPoints(
      const std::vector<Eigen::Vector3d>& input_points,
      double collision_threshold) override;
  void ClearCachedResults(bool use_margins) override;
  void CollisionDetectFromPoints(
      const Eigen::Matrix3Xd& points, bool use_margins,
      std::vector<PointPair>* closest_points) override;
  void UpdateModel() override;
  bool UpdateElementWorldTransform(
      ElementId, const Eigen::Isometry3d& T_local_to_world) override;

 private:
  struct MeshData {
    std::vector<Eigen::Vector3d> plane_normals_;
    std::vector<double> plane_dis_;
    std::vector<Eigen::Vector3d> points_;
    std::vector<int> polygons_;
  };

  fcl::DynamicAABBTreeCollisionManager<double> broadphase_manager_;
  ElementToFclObjMap fcl_collision_objects_;
  // The fcl::Convex class does not own its data, so we store it here.
  std::vector<MeshData> fcl_mesh_data_{};
};

}  // namespace collision
}  // namespace multibody
}  // namespace drake
