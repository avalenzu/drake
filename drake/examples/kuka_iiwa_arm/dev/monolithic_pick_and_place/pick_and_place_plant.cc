#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/pick_and_place_plant.h"

#include "optitrack/optitrack_frame_t.hpp"

#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace monolithic_pick_and_place {

using optitrack::optitrack_frame_t;
using manipulation::util::WorldSimTreeBuilder;

namespace {

const char kIiwaUrdf[] =
    "drake/manipulation/models/iiwa_description/urdf/"
    "iiwa14_polytope_collision.urdf";

// The `z` coordinate of the top of the table in the world frame.
// The quantity 0.736 is the `z` coordinate of the frame associated with the
// 'surface' collision element in the SDF. This element uses a box of height
// 0.057m thus giving the surface height (`z`) in world coordinates as
// 0.736 + 0.057 / 2.
const double kTableTopZInWorld = 0.736 + 0.057 / 2;

std::unique_ptr<systems::RigidBodyPlant<double>> BuildCombinedPlant(
    const pick_and_place::SimulatedPlantConfiguration& configuration,
    std::vector<ModelInstanceInfo<double>>* iiwa_instances,
    std::vector<ModelInstanceInfo<double>>* wsg_instances,
    std::vector<ModelInstanceInfo<double>>* object_instances,
    std::vector<ModelInstanceInfo<double>>* table_instances) {
  auto tree_builder = std::make_unique<WorldSimTreeBuilder<double>>();

  // Adds models to the simulation builder. Instances of these models can be
  // subsequently added to the world.
  tree_builder->StoreModel("iiwa", kIiwaUrdf);
  tree_builder->StoreModel("table",
                           "drake/examples/kuka_iiwa_arm/models/table/"
                           "extra_heavy_duty_table_surface_only_collision.sdf");
  tree_builder->StoreModel("wsg",
                           "drake/manipulation/models/wsg_50_description"
                           "/sdf/schunk_wsg_50_ball_contact.sdf");
  for (int i = 0; i < static_cast<int>(configuration.object_models.size());
       ++i) {
    tree_builder->StoreModel("object_" + std::to_string(i),
                             configuration.object_models[i]);
  }
  for (int i = 0; i < static_cast<int>(configuration.table_models.size());
       ++i) {
    tree_builder->StoreModel("table_" + std::to_string(i),
                             configuration.table_models[i]);
  }

  // The tables that the objects sit on.
  for (int i = 0; i < static_cast<int>(configuration.table_poses.size()); ++i) {
    int table_id = tree_builder->AddFixedModelInstance(
        "table_" + std::to_string(i),
        configuration.table_poses[i].translation(),
        drake::math::rotmat2rpy(configuration.table_poses[i].linear()));
    table_instances->push_back(
        tree_builder->get_model_info_for_instance(table_id));
  }
  tree_builder->AddGround();

  for (const auto& robot_base_pose : configuration.robot_base_poses) {
    // Add the arm.
    int robot_base_id = tree_builder->AddFixedModelInstance(
        "iiwa", robot_base_pose.translation(),
        drake::math::rotmat2rpy(robot_base_pose.linear()));
    iiwa_instances->push_back(
        tree_builder->get_model_info_for_instance(robot_base_id));
    // Add the gripper.
    auto frame_ee = tree_builder->tree().findFrame(
        "iiwa_frame_ee", iiwa_instances->back().instance_id);
    auto wsg_frame = frame_ee->Clone(frame_ee->get_mutable_rigid_body());
    wsg_frame->get_mutable_transform_to_body()->rotate(
        Eigen::AngleAxisd(-0.39269908, Eigen::Vector3d::UnitY()));
    wsg_frame->get_mutable_transform_to_body()->translate(
        0.04 * Eigen::Vector3d::UnitY());
    int wsg_id = tree_builder->AddModelInstanceToFrame(
        "wsg", wsg_frame, drake::multibody::joints::kFixed);
    wsg_instances->push_back(tree_builder->get_model_info_for_instance(wsg_id));
    // Add the table that the arm sits on.
    const Isometry3<double> X_WT{
        robot_base_pose *
        Isometry3<double>::TranslationType(0.0, 0.0, -kTableTopZInWorld)};
    tree_builder->AddFixedModelInstance("table", X_WT.translation(),
                                        drake::math::rotmat2rpy(X_WT.linear()));
  }

  for (int i = 0; i < static_cast<int>(configuration.object_poses.size());
       ++i) {
    int object_id = tree_builder->AddFloatingModelInstance(
        "object_" + std::to_string(i),
        configuration.object_poses[i].translation(),
        drake::math::rotmat2rpy(configuration.object_poses[i].linear()));
    object_instances->push_back(
        tree_builder->get_model_info_for_instance(object_id));
  }

  return std::make_unique<systems::RigidBodyPlant<double>>(
      tree_builder->Build());
}

typedef std::tuple<const RigidBody<double>*, Isometry3<double>, int>
    OptitrackBodyInfo;

class MockOptitrackSystem : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MockOptitrackSystem);

  MockOptitrackSystem(const Isometry3<double>& X_WO,
                      const RigidBodyTree<double>& tree,
                      const std::vector<OptitrackBodyInfo> body_info_vector)
      : X_OW_(X_WO.inverse()),
        tree_(tree),
        body_info_vector_(body_info_vector) {
    this->DeclareInputPort(
        systems::kVectorValued,
        tree_.get_num_positions() + tree_.get_num_velocities());
    DeclareAbstractOutputPort(optitrack_frame_t(),
                              &MockOptitrackSystem::OutputOptitrackFrame);
  }

 private:
  void DoCalcUnrestrictedUpdate(
      const systems::Context<double>& context,
      const std::vector<const systems::UnrestrictedUpdateEvent<double>*>& event,
      systems::State<double>* state) const {
    // Extract internal state
    auto internal_state = state->get_mutable_abstract_state<VectorX<double>>(0);
    // Update world state from inputs.
    const systems::AbstractValue* input = this->EvalAbstractInput(context, 0);
    DRAKE_ASSERT(input != nullptr);
    internal_state = input->GetValue<VectorX<double>>();
  }

  void OutputOptitrackFrame(const systems::Context<double>& context,
                            optitrack_frame_t* output) const {
    const VectorX<double> input = this->EvalEigenVectorInput(context, 0);
    VectorX<double> q = input.head(tree_.get_num_positions());
    KinematicsCache<double> cache = tree_.doKinematics(q);

    output->rigid_bodies.clear();
    output->rigid_bodies.reserve(body_info_vector_.size());
    for (const OptitrackBodyInfo& body_info : body_info_vector_) {
      output->rigid_bodies.emplace_back();
      output->rigid_bodies.back().id = std::get<2>(body_info);
      const RigidBody<double>& body = *std::get<0>(body_info);
      const Isometry3<double>& X_BF = std::get<1>(body_info);
      const Isometry3<double> X_WF =
          tree_.CalcFramePoseInWorldFrame(cache, body, X_BF);
      const Isometry3<double> X_OF{X_OW_ * X_WF};
      const Vector3<double>& r_OF = X_OF.translation();
      const Quaternion<double> quat_OF{X_OF.rotation()};
      for (int i : {0, 1, 2}) {
        output->rigid_bodies.back().xyz[i] = r_OF(i);
      }
      // Convert to optitracks X-Y-Z-W order
      output->rigid_bodies.back().quat[0] = quat_OF.x();
      output->rigid_bodies.back().quat[1] = quat_OF.y();
      output->rigid_bodies.back().quat[2] = quat_OF.z();
      output->rigid_bodies.back().quat[3] = quat_OF.w();
    }
    output->num_rigid_bodies = output->rigid_bodies.size();
  }

  const Isometry3<double> X_OW_;
  const RigidBodyTree<double>& tree_;
  const std::vector<OptitrackBodyInfo> body_info_vector_;
};
}  // namespace

PickAndPlacePlant::PickAndPlacePlant(
    const pick_and_place::SimulatedPlantConfiguration& configuration) {
  systems::DiagramBuilder<double> builder;
  std::vector<ModelInstanceInfo<double>> iiwa_instances, wsg_instances,
      box_instances, table_instances;
  std::unique_ptr<systems::RigidBodyPlant<double>> model_ptr =
      BuildCombinedPlant(configuration, &iiwa_instances, &wsg_instances,
                         &box_instances, &table_instances);
  model_ptr->set_normal_contact_parameters(configuration.stiffness,
                                           configuration.dissipation);
  model_ptr->set_friction_contact_parameters(
      configuration.static_friction_coef,
      configuration.dynamic_friction_coef,
      configuration.v_stiction_tolerance);

  auto plant = builder.AddSystem<IiwaAndWsgPlantWithStateEstimator<double>>(
      std::move(model_ptr), iiwa_instances, wsg_instances, box_instances);
  plant->set_name("plant");
}

}  // namespace monolithic_pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
