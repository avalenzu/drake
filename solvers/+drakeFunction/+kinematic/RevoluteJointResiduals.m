classdef RevoluteJointResiduals < drakeFunction.Concatenated
  methods
    function obj = RevoluteJointResiduals(Ttree, joint_axis)
      import drakeFunction.frames.*
      import drakeFunction.geometry.*
      import drakeFunction.kinematic.*
      
      parent_to_world = RigidTransformIdentity();
      child_to_world = RigidTransformIdentity();
      child_to_parent = parent_to_world.inv()*child_to_world;
      joint_transform = RotationAboutFixedAxis(joint_axis);
      tree_transform = ConstantRigidTransform.fromHomogenousTransform(Ttree);
      joint_transform = tree_transform*joint_transform;
      quat_smooth_distance = QuatSmoothDistance();
      relative_quat_residual = quat_smooth_distance([child_to_parent.getQuaternion(); joint_transform.getQuaternion()]);

      relative_position_residual = minus(child_to_parent.getPosition(), joint_transform.getPosition());
%       relative_position_residual = relative_position_residual.addInputFrame(relative_quat_residual.getInputFrame.getFrameByNum(2));
      fcns = {relative_position_residual, relative_quat_residual};
      obj = obj@drakeFunction.Concatenated(fcns, true);
    end
  end
end
