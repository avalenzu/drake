classdef RevoluteJointResiduals < drakeFunction.Concatenated
  methods
    function obj = RevoluteJointResiduals(rbm, parent, child, Ttree, joint_axis)
      import drakeFunction.geometry.*
      import drakeFunction.kinematic.*

      relative_position = RelativePosition(rbm, child, parent);
      relative_position_residual = relative_position - Ttree(1:3,4);

      joint_quat = QuatRotationAboutFixedAxis(joint_axis);
      quat_product_with_Ttree = QuatProductWithFixedQuat(rotmat2quat(Ttree(1:3,1:3)));
      quat_child_to_parent = quat_product_with_Ttree(joint_quat);
      relative_quat = RelativeQuaternion(rbm, child, parent);
      quat_smooth_distance = QuatSmoothDistance();
      relative_quat_residual = quat_smooth_distance([relative_quat; quat_child_to_parent]);

      relative_position_residual = relative_position_residual.addInputFrame(quat_child_to_parent.getInputFrame());
      fcns = {relative_position_residual, relative_quat_residual};
      obj = obj@drakeFunction.Concatenated(fcns, true);
    end
  end
end
