classdef ConcatenateRigidTransforms < drakeFunction.geometry.RigidTransformValuedFunction & drakeFunction.Concatenated
  methods
    function obj = ConcatenateRigidTransforms()
      import drakeFunction.frames.*
      import drakeFunction.geometry.*
      input_frame = MultiCoordinateFrame(repmat({RigidTransform()}, 1, 2));
      position_fcn = GetQuaternion()*GetPosition();
      position_fcn = plus(position_fcn, GetPosition().addInputFrame(RigidTransform()), true);
      orientation_fcn = GetQuaternion()*GetQuaternion();
      obj = obj@drakeFunction.Concatenated({position_fcn, orientation_fcn}, true);
      obj = obj@drakeFunction.geometry.RigidTransformValuedFunction(input_frame);
    end
  end
end