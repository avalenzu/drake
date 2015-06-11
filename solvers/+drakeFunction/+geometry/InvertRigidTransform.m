classdef InvertRigidTransform < drakeFunction.geometry.RigidTransformValuedFunction & drakeFunction.Concatenated
  methods
    function obj = InvertRigidTransform()
      import drakeFunction.frames.*
      import drakeFunction.geometry.*
      input_frame = RigidTransform();
      orientation_function = GetQuaternion().conjugate();
      position_function = compose(QuaternionRotateVector(), concatenate(orientation_function, -GetPosition(), true));
      obj = obj@drakeFunction.Concatenated({position_function, orientation_function}, true);
      obj = obj@drakeFunction.geometry.RigidTransformValuedFunction(input_frame);
    end
  end
end