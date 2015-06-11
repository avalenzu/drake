classdef ConstantRigidTransform < drakeFunction.geometry.RigidTransformValuedFunction & drakeFunction.Constant
  methods
    function obj = ConstantRigidTransform(value)
      import drakeFunction.frames.*
      obj = obj@drakeFunction.geometry.RigidTransformValuedFunction(Null());
      obj = obj@drakeFunction.Constant(value, RigidTransform());
    end
  end
  
  methods (Static)
    function obj = fromHomogenousTransform(T)
      import drakeFunction.geometry.*
      assert(isHT(T));
      value = [T(1:3,4); rotmat2quat(T(1:3,1:3))];
      obj = ConstantRigidTransform(value);
    end
  end
end