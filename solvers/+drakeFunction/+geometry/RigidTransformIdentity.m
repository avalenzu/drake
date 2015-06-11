classdef RigidTransformIdentity < drakeFunction.geometry.RigidTransformValuedFunction & drakeFunction.Identity
  methods
    function obj = RigidTransformIdentity()
      import drakeFunction.frames.*
      obj = obj@drakeFunction.geometry.RigidTransformValuedFunction(RigidTransform());
      obj = obj@drakeFunction.Identity(RigidTransform());
    end
  end
end