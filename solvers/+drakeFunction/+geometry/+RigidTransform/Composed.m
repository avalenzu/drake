classdef Composed < drakeFunction.geometry.RigidTransformValuedFunction & drakeFunction.Composed
  methods
    function obj = Composed(fcn_outer,fcn_inner)
      obj = obj@drakeFunction.Composed(fcn_outer,fcn_inner);
      typecheck(fcn_outer.getOutputFrame(), 'drakeFunction.frames.RigidTransform');
      obj = obj@drakeFunction.geometry.RigidTransformValuedFunction(fcn_inner.getInputFrame);
    end
  end
end

