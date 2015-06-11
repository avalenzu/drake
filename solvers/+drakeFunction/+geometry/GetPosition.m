classdef GetPosition < drakeFunction.geometry.R3ValuedFunction & drakeFunction.Linear
  methods
    function obj = GetPosition()
      import drakeFunction.frames.*
      obj = obj@drakeFunction.geometry.R3ValuedFunction(RigidTransform());
      A = [eye(3), zeros(3,4)];
      obj = obj@drakeFunction.Linear(RigidTransform(), realCoordinateSpace(3), A);
    end
  end
end