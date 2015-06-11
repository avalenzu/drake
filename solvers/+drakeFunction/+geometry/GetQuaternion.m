classdef GetQuaternion < drakeFunction.geometry.QuaternionValuedFunction & drakeFunction.Linear
  methods
    function obj = GetQuaternion()
      import drakeFunction.frames.*
      obj = obj@drakeFunction.geometry.QuaternionValuedFunction(RigidTransform());
      A = [zeros(4,3), eye(4)];
      obj = obj@drakeFunction.Linear(RigidTransform(), Quaternion(), A);
    end
  end
end