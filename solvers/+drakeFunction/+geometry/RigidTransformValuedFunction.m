classdef RigidTransformValuedFunction < drakeFunction.DrakeFunction
  methods
    function obj = RigidTransformValuedFunction(input_frame)
      import drakeFunction.frames.*
      obj = obj@drakeFunction.DrakeFunction(input_frame, RigidTransform());
    end
    function fcn = mtimes(obj, other)
      import drakeFunction.geometry.*
      if isa(obj, 'RigidTransformValuedFunction') && isa(other, 'RigidTransformValuedFunction')
        fcn = compose(ConcatenateRigidTransforms(), [obj; other]);
      else
        fcn = mtimes@drakeFunction.DrakeFunction(obj, other);
      end
    end
    
    function fcn = inv(obj)
      import drakeFunction.geometry.*
      fcn = compose(InvertRigidTransform(), obj);
    end
    
    function fcn = getPosition(obj)
      import drakeFunction.geometry.*
      fcn = compose(GetPosition(), obj);
    end
    
    function fcn = getQuaternion(obj)
      import drakeFunction.geometry.*
      fcn = compose(GetQuaternion(), obj);
    end
    
    function fcn3 = compose(fcn1, fcn2)
      fcn3 = drakeFunction.geometry.RigidTransform.Composed(fcn1,fcn2);
    end
  end
end