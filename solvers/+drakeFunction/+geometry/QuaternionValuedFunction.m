classdef QuaternionValuedFunction < drakeFunction.DrakeFunction
  methods
    function obj = QuaternionValuedFunction(input_frame)
      import drakeFunction.frames.*
      obj = obj@drakeFunction.DrakeFunction(input_frame, Quaternion());
    end
    function fcn = mtimes(obj, other)
      import drakeFunction.geometry.*
      if isa(obj, 'QuaternionValuedFunction') && isa(other, 'QuaternionValuedFunction')
        fcn = compose(QuaternionProduct(), [obj; other]);
      elseif isa(obj, 'QuaternionValuedFunction') && isa(other, 'R3ValuedFunction')
        fcn = compose(QuaternionRotateVector(), [obj; other]);
      else
        fcn = mtimes@drakeFunction.DrakeFunction(obj, other);
      end
    end
    
    function fcn = conjugate(obj)
      import drakeFunction.geometry.*
      fcn = compose(QuaternionConjugate(), obj);
    end
  end
end