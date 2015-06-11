classdef R3ValuedFunction < drakeFunction.DrakeFunction
  methods
    function obj = R3ValuedFunction(input_frame)
      import drakeFunction.frames.*
      obj = obj@drakeFunction.DrakeFunction(input_frame, realCoordinateSpace(3));
    end
  end
end