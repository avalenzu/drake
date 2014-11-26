classdef QuatRotateVec < drakeFunction.DrakeFunction
  methods
    function obj = QuatRotateVec()
      input_frame = MultiCoordinateFrame.constructFrame( ...
        {drakeFunction.frames.quaternion(), ...
        drakeFunction.frames.realCoordinateSpace(3)});
      output_frame = drakeFunction.frames.realCoordinateSpace(3);
      obj = obj@drakeFunction.DrakeFunction(input_frame,output_frame);
    end

    function [f,df] = eval(obj,x)
      [q,r] = obj.input_frame.splitCoordinates(x);
      [f,df] = quatRotateVec(q,r);
    end
  end
end
