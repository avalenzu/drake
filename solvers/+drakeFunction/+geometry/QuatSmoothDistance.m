classdef QuatSmoothDistance<drakeFunction.DrakeFunction
  % Compute the approximate distance between two quaternions as
  %
  %   d = 1-(quat'*quat_des)^2
  %
  methods
    function obj = QuatSmoothDistance()
      input_frame = MultiCoordinateFrame.constructFrame(repmat({drakeFunction.frames.Quaternion()},1,2));
      output_frame = drakeFunction.frames.realCoordinateSpace(1);
      obj = obj@drakeFunction.DrakeFunction(input_frame,output_frame);
    end
    
    function [distance,ddistance] = eval(obj,x)
      % Compute the approximate distance between two quaternions as
      %
      %   d = 1-(quat'*quat_des)^2
      %
      [quat_A, quat_B] = splitCoordinates(obj.input_frame, x);
      inner_product = quat_A'*quat_B;
      distance = 1 - inner_product^2;
      if nargout > 1
        ddistance = -2*inner_product*[quat_B', quat_A'];
      end
    end
  end
end
