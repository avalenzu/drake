classdef TransformPoint < drakeFunction.DrakeFunction
  properties
    pt;
  end
  methods
    function obj = TransformPoint(pt)
      input_frame = drakeFunction.frames.spatialTransform();
      output_frame = drakeFunction.frames.realCoordinateSpace(3);
      obj = obj@drakeFunction.DrakeFunction(input_frame,output_frame);
      obj.pt = pt;
    end
    function [f,df] = eval(obj,x)
      [quat,r] = obj.input_frame.splitCoordinates(x);
      dquat = [eye(4), zeros(4,3)];
      dr = [zeros(3,4), eye(3)];
      [pt_rotated,dpt_rotated_dquat_pt] = quatRotateVec(quat,obj.pt);
      dpt_rotated = dpt_rotated_dquat_pt(:,1:4)*dquat;
      f = pt_rotated + r;
      df = dpt_rotated + dr;
    end
  end

end
