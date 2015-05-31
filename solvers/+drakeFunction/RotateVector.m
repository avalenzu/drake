classdef RotateVector < drakeFunction.DrakeFunction
  properties
    vec;
  end
  methods
    function obj = RotateVector(vec)
      input_frame = drakeFunction.frames.quaternion();
      output_frame = drakeFunction.frames.realCoordinateSpace(3);
      obj = obj@drakeFunction.DrakeFunction(input_frame,output_frame);
      obj.vec = vec;
    end
    function [f,df] = eval(obj,quat)
      [f,df_dquat_vec] = quatRotateVec(quat,obj.vec);
      df = df_dquat_vec(:,1:4);
    end
  end

end
