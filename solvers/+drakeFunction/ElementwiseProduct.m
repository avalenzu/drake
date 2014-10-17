classdef ElementwiseProduct < drakeFunction.DrakeFunction
  methods
    function obj = ElementwiseProduct(frame1,frame2,output_frame)
      % obj = drakeFunction.ElementwiseProduct(frame1,frame2,output_frame)
      %   returns a DrakeFunction object representing the elementwise product
      %   of a vector from frame1 and another from frame2
      %
      %   @param frame1         --  CoordinateFrame from which the first input
      %                             is drawn
      %   @param frame2         --  CoordinateFrame from which the second input
      %                             is drawn
      %                             @optional 
      %                             @default frame1
      %   @param output_frame   --  CoordinateFrame representing the range
      %                             @optional 
      %                             @default frame1
      if nargin < 2 || isempty(frame2)
        frame2 = frame1;
      end
      if nargin < 3 || isempty(output_frame)
        output_frame = frame1;
      end
      input_frame = MultiCoordinateFrame({frame1,frame2});
      obj = obj@drakeFunction.DrakeFunction(input_frame,output_frame);
    end
    function [f,df] = eval(obj,x)
      n = obj.getNumInputs()/2;
      x1 = x(1:n);
      x2 = x(n+(1:n));
      f = x1.*x2;
      df = [diag(x2),diag(x1)];
    end

    function obj = setSparsityPattern(obj)
      n_output = obj.getOutputFrame().dim;
      obj.iCfun = repmat((1:n_output)',2,1);
      obj.jCvar = (1:2*n_output)';
    end
  end
end
