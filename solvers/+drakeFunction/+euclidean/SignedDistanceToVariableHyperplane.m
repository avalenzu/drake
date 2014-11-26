classdef SignedDistanceToVariableHyperplane < drakeFunction.DrakeFunction
  % DrakeFunction representing the distance from a point to a hyperplane
  properties (SetAccess = protected)
    dim_points
    n_points
    df_da
  end

  methods
    function obj = SignedDistanceToVariableHyperplane(point_frame,n_points)
      % obj = SignedDistanceToHyperplane(origin,normal) returns a 
      %   SignedDistanceToHyperplane object 
      %
      %   @param origin -- Point object representing the origin of the
      %                    hyperplane
      %   @param normal -- Point object representing the (possibly non-unit)
      %                    normal of the hyperplane. Must be in the same frame
      %                    as origin
      %
      %   @retval obj   -- drakeFunction.euclidean.SignedDistanceToHyperplane
      %                    object
      R1 = drakeFunction.frames.realCoordinateSpace(1);
      X_frame = MultiCoordinateFrame.constructFrame(repmat({point_frame},1,n_points));
      input_frame = MultiCoordinateFrame({point_frame,R1,X_frame});
      output_frame = drakeFunction.frames.realCoordinateSpace(n_points);
      obj = obj@drakeFunction.DrakeFunction(input_frame,output_frame);
      obj.n_points = n_points;
      obj.dim_points = point_frame.dim;
      obj.df_da = -ones(n_points,1);
      obj = obj.setSparsityPattern();
    end
    function [f,df] = eval(obj,x)
      [alpha,a,X] = obj.input_frame.splitCoordinates(x);
      X = reshape(X,obj.dim_points,obj.n_points);
      f = X'*alpha - a;
      df = [X',obj.df_da,kron(eye(obj.n_points),alpha')];
    end
    function obj = setSparsityPattern(obj)
      if isempty(obj.n_points), return; end
      C = [ones(obj.n_points,obj.dim_points+1),kron(eye(obj.n_points),[1,1,1])];
      [obj.iCfun,obj.jCvar] = find(C);
    end
  end
end
