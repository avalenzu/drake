classdef WeightedSquaredError < NonlinearConstraint
  % Quadratic error in tracking a trajectory
  properties(SetAccess = protected)
    Q
  end
  
  properties%(Access = private)
    ny
    nT
    y_nom
  end
  
  methods
    function obj = WeightedSquaredError(t,Q,y_nom_traj)
      % @param y_nom - ny x nT 
      %t = unique(t(:))';
      if(~isnumeric(t))
        error('Drake:WeightedSquaredError:NonNumericT', ...
              ':t should be numeric');
      end
      nT = numel(t);
      ny = size(y_nom_traj,1);
      if(~isnumeric(Q) || size(Q,1) ~= ny || size(Q,2) ~= ny || any(eig(Q))<0)
        error('Drake:WeightedSquaredError:BadQ', ...
              'Q should be a nq x nq PSD matrix');
      end
      obj = obj@NonlinearConstraint(-inf,inf,ny*nT);
      obj.nT = length(t);
      obj.ny = ny;
      %valuecheck(obj.nT,size(q_nom,2));
      %if(~isnumeric(q_nom))
        %error('Drake:CubicPostureError:q_nom should be numeric');
      %end
      obj.y_nom = y_nom_traj.eval(t);
      obj.Q = Q;
    end
    
    function [c,dc] = eval(obj,x)
      % @param x    -- A double array of size ny*nT. 
      % 
      %   x = [y(:,1);y(:,2);...;y(:,nT)]
      %
      y = reshape(x,obj.ny,obj.nT);
      yerr = y-obj.y_nom;
      Qyerr = obj.Q*yerr;
      c = sum(sum(yerr.*Qyerr))/obj.nT;
      dc = reshape(2*Qyerr,1,[])/obj.nT;
    end
  end
end
