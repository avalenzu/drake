classdef WeightedSquaredFirstDiffError < WeightedSquaredError
  % Quadratic error in tracking the first derivative of a trajectory
  %
  properties%(Access = private)
    dt
  end
  
  methods
    function obj = WeightedSquaredFirstDiffError(t,Q,int_of_y_nom_traj)
      % @param y_nom - ny x nT 
      %t = unique(t(:))';
      y_nom_traj = fnder(int_of_y_nom_traj);
      dt = diff(t);
      t_mid = t(1:(end-1)) + dt/2;
      obj = obj@WeightedSquaredError(t_mid,Q,y_nom_traj);
      obj.dt = reshape(dt,1,obj.nT);
      obj.xdim = obj.ny*(obj.nT+1);
      obj = obj.setSparseStructure(ones(obj.xdim,1),(1:obj.xdim)');
    end
    
    function [c,dc] = eval(obj,x)
      % @param x    -- A double array of size ny*nT. 
      % 
      %   x = [y(:,1);y(:,2);...;y(:,nT)]
      %
      Y = reshape(x,obj.ny,obj.nT+1);
      idx_plus = 2:(obj.nT+1);
      idx_minus = 1:obj.nT;
      Y_plus = Y(:,idx_plus);
      Y_minus = Y(:,idx_minus);
      dY_plus_dx  = [zeros(numel(Y_plus),obj.ny),eye(numel(Y_plus))];
      dY_minus_dx = [eye(numel(Y_minus)),zeros(numel(Y_minus),obj.ny)];
      dt_mat = repmat(obj.dt,obj.ny,1);
      %y = bsxfun(@rdivide,diff(Y,[0,1]),obj.dt);
      y = (Y_plus - Y_minus)./dt_mat;
      dydY_plus = bsxfun(@times,reshape(1./dt_mat,numel(y),1),eye(numel(y)));
      dydY_minus = bsxfun(@times,reshape(1./dt_mat,numel(y),1),-eye(numel(y)));
      dydx = dydY_plus*dY_plus_dx + dydY_minus*dY_minus_dx;
      [c,dcdy] = eval@WeightedSquaredError(obj,y(:));
      %yerr = y-obj.y_nom;
      %Qyerr = obj.Q*yerr;
      %c = sum(sum(yerr.*Qyerr))/obj.nT;
      %dcdy = (reshape(2*Qyerr,1,[]))/obj.nT;
      dc = dcdy*dydx;
    end
  end
end
