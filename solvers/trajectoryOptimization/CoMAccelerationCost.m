classdef CoMAccelerationCost < NonlinearConstraint
  % Penalize sum_i comddot(:,i)'*Q*comddot(:,i) given comddot(:,i) =
  % compp(:,i)*sdot(i)^2+comp(:,i)*(nT-1)/2*(sdot(i+1)^2-sdot(i)^2)
  properties(SetAccess = protected)
    Q % A 3 x 3 PSD matrix
    nT % number of knot points
    sdotsquare % The square of the time scaling function s
    comp_idx % A 3*obj.nT matrix. x(comp_idx(:,i)) is the first derivative of CoM w.r.t time scaling function s at i'th knot point
    compp_idx % A 3*obj.nT matrix. x(comp_idx(:,i)) is the second derivative of CoM w.r.t time scaling function s at i'th knot point
  end
  
  properties(Access = protected)
    sdotsquare_diff;
    delta_s;
  end
  methods
    function obj = CoMAccelerationCost(Q,nT,sdotsquare)
      % @param Q   A 3 x 3 PSD matrix
      % @param nT   The number of knot points
      % @param sdotsquare   The square of the time scaling function s
      obj = obj@NonlinearConstraint(-inf,inf,6*nT);
      sizecheck(sdotsquare,[1,nT]);
      obj.nT = nT;
      if(any(sdotsquare<=0))
        error('sdotsquare should be positive');
      end
      obj.sdotsquare = sdotsquare;
      sizecheck(Q,[3,3]);
      if(any(eig(Q)<0))
        error('Q should be PSD');
      end
      obj.Q = Q;
      obj.comp_idx = reshape(1:3*obj.nT,3,obj.nT);
      obj.compp_idx = 3*obj.nT+reshape(1:3*obj.nT,3,obj.nT);
      obj.sdotsquare_diff = diff(sdotsquare,1);
      obj.sdotsquare_diff = [obj.sdotsquare_diff obj.sdotsquare_diff(end)];
      obj.delta_s = 1/(obj.nT-1);
    end
    
    function [c,dc] = eval(obj,x)
      % @param x = [comp(:);compp(:)];
      comp = reshape(x(obj.comp_idx),3,obj.nT);
      compp = reshape(x(obj.compp_idx),3,obj.nT);
      comddot = compp.*bsxfun(@times,obj.sdotsquare,ones(3,1))+comp.*(bsxfun(@times,obj.sdotsquare_diff,1/(2*obj.delta_s)*ones(3,1)));
      dcomddot_row = [(1:3*obj.nT)';(1:3*obj.nT)'];
      dcomddot_col = [obj.compp_idx(:);obj.comp_idx(:)];
      dcomddot_val = [reshape(bsxfun(@times,obj.sdotsquare,ones(3,1)),[],1);...
        reshape(bsxfun(@times,obj.sdotsquare_diff,1/(2*obj.delta_s)*ones(3,1)),[],1)];
      dcomddot = sparse(dcomddot_row,dcomddot_col,dcomddot_val,3*obj.nT,6*obj.nT);
      c = sum(sum(comddot.*(obj.Q*comddot)));
      dc = reshape((2*comddot'*obj.Q)',1,3*obj.nT)*dcomddot;
    end
  end
end