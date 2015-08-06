classdef StormerVerletResiduals < drakeFunction.DrakeFunction
  %STORMERVERLETRESIDUALS Residuals for Störmer-Verlet integration
  %   Returns 
  %     r[n+1] - (r[n] + h*v[n+1/2])
  %     v[n+1] - (v[n+1/2] + h/2*F[n+1])
  %   where
  %     v[n+1/2] = v[n] + h/2*F[n]
  %   for n = 1, ... , N-1
  
  properties (SetAccess = immutable)
    N
    dim_r
    r_inds
    v_inds
    F_inds
    h_inds
    r0_inds
    v0_inds
    F0_inds
    r1_inds
    v1_inds
    F1_inds
    dr0
    dr1
    dv0
    dv1
    dF0
    dF1
    dh
  end
  
  methods
    function obj = StormerVerletResiduals(dim_r, N)
      dim_input = 3*dim_r*N + N-1;
      dim_output = 2*dim_r*(N-1);
      obj = obj@drakeFunction.DrakeFunction(dim_input, dim_output);
      obj.dim_r = dim_r;
      obj.N = N;
      obj.r_inds = reshape(1:obj.dim_r*obj.N, [obj.dim_r, N]);
      obj.r0_inds = obj.r_inds(:, 1:obj.N-1);
      obj.r1_inds = obj.r_inds(:, 2:obj.N);
      
      obj.v_inds = obj.dim_r*obj.N + obj.r_inds;
      obj.v0_inds = obj.v_inds(:, 1:obj.N-1);
      obj.v1_inds = obj.v_inds(:, 2:obj.N);
      
      obj.F_inds = obj.dim_r*obj.N + obj.v_inds;
      obj.F0_inds = obj.F_inds(:, 1:obj.N-1);
      obj.F1_inds = obj.F_inds(:, 2:obj.N);
      
      obj.h_inds = 3*obj.dim_r*obj.N + (1:obj.N-1);
      
      obj.dr0 = sparse(1:numel(obj.r0_inds), obj.r0_inds, 1, numel(obj.r0_inds), obj.dim_input);
      obj.dr1 = sparse(1:numel(obj.r1_inds), obj.r1_inds, 1, numel(obj.r1_inds), obj.dim_input);
      obj.dv0 = sparse(1:numel(obj.v0_inds), obj.v0_inds, 1, numel(obj.v0_inds), obj.dim_input);
      obj.dv1 = sparse(1:numel(obj.v1_inds), obj.v1_inds, 1, numel(obj.v1_inds), obj.dim_input);
      obj.dF0 = sparse(1:numel(obj.F0_inds), obj.F0_inds, 1, numel(obj.F0_inds), obj.dim_input);
      obj.dF1 = sparse(1:numel(obj.F1_inds), obj.F1_inds, 1, numel(obj.F1_inds), obj.dim_input);
      obj.dh = sparse(1:numel(obj.h_inds), obj.h_inds, 1, numel(obj.h_inds), obj.dim_input);
    end
    
    function [f, df] = eval(obj, x)
      r0 = x(obj.r0_inds);
      r1 = x(obj.r1_inds);
      v0 = x(obj.v0_inds);
      v1 = x(obj.v1_inds);
      F0 = x(obj.F0_inds);
      F1 = x(obj.F1_inds);
      h = x(obj.h_inds);
      
      v_mid = v0 + 0.5*bsxfun(@times, h', F0);
      dv_mid = obj.dv0 + ...
               0.5*(sparse(1:obj.dim_r*(obj.N-1), repmat(1:obj.N-1, obj.dim_r, 1), F0)*obj.dh + ...
                    kron(diag(h), speye(obj.dim_r))*obj.dF0);
       r1_des = r0 + bsxfun(@times, h', v_mid);
       dr1_des = obj.dr0 + ...
                 (sparse(1:obj.dim_r*(obj.N-1), repmat(1:obj.N-1, obj.dim_r, 1), v_mid)*obj.dh + ...
                  kron(diag(h), speye(obj.dim_r))*dv_mid);
       r_res = r1 - r1_des;
       dr_res = obj.dr1 - dr1_des;
       
       v1_des = v_mid + 0.5*bsxfun(@times, h', F1);
       dv1_des = dv_mid + ...
                 0.5*(sparse(1:obj.dim_r*(obj.N-1), repmat(1:obj.N-1, obj.dim_r, 1), F1)*obj.dh + ...
                      kron(diag(h), speye(obj.dim_r))*obj.dF1);
       v_res = v1 - v1_des;
       dv_res = obj.dv1 - dv1_des;
       f = [r_res(:); v_res(:)];
       df = [dr_res; dv_res];
    end
  end
  
end

