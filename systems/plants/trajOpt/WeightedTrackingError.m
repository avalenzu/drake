classdef WeightedTrackingError < NonlinearConstraint
  % approximate the posture as a cubic spline, and penalize the cost
  % sum_i (q(:,i)-q_nom(:,i))'*Q*(q(:,i)-q_nom(:,i))+
  % sum_i qdot(:,i)'*Qv*qdot(:,i) +
  % sum_i qddot(:,i)'*Qa*qddot(:,i)
  % [qdot(:,2);qdot(:,3);...;qdot(:,obj.nT-1)] =
  % velocity_mat*[q(:,1);q(:,2);...;q(:,obj.nT)]+velocity_mat_qd0*qdot(:,1)+velocity_mat_qdf*qdot(:,obj.nT)
  % @param Qv      -- A nq x nq PSD matrix
  % [qddot(:,1);qddot(:,2);...;qddot(:,obj.nT)] = accel_mat*[q(:,1);q(:,2);...;q(:,obj.nT)]+
  % accel_mat_qd0*qdot(:,1)+accel_mat_qdf*qdot(:,obj.nT)
  % @param velocity_mat    --A nq*(obj.nT-2) x nq*obj.nT matrix.
  % @param velocity_mat_qd0  -- A nq*(obj.nT-2) x nq matrix
  % @param velocity_mat_qdf  -- A nq*(obj.nT-2) x nq matrix
  % @param accel_mat       -- A nq*obj.nT x nq*obj.nT matrix
  % @param accel_mat_qd0     -- A nq*obj.nT x nq matrix
  % @param accel_mat_qdf     -- A nq*obj.nT x nq matrix
  properties(SetAccess = protected)
    Q
    Qv
    Qa
    q_nom
    v_nom
    q_idx
    v_idx
  end
  
  properties(Access = private)
    dt
    nq
    nv
    nT
  end
  
  methods
    function obj = WeightedTrackingError(t,Q,q_nom,Qv,v_nom,Qa,q_idx,v_idx)
      t = unique(t(:))';
      obj = obj@NonlinearConstraint(-inf,inf,(size(q_nom,1)+size(v_nom,1))*length(t));
      if(~isnumeric(t))
        error('Drake:CubicPostureError:t should be numeric');
      end
      obj.nT = length(t);
      obj.dt = diff(t);
      if(obj.nT<=1)
        error('Drake:CubicPostureError: t should have at least two distince elemeobj.nTs');
      end
      obj.nq = size(q_nom,1);
      valuecheck(obj.nT,size(q_nom,2));
      if(~isnumeric(q_nom))
        error('Drake:CubicPostureError:q_nom should be numeric');
      end
      obj.q_nom = q_nom;
      obj.nv = size(v_nom,1);
      valuecheck(obj.nT,size(v_nom,2));
      if(~isnumeric(v_nom))
        error('Drake:CubicPostureError:v_nom should be numeric');
      end
      obj.v_nom = v_nom;
      if(~isnumeric(Q) || size(Q,1) ~= obj.nq || size(Q,2) ~= obj.nq || any(eig(Q))<0)
        error('Drake:CubicPostureError:Q should be a nq x nq PSD matrix');
      end
      obj.Q = Q;
      if(~isnumeric(Qv) || size(Qv,1) ~= obj.nq || size(Qv,2) ~= obj.nq || any(eig(Qv))<0)
        error('Drake:CubicPostureError:Qv should be a nq x nq PSD matrix');
      end
      obj.Qv = Qv;
      if(~isnumeric(Qa) || size(Qa,1) ~= obj.nq || size(Qa,2) ~= obj.nq || any(eig(Qa))<0)
        error('Drake:CubicPostureError:Qa should be a nq x nq PSD matrix');
      end
      obj.Qa = Qa;
      obj.q_idx = 1:obj.nT*obj.nq;
      obj.v_idx = obj.q_idx(end) + (1:obj.nT*obj.nv);
    end
    
    function [c,dc] = eval(obj,x)
      % @param x    -- A double array of size nq*(nT+2). x =
      % [q(:,1);q(:,2);...;q(:,nT);qdot(:,1);qdot(:,nT)]
      q = x(obj.q_idx);
      v = x(obj.v_idx);
      q = reshape(q,obj.nq,obj.nT);
      v = reshape(v,obj.nv,obj.nT);
      a = diff(v,1,2);
      qerr = q-obj.q_nom;
      verr = v-obj.v_nom;
      Qqerr = obj.Q*qerr;
      Qvv = obj.Qv*v;
      Qvv(:,[1,end]) = 0.5*Qvv(:,[1,end]);
      Qaa = obj.Qa*a;
      c = sum(sum(qerr.*Qqerr))+sum(sum(verr.*Qvv)+sum(sum(a.*Qaa)));
      c = c/obj.nT;
      dc = zeros(1,(obj.nq+obj.nv)*obj.nT);
      dc(obj.q_idx) = reshape(2*Qqerr,1,[]);
      dc(obj.v_idx) = reshape(2*Qvv,1,[]);
      dc(obj.v_idx(:,1:(end-obj.nv))) = dc(obj.v_idx(:,1:(end-obj.nv))) - reshape(2*Qaa,1,[]);
      dc(obj.v_idx(:,obj.nv+1:end)) = dc(obj.v_idx(:,obj.nv+1:end)) + reshape(2*Qaa,1,[]);
      dc = dc/obj.nT;
    end
  end
end
