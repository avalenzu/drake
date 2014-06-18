classdef LinearDynamics < LinearConstraint
  methods
    function obj = LinearDynamics(t,nq,nv,q_idx,qd_idx)
      % obj = LinearPostureError(q_idx,qd_idx) constructs a LinearPostureError
      % object
      %
      % @param q_idx    -- vector of indices into the decision varible vector
      %                    corresponding to: 
      %                    ( q_1[1], ... , q_nq[1], ... , 
      %                      q_1[nT], ... , q_nq[nT]);
      % @param qd_idx    -- vector of indices into the decision varible vector
      %                    corresponding to: 
      %                    ( qd_1[1], ... , qd_nq[1], ... , 
      %                      qd_1[nT], ... , qd_nq[nT]);
      nT = numel(t);
      dt = repmat(diff(t),nq,1);
      A = zeros(nq*(nT-1),2*nq*nT);

      q_idx         = reshape(q_idx,nq,nT);
      qd_idx        = reshape(qd_idx,nv,nT-1);
      q_minus_idx   = reshape(q_idx(:,1:(nT-1)),1,nq*(nT-1));
      q_plus_idx    = reshape(q_idx(:,2:nT),1,nq*(nT-1));
      q_minus_idx   = sub2ind(size(A),1:nq*(nT-1),q_minus_idx);
      q_plus_idx    = sub2ind(size(A),1:nq*(nT-1),q_plus_idx);
      qd_minus_idx  = reshape(qd_idx(:,1:(nT-1)),1,nq*(nT-1));
      %qd_plus_idx   = reshape(qd_idx(:,1:(nT-1)),1,nq*(nT-1));
      qd_minus_idx  = sub2ind(size(A),1:nq*(nT-1),qd_minus_idx);
      %qd_plus_idx   = sub2ind(size(A),1:nq*(nT-1),qd_plus_idx);

      A(q_minus_idx) = -1;
      A(q_plus_idx) = 1;
      A(qd_minus_idx) = -dt(:);
      %A(qd_plus_idx) = 0.5*dt(:);
      obj = obj@LinearConstraint(zeros(nq*(nT-1),1),zeros(nq*(nT-1),1),A);
    end
  end
end
