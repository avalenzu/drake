classdef StepCost 
  % Given a nominal step location of footB in the footA frame, penalize the scaled euclidean
  % distance to this nominal step
  properties(SetAccess = protected)
    fsr_cnstrA % FootStepRegionConstraint of footA
    fsr_cnstrB % FootStepRegionConstraint of footB
    footB_pos_des % A 3 x 1 vector. The desired xy position of footB in footA frame
    Q % A 3 x 3 PSD matrix. The penalized cost is d'Qd where d is the displacement
  end
  
  methods
    function obj = StepCost(fsr_cnstrA,fsr_cnstrB,footB_pos_des,Q)
      % @param fsr_cnstrA  A FootStepRegion of footA
      % @param fsr_cnstrB  A FootStepRegion of footB
      % @param footB_pos_des  A 3 x 1 vector. The desired xy position of footB in footA
      % frame
      % @param Q  A 3 x 3 PSD matrix. The penalized cost is d'Qd where d is the
      % displacement
      if(~isa(fsr_cnstrA,'FootStepRegionConstraint') || ~isa(fsr_cnstrB,'FootStepRegionConstraint'))
        error('Drake:StepCost:fsr_cnstrA and fsr_cnstrB should be FootStepRegionConstraint');
      end
      obj.fsr_cnstrA = fsr_cnstrA;
      obj.fsr_cnstrB = fsr_cnstrB;
      if(~isnumeric(footB_pos_des))
        error('Drake:StepCost:footB_pos_des should be numeric');
      end
      sizecheck(footB_pos_dex,[3,1]);
      obj.footB_pos_des = footB_pos_des;
      if(~isnumeric(Q))
        error('Drake:StepCost:Q should be numeric');
      end
      sizecheck(Q,[3,3]);
      if(any(eig(Q)<0))
        error('Drake:StepCost:Q should be PSD');
      end
      obj.Q = (Q+Q')/2;
    end
    
    function [H,f] = FixedYawCost(obj,footA_yaw,footB_yaw)
      % generate a quadratic cost on the foot displacement when fixing the yaw angle of
      % footA
      % @param footA_yaw   A double scalar. The yaw angle of footA
      % @param footB_yaw   A double scalar. The yaw angle of footB
      % @retval H   A 4 x 4 matrix. The Hessian of the cost w.r.t [xA;yA;xB;yB]
      % @retval f   A 4 x 1 matrix. The linear componenet of the cost
      if(~isnumeric(footA_yaw) || numel(footA_yaw) ~= 1 || ~isnumeric(footB_yaw) || numel(footB_yaw) ~= 1)
        error('Drake:StepCost:footA_yaw and footB_yawshould be a double scaler');
      end
      [rotmatA,A_fsrA,b_fsrA] = obj.fsr_cnstrA.bodyTransform(footA_yaw);
      [rotmatB,A_fsrB,b_fsrB] = obj.fsr_cnstrB.bodyTransform(footB_yaw);
      % cost = (Ax+b)'Q(Ax+b)
      A = [-rotmatA'*A_fsrA rotmatB'*A_fsrB];
      b = rotmatA'*b_fsrB-rotmatA'*b_fsrA;
      H = A'*obj.Q*A;
      f = 2*b'*obj.Q*A;
    end
  end
end