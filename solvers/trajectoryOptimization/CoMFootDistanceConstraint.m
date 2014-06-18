classdef CoMFootDistanceConstraint < NonlinearConstraint
  % bound the distance between the CoM and the foot given the xy position of the foot
  properties(SetAccess = protected)
    fsr_cnstr % A FootStepRegionConstraint object. The distance is measured between Center of Mass and the body_pt of fsr_cnstr
    dist_lb % A scalar. The lower bound of the distance
    dist_ub % A scalar. The upper bound of the distance
  end
  
  methods
    function obj = CoMFootDistanceConstraint(fsr_cnstr,dist_lb,dist_ub)
      % @param fsr_cnstr  A FootStepRegionConstraint object
      % @param dist_lb  A scalar. The lower bound of the distance
      % @param dist_ub  A scalar. The upper bound of the distance
      obj = obj@NonlinearConstraint(dist_lb^2,dist_ub^2,6);
      sizecheck(dist_lb,[1,1]);
      sizecheck(dist_ub,[1,1]);
      if(~isa(fsr_cnstr,'FootStepRegionConstraint'))
        error('Drake:CoMFootDistanceConstraint:input should be a FootStepRegionConstraint');
      end
      obj.fsrc_cnstr = fsr_cnstr;
      obj.dist_lb = dist_lb;
      obj.dist_ub = dist_ub;
    end
    
    function [c,dc] = eval(obj,x)
      % @param x = [com_x;com_y;com_z;foot_x;foot_y;foot_yaw]
      com = x(1:3);
      foot_xy = x(4:5);
      yaw = x(6);
      [rotmat,A,b,drotmat,dA,db] = obj.fsr_cnstr.bodyTransform(yaw);
      foot_pos = A*foot_xy+b+rotmat*obj.fsr_cnstr.body_pt;
      dfoot_pos = [A dA*foot_xy+db+drotmat*obj.fsr_cnstr.body_pt];
      com_foot_pos = com-foot_pos;
      c = sum(com_foot_pos.^2);
      dc = 2*com_foot_pos'*[eye -dfoot_pos];
    end
    
  end
end