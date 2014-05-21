classdef CoMFootStepPolygon < NonlinearConstraint
  % Given the FootStepRegionContactConstraint and the vertices of the CoM region relative
  % to the foot, give a halfspace constraint in the world frame
  properties(SetAccess = protected)
    fsr_cnstr % A FootStepRegionConstraint
    vertices % A 3 x num_vertices matrix. vertices(:,i) is the i'th vertex in the foot body frame
    num_halfspace % Number of half space in the H representation of the polygon
  end
  
  properties(Access = protected)
    A_body % A_body*[x;y;z]<=b_body encodes the constraint that point with coordinate [x;y;z] in the body frame is within the polygon
    b_body
  end
  
  methods
    function obj = CoMFootStepPolygon(fsr_cnstr,vertices)
      % @param fsr_cnstr  A FootStepRegionConstraint
      % @param vertices   A 3 x n matrix
      
      if(~isa(fsr_cnstr,'FootStepRegionConstraint'))
        error('Drake:CoMFootStepPolygon: should be a FootStepRegionConstraint');
      end
      sizecheck(vertices,[3,nan]);
      H = cddmex('hull',struct('V',vertices'));
      Hred = cddmex('reduce_h',H);
      H_dim = size(Hred.B,1);
      obj = obj@NonlinearConstraint(-inf(H_dim,1),zeros(H_dim,1),6);
      obj.num_halfspace = H_dim;
      obj.fsr_cnstr = fsr_cnstr;
      obj.vertices = vertices;
      obj.A_body = Hred.A;
      obj.b_body = Hred.B;
    end
    
    function [c,dc] = eval(obj,x)
      % @param x   A 6 x 1 vector. x = [com_x;com_y;com_z;foot_x;foot_y;yaw_foot]
      com = x(1:3);
      foot_xy = x(4:5);
      yaw = x(6);
      [rotmat,A,b,drotmat,dA,db] = obj.fsr_cnstr.bodyTransform(yaw);
      com_foot = rotmat'*(com-A*foot_xy-b);
      dcom_foot = [rotmat'*[eye(3) -A] drotmat'*(com-A*foot_xy-b)+rotmat'*(-dA*foot_xy-db)];
      c = obj.A_body*com_foot-obj.b_body;
      dc = obj.A_body*dcom_foot;
    end
    
    function [A,b] = halfspace(obj,yaw)
      % The halfspace A*[com_x;com_y;com_z;foot_x;foot_y]<=b encodes that the CoM location
      % is in the desired polygon, with a fixed yaw angle of the foot
      % @param yaw   A scalar. The yaw angle of the foot
      if(~isnumeric(yaw) || numel(yaw) ~= 1)
        error('Drake:CoMFootStepPolygon: yaw should be a double scalar');
      end
      [rotmat,A,b] = obj.fsr_cnstr.bodyTransform(yaw);
      A = [obj.A_body*rotmat' -obj.A_body*rotmat'*A];
      b = obj.b_body+obj.A_body*rotmat'*b;
    end
  end
end