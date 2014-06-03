classdef FootStepRegionConstraint < SingleTimeKinematicConstraint
  % Based on the constraint on the robot x,y and yaw, transcribe it to full body kinematic
  % constraint.
  % @param A,b,C,d are the constraints on the x,y and yaw of the foot. A is a m x 3
  % matrix, b is m x 1 vector. C is a 3 x 3 positive definite matrix, d is a 3 x 1 vector. 
  % A[x;y;yaw] <= b is the half-space constraint on x,y and yaw.
  % ([x;y;yaw]-d)'(C^-T*C^-1)([x;y;yaw]-d) <= 1 is the ellipsoidal inner approximation of
  % the half space constraint
  % @param body    -- A int scalar. The body index of the foot
  % @param body_pt -- A 3 x 1 vector. The point on the body whose x,y position are
  % constraint
  % @param z_offset -- A positive scalar. When in contact, the distance from body_pt to
  % the contact plane is z_offset
  % @param ground_normal  -- A 3 x 1 unit vector. The normal vector of the ground plane
  % pointing out of the ground plane.
  % @param ground_pt      -- A 3 x 1 vector. A point on the ground plane
  % @param halfspace_constraint   -- A boolean flag. True if the half space constraint
  % A[x;y;yaw] <= b is used. False if the ellipoidal constraint
  % ([x;y;yaw]-d)^T(C^-TC^-1)([x;y;yaw]-d)<=1 is used. Default is true;
  properties(SetAccess = protected)
    A
    b
    C
    d
    body
    body_name
    body_pt
    z_offset
    ground_normal
    ground_pt
    halfspace_constraint
  end
  
  properties(Access = protected)
    inv_C;
    num_halfspace;
  end
  
  methods
    function obj = FootStepRegionConstraint(robot,A,b,C,d,body,body_pt,z_offset,ground_normal,ground_pt,tspan)
      % @param A,b,C,d are the constraints on the x,y and yaw of the foot. A is a m x 3
      % matrix, b is m x 1 vector. C is a 3 x 3 positive definite matrix, d is a 3 x 1 vector. 
      % A[x;y;yaw] <= b is the half-space constraint on x,y and yaw.
      % ([x;y;yaw]-d)'(C^-T*C^-1)([x;y;yaw]-d) <= 1 is the ellipsoidal inner approximation of
      % the half space constraint
      % @param body    -- A int scalar. The body index of the foot
      % @param body_pt -- A 3 x 1 vector. The point on the body whose x,y position are
      % constraint
      % @param z_offset -- A positive scalar. When in contact, the distance from body_pt to
      % the contact plane is z_offset
      % @param ground_normal  -- A 3 x 1 unit vector. The normal vector of the ground
      % plane pointing out of the ground plane
      % @param ground_pt      -- A 3 x 1 vector. A point on the ground plane
      % @param tspan     -- A 1 x 2 vector. The time span of the constraint. Default is
      % [-inf,inf]
      if(nargin<11)
        tspan = [-inf,inf];
      end
      obj = obj@SingleTimeKinematicConstraint(robot,tspan);
      if(~isnumeric(A))
        error('Drake:FootStepRegionConstraint:A should be numeric');
      end
      A_size = size(A);
      obj.num_halfspace = A_size(1);
      if(A_size(2) ~= 3 || length(A_size) ~= 2)
        error('Drake:FootStepRegionConstraint: A should be a m x 3 matrix');
      end
      obj.A = A;
      if(~isnumeric(b))
        error('Drake:FootStepRegionConstraint: b should be numeric');
      end
      sizecheck(b,[obj.num_halfspace,1]);
      obj.b = b;
      if(~isnumeric(C) || any(eig(C))<eps)
        error('Drake:FootStepRegionConstraint: C should be a positive matrix');
      end
      sizecheck(C,[3,3]);
      obj.C = C;
      obj.inv_C = inv(obj.C);
      if(~isnumeric(d))
        error('Drake:FootStepRegionConstraint: d should be numeric');
      end
      sizecheck(d,[3,1]);
      obj.d = d;
      if(~isnumeric(body))
        error('Drake:FootStepRegionConstraint:body should be numeric');
      end
      sizecheck(body,[1,1]);
      obj.body = floor(body);
      obj.body_name = obj.robot.getBody(obj.body).linkname;
      if(~isnumeric(body_pt))
        error('Drake:FootStepRegionConstraint:body_pt should be numeric');
      end
      sizecheck(body_pt,[3,1]);
      obj.body_pt = body_pt;
      if(~isnumeric(z_offset))
        error('Drake:FootStepRegionConstraint:z_offset should be numeric');
      end
      sizecheck(z_offset,[1,1]);
      if(z_offset<0)
        error('Drake:FootStepRegionConstraint:z_offset should be non-negative');
      end
      obj.z_offset = z_offset;
      if(~isnumeric(ground_normal))
        error('Drake:FootStepRegionConstraint:ground_normal should be numeric');
      end
      sizecheck(ground_normal,[3,1]);
      norm_ground_normal = norm(ground_normal);
      if(norm_ground_normal<eps)
        error('Drake:FootStepRegionConstraint:ground_normal should be non-zero');
      end
      obj.ground_normal = ground_normal/norm_ground_normal;
       if(~isnumeric(ground_pt))
        error('Drake:FootStepRegionConstraint:ground_pt should be numeric');
      end
      sizecheck(ground_pt,[3,1]);
      obj.ground_pt = ground_pt;
      obj = obj.setUseHalfSpaceConstraint(true);
      obj.type = RigidBodyConstraint.FootStepRegionConstraintType;
    end
    
    function obj = setUseHalfSpaceConstraint(obj,flag)
      % @param flag  -- A boolean variable. True if the [x;y;yaw] is constraint by
      % halfspace A[x;y;yaw] <= b. False if the [x;y;yaw] is constrained by ellipsoidal
      % constraint ([x;y;yaw]-d)^T(C^-TC^-1)([x;y;yaw]-d) <= 1
      if(~islogical(flag))
        error('Drake:FootStepRegionConstraint:setUseHalfSpaceConstraint:flag should be boolean');
      end
      sizecheck(flag,[1,1]);
      obj.halfspace_constraint = flag;
      if(obj.halfspace_constraint)
        obj.num_constraint = obj.num_halfspace+2;
      else
        obj.num_constraint = 3;
      end
    end
    
    function [lb,ub] = bounds(obj,t)
      if(obj.isTimeValid(t))
        if(obj.halfspace_constraint)
          lb = [1;obj.z_offset;-inf(obj.num_halfspace,1)];
          ub = [1;obj.z_offset;zeros(obj.num_halfspace,1)];
        else
          lb = [1;obj.z_offset;0];
          ub = [1;obj.z_offset;1];
        end
      else
        lb = [];
        ub = [];
      end
    end
    
    function name_str = name(obj,t)
      if(obj.isTimeValid(t))
        if(isempty(t))
          t_str = '';
        else
          t_str = sprintf(' at time %5.2f',t);
        end
        normal_align_name = sprintf('FootStepRegionConstraint align normal vector%s',t_str);
        dist_name = sprintf('FootStepRegionConstraint contact distance%s',t_str);
        if(obj.halfspace_constraint)
          half_space_name = {sprintf('FootStepRegionConstraint halfspace%s',t_str)};
          xyyaw_name = half_space_name(ones(obj.num_halfspace,1),:);
        else
          xyyaw_name = {sprintf('FootStepRegionConstraint ellipsoidal%s',t_str)};
        end
        name_str = [{normal_align_name};{dist_name};xyyaw_name];
      else
        name_str = {};
      end
    end
    
    function joint_idx = kinematicsPathJoints(obj)
      [~,joint_path] = obj.robot.findKinematicPath(1,obj.body);
      joint_idx = vertcat(obj.robot.body(joint_path).dofnum)';
    end
    
    function T = bodyT(obj,x,y,yaw)
      % given x,y position of the body_pt in the world frame and the yaw angle, return the
      % homogeneous transform of the body
      [rotmat,A,b] = obj.bodyTransform(yaw);
      T = [rotmat A*[x;y]+b;0 0 0 1];
    end
    
    function [rotmat,A,b,drotmat,dA,db] = bodyTransform(obj,yaw)
      % Given the x,y position of obj.body_pt and yaw angle of the foot. The world
      % coordinate of a point P is P_w = A*[x;y]+b+rotmat*P_b, where P_b is the coordinate
      % of P in the body frame.
      % @retval rotmat   A 3 x 3 rotation matrix
      % @retval A,b    A is a 3 x 2 matrix and b is a 3 x 1 vector. A*[x;y]+b is the world
      % coordinate of 0_b, where O_b is the origin in the body frame.
      % @retval drotmat  A 3 x 3 matrix. gradient of rotmat w.r.t yaw
      % @retval dA      A 3 x 2 matrix. gradient of A w.r.t yaw
      % @retval db      A 3 x 1 matrix. gradient of b w.r.t yaw
      axis = cross([0;0;1],obj.ground_normal);
      axis_norm = norm(axis);
      if(axis_norm<1e-3)
        rotmat1 = eye(3);
      else
        theta = asin(axis_norm);
        axis = axis/axis_norm;
        rotmat1 = axis2rotmat([axis;theta]);
      end
      rotmat2 = axis2rotmat([0;0;1;yaw]);
      drotmat2 = [-sin(yaw) -cos(yaw) 0;cos(yaw) -sin(yaw) 0;0 0 0];
      rotmat = rotmat1*rotmat2;
      drotmat = rotmat1*drotmat2;
      A = [1 0;0 1;-obj.ground_normal(1:2)'/obj.ground_normal(3)];
      dA = zeros(3,2);
      b = [0;0;(obj.z_offset+obj.ground_normal'*obj.ground_pt)/obj.ground_normal(3)]-rotmat*obj.body_pt;
      db = -drotmat*obj.body_pt;
    end
    
    function [position_cnstr,quat_cnstr] = generateFixedPosConstraint(obj,robot,xy,yaw)
      % generate a WorldPositionConstraint and a WorldQuatConstraint with given xy
      % position and yaw angles
      % @param xy  A 2 x 1 vector. The xy position of the foot
      % @param yaw  A double. The yaw angle of the foot
      [rotmat,A_xy,b_xy] = obj.bodyTransform(yaw);
      position = A_xy*xy+b_xy;
      position_cnstr = WorldPositionConstraint(robot,obj.body,[0;0;0],position,position,obj.tspan);
      quat_cnstr = WorldQuatConstraint(robot,obj.body,rotmat2quat(rotmat),0,obj.tspan);
    end
  end
  
  methods(Access = protected)
    function [c,dc] = evalValidTime(obj,kinsol)
      [pos,dpos] = obj.robot.forwardKin(kinsol,obj.body,[obj.body_pt obj.body_pt+[0;0;1]],1);
      body_normal_world = pos(1:3,2)-pos(1:3,1);
      dbody_normal_world = dpos(7:9,:)-dpos(1:3,:);
      normal_align = body_normal_world'*obj.ground_normal;
      dnormal_align = obj.ground_normal'*dbody_normal_world;
      dist = (pos(1:3,1)-obj.ground_pt)'*obj.ground_normal;
      ddist = obj.ground_normal'*dpos(1:3,:);
      xyyaw = [pos(1,1);pos(2,1);pos(6,1)];
      dxyyaw = [dpos(1,:);dpos(2,:),dpos(6,:)];
      if(obj.halfspace_constraint)
        xyyaw_constr = obj.A*xyyaw-obj.b;
        dxyyaw_constr = obj.A*dxyyaw;
      else
        ellip_vector = obj.inv_C*(xyyaw-obj.d);
        xyyaw_constr = sum(ellip_vector.^2);
        dxyyaw_constr = 2*ellip_vector'*obj.inv_C*dxyyaw;
      end
      c = [normal_align;dist;xyyaw_constr];
      dc = [dnormal_align;ddist;dxyyaw_constr];
    end
  end
end