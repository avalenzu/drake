classdef FootStepRegionContactConstraint
  % This constraint wraps the iris result on the feasible foot regions, together with the
  % linearized friction cone information, the body contact
  % points and contact force paramters
  properties(SetAccess = protected)
    robot_mass % The mass of the robot
    foot_step_region_cnstr % A FootStepRegionConstraint
    mu % The friction coefficient
    num_edges % The number of edges sampled in each cone
    edges % A 3 x num_edges matrix. edges(:,i) is the coordinate of the i'th edge in the linearized friction cone, in the body frame, where the normal force is along the z direction.
    num_contact_pts % The number of contact points on the body
    body_contact_pts % A 3 x num_contact_pts matrix. body_contact_pts(:,i) is the coordinate of i'th contact point in the body frame.
    num_force_weight % total number of force weights, num_edges*num_contact_pts
  end  
  
  methods
    function obj = FootStepRegionContactConstraint(foot_step_region_cnstr,mu,num_edges,body_contact_pts)
      if(~isa(foot_step_region_cnstr,'FootStepRegionConstraint') )
        error('The input should be a FootStepRegionConstraint');
      end
      obj.foot_step_region_cnstr = foot_step_region_cnstr;
      sizecheck(mu,[1,1]);
      if(mu<0)
        error('friction coefficient must be non-negative');
      end
      obj.mu = mu;
      sizecheck(num_edges,[1,1]);
      if(num_edges<=0)
        error('number of edges must be positive in the linearized friction cone');
      end
      obj.num_edges = num_edges;
      theta = linspace(0,2*pi,num_edges+1);
      theta = theta(1:end-1);
      obj.edges = [cos(theta);sin(theta);ones(1,num_edges)];
      body_pts_size = size(body_contact_pts);
      if(length(body_pts_size) ~= 2 || body_pts_size(1) ~= 3)
        error('body_pts should be 3 x m matrix');
      end
      obj.num_contact_pts = body_pts_size(2);
      obj.body_contact_pts = body_contact_pts;
      obj.num_force_weight = obj.num_edges*obj.num_contact_pts;
      obj.robot_mass = obj.foot_step_region_cnstr.robot.getMass();
    end
    
    function [A,dA] = force(obj,yaw)
      % The force in the world frame is A*w where w is the force weight;
      % @param yaw   The yaw angle
      % @retval A   A matrix. A*w is the force
      % @retval dA  A matrix. gradient of A w.r.t yaw
      if(nargin <2)
        R = obj.foot_step_region_cnstr.bodyTransform(yaw);
        A = obj.robot_mass*R*obj.edges;
      else
        [R,~,~,dR] = obj.foot_step_region_cnstr.bodyTransform(yaw);
        A = obj.robot_mass*R*obj.edges;
        dA = obj.robot_mass*dR*obj.edges;
      end
    end
    
    function pos = contactPosition(obj,x,y,yaw)
      % Given the xy position and yaw angle of the body, return the position of the
      % contact points in the world frame.
      T = obj.foot_step_region_cnstr.bodyT(x,y,yaw);
      pos = T*[obj.body_contact_pts;ones(1,obj.num_contact_pts)];
      pos = pos(1:3,:);
    end
    
  end
end