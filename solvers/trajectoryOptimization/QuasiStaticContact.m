classdef QuasiStaticContact 
  % Given several FootStepRegionContactConstraint, the projection of CoM is within the
  % convex hull
  properties(SetAccess = protected)
    fsrc_cnstr % A cell of FootStepRegionContactConstraint
    num_fsrc_cnstr  % The number of FootStepRegionContactConstraint
    scale_factor % A nonnegative scaler. Scale the foot contact region to get the allowable region of CoM
    num_contact_pts % The number of contact points 
  end
  
  methods
    function obj = QuasiStaticContact(fsrc_cnstr,scale_factor)
      % @param fsrc_cnstr  A cell of FootStepRegionContactConstraint
      % @param scale_factor   A nonnegative scalar. Scale the convex hull of foot contact
      % region to get the allowable region of CoM
      obj.fsrc_cnstr = fsrc_cnstr;
      if(~isnumeric(scale_factor) || numel(scale_factor) ~= 1 || scale_factor<0)
        error('Drake:QuasiStaticContact:scale factor should be a non-negative scaler');
      end
      typecheck(fsrc_cnstr,'FootStepRegionContactConstraint');
      obj.num_fsrc_cnstr = length(fsrc_cnstr);
      obj.num_contact_pts = 0;
      for i = 1:obj.num_fsrc_cnstr
        obj.num_contact_pts = obj.num_contact_pts+fsrc_cnstr{i}.num_contact_pts;
      end
    end
    
    function [Ain,bin] = halfspace(obj,foot_xy,yaw)
      % generate the halfspace constraint Ain*x<=bin for x =
      % [com_x;com_y]. These
      % linear constraints enforce the CoM lie within the allowable polygon
      % @param foot_xy A 2 x obj.num_fsrc_cnstr matrix. foot_xy(:,i) is the xy location of
      % obj.fsrc_cnstr{i}
      % @param yaw     A 1 x obj.num_fsrc_cnstr vector. yaw(i) is the yaw angle for
      % obj.fsrc_cnstr{i}
      foot_contact_pos = zeros(3,obj.num_contact_pts);
      contact_pts_count = 0;
      for i = 1:obj.num_fsrc_cnstr
        num_contact_pts_i = obj.fsrc_cnstr{i}.num_contact_pts;
        [rotmat_fsrc_i,A_fsrc_i,b_fsrc_i] = obj.fsrc_cnstr{i}.foot_step_region_cnstr.bodyTransform(yaw(i));
        foot_contact_pos(:,contact_pts_count+(1:num_contact_pts_i)) = ...
          bsxfun(@times,ones(1,num_contact_pts_i),A_fsrc_i*foot_xy(:,i)+b_fsrc_i)+rotmat_fsrc_i*obj.fsrc_cnstr{i}.body_contact_pts;
        contact_pts_count = contact_pts_count+num_contact_pts_i;
      end
      center_pt = mean(foot_contact_pos,2);
      support_vert = bsxfun(@times,(1-obj.scale_factor)*ones(1,obj.num_contact_pts),center_pt)+obj.scale_factor*foot_contact_pos;
      H = cddmex('hull',struct('V',support_vert(1:2,:)'));
      Hred = cddmex('reduce_h',H);
      Ain = Hred.A;
      bin = Hred.B;
    end
  end
end