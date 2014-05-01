classdef WrenchMatchConstraint < NonlinearConstraint
  % This constraint is used in CoMForcePlanning, that the wrench computed from external
  % forces should match with the wrench computed from CoM at a single time.
  % The decision variables are [com;comp;compp;sdot1;sdot2;Hdot;foot_pos;yaw;force_weights]
  properties(SetAccess = protected)
    robot_mass % mass of the robot
    g % gravitational acceleration
    fsrc_cnstr  % A cell of FootStepRegionContactConstraint
    delta_s % A positive scalar. The difference of scaling function s between two consecutive knot points
  end
  
  properties(Access = protected)
    num_fsrc_cnstr % The number of FootStepRegionContactConstraint
    com_idx; % 3 x 1 vector. The indices of CoM in x
    comp_idx; % 3 x 1 vector. The indices of first order derivative of CoM w.r.t s in x
    compp_idx; % 3 x 1 vector. The indices of second order derivative of CoM w.r.t s in x
    sdot_idx; % 2 x 1 vector. The indices of sdot1 and sdot2 in x
    Hdot_idx; % 3 x 1 vector. The indices of rate of angular momentum in x
    foot_pos_idx; % A 2 x num_fsrc_cnstr matrix. foot_pos_idx(:,i) is the indices of the xy position of the foot for the i'th FootStepRegionContactConstraint
    yaw_idx; % A 1 x num_fsrc_cnstr matrix. foot_pos_idx(:,i) is the indices of the yaw of the foot for the i'th FootStepRegionContactConstraint
    F_idx % A cell array. F_idx{i} is the indices of the foot weight of the i'th FootStepRegionContactConstraint
    num_force_weight; % The total number of force weight.
  end
  
  methods
    function obj = WrenchMatchConstraint(robot_mass,g,fsrc_cnstr,delta_s)
      % @param robot_mass  The mass of the robot
      % @param g  The gravitational acceleration
      % @param fsrc_cnstr  A cell of FootStepRegionContactConstraint
      count_force_weight = 0;
      for i = 1:length(fsrc_cnstr)
        count_force_weight = count_force_weight+fsrc_cnstr{i}.num_force_weight;
      end
      obj = obj@NonlinearConstraint(zeros(6,1),zeros(6,1),3+3+3+2+3+length(fsrc_cnstr)*3+count_force_weight);
      obj.robot_mass = robot_mass;
      obj.g = g;
      obj.fsrc_cnstr = fsrc_cnstr;
      obj.num_fsrc_cnstr = length(obj.fsrc_cnstr);
      obj.delta_s = delta_s;
      obj.num_force_weight = count_force_weight;
      num_x = 0;
      obj.com_idx = num_x+(1:3)';
      num_x = num_x+3;
      obj.comp_idx = num_x+(1:3)';
      num_x = num_x+3;
      obj.compp_idx = num_x+(1:3)';
      num_x = num_x+3;
      obj.sdot_idx = num_x+(1:2)';
      num_x = num_x+2;
      obj.Hdot_idx = num_x+(1:3)';
      num_x = num_x+3;
      obj.foot_pos_idx = num_x+reshape(1:2*obj.num_fsrc_cnstr,2,obj.num_fsrc_cnstr);
      num_x = num_x+2*obj.num_fsrc_cnstr;
      obj.yaw_idx = num_x+(1:obj.num_fsrc_cnstr);
      num_x = num_x+obj.num_fsrc_cnstr;
      obj.F_idx = cell(1,obj.num_fsrc_cnstr);
      F_idx_all = zeros(obj.num_force_weight,1);
      count_force_weight = 0;
      for i = 1:obj.num_fsrc_cnstr
        obj.F_idx{i} = num_x+reshape(1:fsrc_cnstr{i}.num_force_weight,fsrc_cnstr{i}.num_edges,fsrc_cnstr{i}.num_contact_pts);
        num_x = num_x+fsrc_cnstr{i}.num_force_weight;
        F_idx_all(count_force_weight+(1:fsrc_cnstr{i}.num_force_weight)) = obj.F_idx{i}(:);
        count_force_weight = count_force_weight+fsrc_cnstr{i}.num_force_weight;
      end
      iCfun = reshape(bsxfun(@times,(1:3)',ones(1,3+3+2+obj.num_fsrc_cnstr+obj.num_force_weight)),[],1);
      jCvar = reshape(bsxfun(@times,ones(3,1),[obj.comp_idx;obj.compp_idx;obj.sdot_idx;obj.yaw_idx(:);F_idx_all]'),[],1);
      iCfun = [iCfun;reshape(bsxfun(@times,(4:6)',ones(1,2*obj.num_fsrc_cnstr+obj.num_fsrc_cnstr+3+3+obj.num_force_weight)),[],1)];
      jCvar = [jCvar;reshape(bsxfun(@times,ones(3,1),[obj.foot_pos_idx(:);obj.yaw_idx(:);obj.com_idx;obj.Hdot_idx;F_idx_all]'),[],1)];
      obj = obj.setSparseStructure(iCfun,jCvar);
    end
    
    function [c,dc] = eval(obj,x)
      c = zeros(6,1);
      dc = zeros(6,obj.xdim);
      com = x(obj.com_idx);
      comp = x(obj.comp_idx);
      compp = x(obj.compp_idx);
      sdot = x(obj.sdot_idx);
      Hdot = x(obj.Hdot_idx);
      foot_pos = x(obj.foot_pos_idx);
      yaw = x(obj.yaw_idx);
      F_weight = cell(1,obj.num_fsrc_cnstr);
      force_total = zeros(3,1);
      c(4:6) = -Hdot;
      dc(4:6,obj.Hdot_idx) = -eye(3);
      for i = 1:obj.num_fsrc_cnstr
        F_weight{i} = reshape(x(obj.F_idx{i}(:)),obj.fsrc_cnstr{i}.num_edges,obj.fsrc_cnstr{i}.num_contact_pts);
        [A_force_i,dA_force_i] = obj.fsrc_cnstr{i}.force(yaw(i));
        force_i = A_force_i*F_weight{i};
        dforce_idyaw = dA_force_i*F_weight{i};
        force_i_total = sum(force_i,2);
        force_total = force_total+force_i_total;
        dc(1:3,obj.F_idx{i}(:)) = repmat(A_force_i,1,obj.fsrc_cnstr{i}.num_contact_pts);
        dc(1:3,obj.yaw_idx(i)) = sum(dA_force_i*F_weight{i},2);
        [rotmat,A_xy,b_xy,drotmat,dA_xy,db_xy] = obj.fsrc_cnstr{i}.foot_step_region_cnstr.bodyTransform(yaw(i));
        contact_pos_CoM = rotmat*obj.fsrc_cnstr{i}.body_contact_pts+bsxfun(@times,A_xy*foot_pos(:,i)+b_xy-com,ones(1,obj.fsrc_cnstr{i}.num_contact_pts));
        dcontact_pos_CoMdyaw = drotmat*obj.fsrc_cnstr{i}.body_contact_pts+bsxfun(@times,dA_xy*foot_pos(:,i)+db_xy,ones(1,obj.fsrc_cnstr{i}.num_contact_pts));
        c(4:6) = c(4:6)+sum(cross(contact_pos_CoM,force_i),2);
        dc(4:6,obj.F_idx{i}(:)) = cross(reshape(repmat(contact_pos_CoM,obj.fsrc_cnstr{i}.num_edges,1),3,[]),...
          repmat(A_force_i,1,obj.fsrc_cnstr{i}.num_contact_pts));
        dc(4:6,obj.yaw_idx(i)) = sum(cross(contact_pos_CoM,dforce_idyaw),2)+sum(cross(dcontact_pos_CoMdyaw,force_i),2);
        dc(4:6,obj.foot_pos_idx(:,i)) = -[sum(cross(force_i,bsxfun(@times,A_xy(:,1),ones(1,obj.fsrc_cnstr{i}.num_contact_pts))),2) sum(cross(force_i,bsxfun(@times,A_xy(:,2),ones(1,obj.fsrc_cnstr{i}.num_contact_pts))),2)];
      end
      dc(4:6,obj.com_idx) = [0 -force_total(3) force_total(2);force_total(3) 0 -force_total(1);-force_total(2) force_total(1) 0];
      mcomddot = obj.robot_mass*(compp*sdot(1)^2+comp/(2*obj.delta_s)*(sdot(2)^2-sdot(1)^2));
      dmcomddotdsdot = obj.robot_mass*[(compp-comp/(2*obj.delta_s))*2*sdot(1) comp/(2*obj.delta_s)*2*sdot(2)];
      dmcomddotdcomp = obj.robot_mass/(2*obj.delta_s)*(sdot(2)^2-sdot(1)^2)*eye(3);
      dmcomddotdcompp = obj.robot_mass*sdot(1)^2*eye(3);
      c(1:3) = force_total-[0;0;obj.robot_mass*obj.g]-mcomddot;
      dc(1:3,obj.comp_idx) = -dmcomddotdcomp;
      dc(1:3,obj.compp_idx) = -dmcomddotdcompp;
      dc(1:3,obj.sdot_idx) = -dmcomddotdsdot;
    end
    
  end
end