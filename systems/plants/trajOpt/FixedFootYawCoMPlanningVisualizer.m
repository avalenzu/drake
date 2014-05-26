classdef FixedFootYawCoMPlanningVisualizer 
  properties(SetAccess = protected)
    robot_mass % The mass of the robot
    t_knot % The time knot for planning
    g % The gravitational acceleration
    num_fsrc_cnstr % An integer. The total number of FootStepRegionContactConstraint
    fsrc_cnstr % A cell array. All the FootStepRegionContactConstraint object
    F2fsrc_map % A cell arry. obj.fsrc_cnstr{F2fsrc_map{i}(j)} is the FootStepContactRegionConstraint corresponds to the force x(obj.F_idx{i}{j})
    yaw % A 1 x num_fsrc_cnstr double vector. yaw(i) is the yaw angle for obj.fsrc_cnstr{i}
  end
  
  properties(Access = protected)
    A_force % A cell array.  A_force{i} = obj.fsrc_cnstr{i}.force, which is a 3 x obj.fsrc_cnstr[i}.num_edges matrix
    A_xy,b_xy,rotmat
  end
  
  methods
    function obj = FixedFootYawCoMPlanningVisualizer(robot_mass,t_knot,g,fsrc_cnstr,yaw,F2fsrc_map,A_force,A_xy,b_xy,rotmat)
      obj.robot_mass = robot_mass;
      obj.t_knot = t_knot;
      obj.g = g;
      obj.num_fsrc_cnstr = length(fsrc_cnstr);
      obj.fsrc_cnstr = fsrc_cnstr;
      obj.F2fsrc_map = F2fsrc_map;
      obj.yaw = yaw;
      obj.A_xy = A_xy;
      obj.b_xy = b_xy;
      obj.rotmat = rotmat;
      obj.A_force = A_force;
    end
    
    function draw(obj,t,com_traj,foot_pos,F)
      com = com_traj.eval(t);
      plot3(com(1),com(2),com(3),'o','MarkerSize',10);
      t_knot_idx = find(obj.t_knot<=t,1,'last');
      for i = 1:length(obj.F2fsrc_map{t_knot_idx})
        fsrc_idx = obj.F2fsrc_map{t_knot_idx}(i);
        num_body_contact_pts = obj.fsrc_cnstr{fsrc_idx}.num_contact_pts;
        foot_contact_pos = bsxfun(@times,ones(1,num_body_contact_pts),  obj.A_xy(:,:,fsrc_idx)*foot_pos(:,fsrc_idx)+obj.b_xy(:,:,fsrc_idx))+obj.rotmat(:,:,fsrc_idx)*obj.fsrc_cnstr{fsrc_idx}.body_contact_pts;
        plot3([foot_contact_pos(1,:) foot_contact_pos(1,1)],[foot_contact_pos(2,:) foot_contact_pos(2,1)],[foot_contact_pos(3,:) foot_contact_pos(3,1)]);
        force = obj.A_force{fsrc_idx}*F{t_knot_idx}{i};
        force_start = foot_contact_pos;
        force_end = foot_contact_pos+force/(obj.robot_mass*obj.g);
        for j = 1:num_body_contact_pts
          plot3([force_start(1,j) force_end(1,j)],[force_start(2,j) force_end(2,j)],[force_start(3,j) force_end(3,j)]);
        end
      end
    end
  end
end