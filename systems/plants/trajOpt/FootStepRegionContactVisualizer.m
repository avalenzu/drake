classdef FootStepRegionContactVisualizer < Visualizer
  properties(SetAccess = protected)
    fsrc_cnstr % A FootStepRegionContactConstraint
    force_normalizer % A positive scalar. The force is normalized by obj.force_scale for drawing
    tspan % A 1 x 2 vector. The time span of this constraint being active
    lcmgl
  end
  
  properties(Access = protected)
    yaw_cache;
    A_xy_cache;
    b_xy_cache;
    rotmat_cache;
  end
  
  methods
    function obj = FootStepRegionContactVisualizer(fsrc_cnstr,force_normalizer,tspan,frame,lcmgl_name)
      % @param fsrc_cnstr   A FootStepRegionContactConstraint object
      % @param force_normalizer  A positive scaler. 
      % @param tspan    A 1 x 2 double vector. The time span of this constraint being
      % active
      % @param lcmgl_name   A string, the name of the lcmgl object. This should be unique
      % for each visualizer
      if(~isa(fsrc_cnstr,'FootStepRegionContactConstraint'))
        error('Drake:FootStepRegionContactVisualizer: The input should be a FootStepRegionContactConstraint');
      end
      if(~isnumeric(force_normalizer) || numel(force_normalizer) ~= 1 || force_normalizer<=0)
        error('Drake:FootStepRegionContactVisualizer:force_normalizer should be positive');
      end
      if(~isnumeric(tspan) || size(tspan,1) ~= 1 || size(tspan,2) ~= 2 || tspan(1)>tspan(2))
        error('Drake:FootStepRegionContactVisualizer:invalid tspan');
      end
      if(~isa(frame,'CoordinateFrame'))
        error('Drake:FootStepRegionContactVisualizer:the input should be a CoordinateFrame');
      end
      if(~ischar(lcmgl_name))
        error('Drake:FootStepRegionContactVisualizer: the input should be a string');
      end
      obj = obj@Visualizer(frame);
     
      obj.fsrc_cnstr = fsrc_cnstr;
      obj.force_normalizer = force_normalizer;
      obj.tspan = tspan;
      obj.yaw_cache = inf;
      
      obj.lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton,lcmgl_name);
      
    end
    
    function draw(obj,t,y)
      % y = [x;y;yaw;force_x1;force_y1;force_z1;...;force_xN;force_yN;force_zN];
      if(t>=obj.tspan(1) && t<=obj.tspan(2))
        yaw = y(3);
        if(abs(yaw-obj.yaw_cache)<1e3)
          rotmat = obj.rotmat_cache;
          A_xy = obj.A_xy_cache;
          b_xy = obj.b_xy_cache;
        else
          [rotmat,A_xy,b_xy] = obj.fsrc_cnstr.foot_step_region_cnstr.bodyTransform(yaw);
          obj.rotmat_cache = rotmat;
          obj.A_xy_cache = A_xy;
          obj.b_xy_cache = b_xy;
        end
        foot_contact_pos = bsxfun(@times,ones(1,obj.fsrc_cnstr.num_contact_pts),A_xy*y(1:2)+b_xy)+rotmat*obj.fsrc_cnstr.body_contact_pts;
        conv_hull_idx = convhull(foot_contact_pos(1,:),foot_contact_pos(2,:));
        force = reshape(y(4:end),3,obj.fsrc_cnstr.num_contact_pts);
        force_normalized = force/obj.force_normalizer;         
        for i = 1:obj.fsrc_cnstr.num_contact_pts
          obj.lcmgl.glColor3f(0,0,1);
          obj.lcmgl.line3(foot_contact_pos(1,conv_hull_idx(i)),foot_contact_pos(2,conv_hull_idx(i)),foot_contact_pos(3,conv_hull_idx(i)),...
            foot_contact_pos(1,conv_hull_idx(i+1)),foot_contact_pos(2,conv_hull_idx(i+1)),foot_contact_pos(3,conv_hull_idx(i+1)));
          obj.lcmgl.line3(foot_contact_pos(1,conv_hull_idx(i)),foot_contact_pos(2,conv_hull_idx(i)),foot_contact_pos(3,conv_hull_idx(i)),...
            force_normalized(1,conv_hull_idx(i))+foot_contact_pos(1,conv_hull_idx(i)),force_normalized(2,conv_hull_idx(i))+foot_contact_pos(2,conv_hull_idx(i)),force_normalized(3,conv_hull_idx(i))+foot_contact_pos(3,conv_hull_idx(i)));
        end
      end
      obj.lcmgl.switchBuffers;
    end
  end
end