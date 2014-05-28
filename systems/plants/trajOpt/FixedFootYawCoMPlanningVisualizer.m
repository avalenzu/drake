classdef FixedFootYawCoMPlanningVisualizer < MultiVisualizer
  properties(SetAccess = protected)
    robot_mass % The mass of the robot
    t_knot % The time knot for planning
    g % The gravitational acceleration
    num_fsrc_cnstr % An integer. The total number of FootStepRegionContactConstraint
    fsrc_cnstr % A cell array. All the FootStepRegionContactConstraint object
    F2fsrc_map % A cell arry. obj.fsrc_cnstr{F2fsrc_map{i}(j)} is the FootStepContactRegionConstraint corresponds to the force x(obj.F_idx{i}{j})
    yaw % A 1 x num_fsrc_cnstr double vector. yaw(i) is the yaw angle for obj.fsrc_cnstr{i}
    
  end
  
  methods
    function obj = FixedFootYawCoMPlanningVisualizer(robot_mass,g,fsrc_cnstr,fsrc_tspan,com_frame,fsrc_frame,zmp_frame)
      % @param fsrc_tspan    A obj.num_fsrc_cnstr x 2 matrix. fsrc_tspan(i,:) is the time
      % span for the i'th FootStepRegionContactConstraint
      com_visualizer = CoMVisualizer(com_frame);
      force_normalizer = robot_mass*g;
      fsrc_visualizer = cell(length(fsrc_cnstr),1);
      for i = 1:length(fsrc_cnstr)
        fsrc_visualizer{i} = FootStepRegionContactVisualizer(fsrc_cnstr{i},force_normalizer,fsrc_tspan(i,:),fsrc_frame{i});
      end
      zmp_visualizer = ZMPVisualizer(zmp_frame);
      obj = obj@MultiVisualizer([{com_visualizer};fsrc_visualizer;{zmp_visualizer}]);
    end
    
  end
end