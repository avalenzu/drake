classdef FixedFootYawCoMPlanning
  % this planner takes the input FootStepRegionConstraint and FIXED yaw angle positions,
  % and output a dynamically feasible CoM trajectory of the robot and the corresponding
  % contact forces.
  properties(SetAccess = protected)
    robot_mass
    t_knot
    g
    nT
  end
  
  methods
    function obj = FixedFootYawCoMPlanning(robot_mass,t,varargin)
      % obj =
      % FixedFootYawCoMPlanning(robot_mass,t,foot_step_region_contact_cnstr1,foot_step_region_contact_cnstr2,...)
      % @properties robot_mass    The mass of the robot
      % @properties t             The time knot for planning
      % @properties foot_step_region_contact_cnstr    A FootStepRegionContactConstraint
      if(~isnumeric(robot_mass))
        error('robot mass should be numeric');
      end
      sizecheck(robot_mass,[1,1]);
      if(robot_mass<=0)
        error('robot mass should be positive');
      end
      obj.robot_mass = robot_mass;
      if(~isnumeric(t))
        error('t should be numeric');
      end
      obj.t_knot = reshape(unique(t),1,[]);
      obj.nT = length(obj.t_knot);
      obj.g = 9.81;
      
    end
  end
end