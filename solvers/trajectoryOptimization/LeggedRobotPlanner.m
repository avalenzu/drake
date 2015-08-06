classdef LeggedRobotPlanner
  properties
    contact_wrench_struct
    num_feet
    r_ee_inds
    v_ee_inds
    feet
  end
  methods
    function obj = LeggedRobotPlanner(foot_names, foot_pts, radii)
      obj.num_feet = numel(foot_names);
      for i = 1:numel(foot_names)
        obj.feet.(foot_names{i}).pts_in_body = foot_pts{i};
        obj.feet.(foot_names{i}).radii = radii{i};
      end
    end

    function [obj, prog] = constructNLP(obj, m, I, varargin)
      prog = RigidBodySymplecticTrajectoryOptimization(m, I, obj.contact_wrench_struct, varargin{:});
      [obj, prog] = addFootVariables(obj, prog);
      [obj, prog] = addPositionConstraints(obj, prog);
      [obj, prog] = addVelocityConstraints(obj, prog);
      [obj, prog] = addDynamicConstraints(obj, prog);
    end

    function obj = addFootstep(obj, foot, pos, normal, mu, knots)
      import drakeFunction.*
      import drakeFunction.geometry.*
      obj.contact_wrench_struct(end+1).foot = foot;
      obj.contact_wrench_struct(end).knots = knots;
      obj.contact_wrench_struct(end).num_forces = 1;
      obj.contact_wrench_struct(end).pt_in_world = pos;
      obj.contact_wrench_struct(end).normal = normal;
      obj.contact_wrench_struct(end).mu = mu;

      lb = obj.contact_wrench_struct(end).pt_in_world;
      ub = lb;
      obj.contact_wrench_struct(end).constraint{1} = BoundingBoxConstraint(lb, ub);
      obj.contact_wrench_struct(end).vars{1} = {'p'};

      for i = 1:size(obj.feet.(foot).pts_in_body, 2)
        lb = 0;
        ub = obj.feet.(foot).radii(i)^2;
        relative_position_fcn = Identity(3) - Identity(3) - compose(RotateVectorByQuaternion(), [Identity(4); Affine(zeros(3,0), obj.feet.(foot).pts_in_body(:,i))]);
        obj.contact_wrench_struct(end).constraint{1+i} = DrakeFunctionConstraint(lb, ub, ...
          compose(euclidean.NormSquared(3),relative_position_fcn));
        obj.contact_wrench_struct(end).vars{1+i} = {'p','r','z'};
      end
      
      offset = 1 + size(obj.feet.(foot).pts_in_body, 2);
      % dot(f, normal) > 0
      lb = 0;
      ub = Inf;
      constraint = LinearConstraint(lb, ub, normal');
      obj.contact_wrench_struct(end).constraint{offset + 1} = constraint;
      obj.contact_wrench_struct(end).vars{offset + 1} = {'f'};
      
      % dot(f, normal)^2 - mu^2 ||f||^2 > 0
      % f'*normal*normal'*f - f'*mu^2*eye(3)*f > 0
      % f'*(normal*normal' - mu^2*eye(3))*f > 0
      Q = normal*normal' - 1/(mu^2+1)*eye(3);
      constraint = QuadraticConstraint(lb, ub, Q, zeros(3,1));
      obj.contact_wrench_struct(end).constraint{offset + 2} = constraint;
      obj.contact_wrench_struct(end).vars{offset + 2} = {'f'};
    end

    function [obj, prog] = addFootVariables(obj, prog)
      for name = fieldnames(obj.feet)'
        foot = name{1};

        var_names = cellStrCat(sprintf('r_%s_', foot), num2cellStr(1:prog.N), '_', {'x', 'y', 'z'});
        prog = prog.addDecisionVariable(3*prog.N, var_names);
        obj.feet.(foot).r_inds = reshape(prog.num_vars - 3*prog.N + (1:3*prog.N), [3, prog.N]);

        var_names = cellStrCat(sprintf('v_%s_', foot), num2cellStr(1:prog.N), '_', {'x', 'y', 'z'});
        prog = prog.addDecisionVariable(3*prog.N, var_names);
        obj.feet.(foot).v_inds = reshape(prog.num_vars - 3*prog.N + (1:3*prog.N), [3, prog.N]);
      end
    end

    function [obj, prog] = addPositionConstraints(obj, prog)
      p_inds = [];
      r_foot_inds = [];
      for i = 1:numel(obj.contact_wrench_struct)
        foot = obj.contact_wrench_struct(i).foot;
        knots = obj.contact_wrench_struct(i).knots;
        p_inds = [p_inds, prog.p_inds_by_contact{i}];
        r_foot_inds = [r_foot_inds, obj.feet.(foot).r_inds(:, knots)];
      end
      ncons = numel(p_inds);
      lb = zeros(ncons,1);
      ub = zeros(ncons,1);
      A = [speye(ncons), -speye(ncons)];
      xinds = [p_inds(:); r_foot_inds(:)];
      prog = prog.addConstraint(LinearConstraint(lb, ub, A), xinds);
    end

    function [obj, prog] = addVelocityConstraints(obj, prog)
    end

    function [obj, prog] = addDynamicConstraints(obj, prog)
    end
  end
end
