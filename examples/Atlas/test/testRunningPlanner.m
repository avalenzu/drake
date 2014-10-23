function [sol,robot_vis,v,cdfkp] = testRunningPlanner(seed,stride_length,major_iteration_limit, suffix,options)
  checkDependency('lcmgl');
  if nargin < 1, seed = []; end
  if (nargin < 2 || isempty(stride_length)), stride_length = 2; end
  if (nargin < 3 || isempty(major_iteration_limit)), major_iteration_limit = 200; end
  if (nargin < 4 || isempty(suffix)), suffix = 'testRunning'; end
  if (nargin < 5 || isempty(options))
    options = defaultOptionsStruct();
  else
    options = parseOptionsStruct(options);
  end

  % Construct RigidBodyManipulator
  w = warning('off','Drake:RigidBody:SimplifiedCollisionGeometry');
  warning('off','Drake:RigidBody:NonPositiveInertiaMatrix');
  warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
  warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
  options.floating = true;
  options.terrain = RigidBodyFlatTerrain;
  atlas_urdf = [getDrakePath,'/examples/Atlas/urdf/atlas_convex_hull.urdf'];
  atlas_vis_urdf = [getDrakePath,'/examples/Atlas/urdf/atlas_minimal_contact.urdf'];
  robot = RigidBodyManipulator(atlas_urdf,options);
  robot = robot.replaceContactShapesWithCHull(robot.findLinkInd('l_hand'),1);
  robot = robot.replaceContactShapesWithCHull(robot.findLinkInd('r_hand'),1);
  robot = compile(robot);
  robot_vis = RigidBodyManipulator(atlas_vis_urdf,options);
  if options.add_obstacle
    %robot = addObstacle(robot,0.7*(stride_length/2));
    %robot_vis = addObstacle(robot_vis,0.7*(stride_length/2));
    robot = addObstacle(robot,0);
    robot_vis = addObstacle(robot_vis,0);
  end
  %mu = 1.16; % rubber on rubber
  mu = 1; % rubber on rubber
  warning(w);
  plan_publisher = RobotPlanPublisherWKeyFrames('CANDIDATE_MANIP_PLAN',true,robot.getStateFrame.coordinates);


  % Create convenience variables
  nq = robot.getNumPositions();
  nu = robot.getNumInputs();
  %nv = robot.getNumVelocities();
  %world = robot.findLinkInd('world');
  l_foot = robot.findLinkInd('l_foot');
  r_foot = robot.findLinkInd('r_foot');
  l_uleg = robot.findLinkInd('l_uleg');
  r_uleg = robot.findLinkInd('r_uleg');
  head = robot.findLinkInd('head');
  neck_idx = robot.getBody(robot.findJointInd('neck_ay')).position_num;
  freeze_idx = neck_idx;
  r_foot_bottom = robot.getBody(r_foot).getTerrainContactPoints();
  l_foot_toe    = robot.getBody(l_foot).getTerrainContactPoints('toe');
  l_foot_heel   = robot.getBody(l_foot).getTerrainContactPoints('heel');
  arm_idx = findJointIndices(robot,'arm');
  leg_idx = findJointIndices(robot,'ak');
  knee_idx = findJointIndices(robot,'kny');
  back_idx = findJointIndices(robot,'back');
  wrist_idx = [findJointIndices(robot,'uwy');findJointIndices(robot,'mwx');findJointIndices(robot,'ely');findJointIndices(robot,'shx')];

  % Construct visualization tools
  v = constructVisualizer(robot_vis);

  % Load nominal data
  nomdata = load([getDrakePath(), '/examples/Atlas/data/atlas_fp.mat']);
  qstar = nomdata.xstar(1:nq);
  v.draw(0,qstar);
  q0 = qstar;
  q0(neck_idx) = 0;
  if options.start_from_standing
    assert(~isempty(options.stride_filename));
    S = load(options.stride_filename);
    options.q_apex = S.sol.q(:,end);
    options.comdot_apex = S.sol.comdot(:,end);
    qf = options.q_apex;
    qf(1) = qf(1)/2;
  elseif options.add_obstacle
    assert(~isempty(options.stride_filename));
    S = load(options.stride_filename);
    options.q_apex = S.sol.q(:,end);
    options.comdot_0 = S.sol.comdot(:,1);
    options.comdot_apex = S.sol.comdot(:,end);
    options.com_0 = S.sol.com(:,1);
    options.com_apex = S.sol.com(:,end);
    q0 = S.sol.q(:,1);
    %q0(1) = -stride_length/4;
    q0(1) = -1.2;
    qf = options.q_apex;
    qf(1) = 0.8;
    H0 = S.sol.H(:,1);
    Hf = S.sol.H(:,end);
    Hdot0 = S.sol.Hdot(:,1);
    Hdotf = S.sol.Hdot(:,end);
  else
    qf = q0;
    qf(1) = stride_length/2;
  end
  com_0 = robot.getCOM(q0);
  com_f = robot.getCOM(qf);

  % Set up time limits
  if options.start_from_standing
    N = 19;
  else
    %N = 16;
    N = options.N;
  end
  T = stride_length/2/options.speed;
  if options.start_from_standing
    tf_range = [0,10*T];
    h_min = 1/(2*N)*T; h_max_stance = 2/N*T; h_max_flight = 2/N*T;
  elseif options.add_obstacle
    tf_range = [0,10*T];
    h_min = 1/(2*N)*T; h_max_stance = 2/N*T; h_max_flight = 2/N*T;
  else
    if options.fix_q
      tf_range = T*[0.1,10];
      h_min = 3/(4*N)*T; h_max_stance = 5/N*T; h_max_flight = 5/N*T;
    else
      tf_range = T*[1,1];
      %tf_range = T*[0.2,10];
      %tf_range = [0.25,1];
      %h_min = 3/(4*N)*T; h_max_stance = 4/(2*N)*T; h_max_flight = 2/N*T;
      h_min = 1/(3*N)*T; h_max_stance = 2/N*T; h_max_flight = 2/N*T;
      %h_min = 0.005; h_max_stance = 0.03; h_max_flight = h_max_stance;
    end
  end

  % Set up cost variables
  if 0 && ~isempty(seed)
    q_nom = seed.q;
  else
    q_nom = bsxfun(@times,qstar,ones(1,N));
  end
  Q = 1e3*eye(nq);
  %Q = 1e-4*eye(nq);
  Q(1,1) = 0;
  Q(2,2) = 0;
  Q(6,6) = 0;
  %Q(knee_idx,knee_idx) = 1e3*ones(numel(knee_idx));
  %Q(arm_idx,arm_idx) = 1e3*eye(numel(arm_idx));
  Qv = 1e1*eye(nq);
  %Qv = 1e-4*eye(nq);
  %Qv(back_idx,back_idx) = 1e1*eye(numel(back_idx));
  %Qv(arm_idx,arm_idx) = eye(numel(arm_idx));
  Q_comddot = diag([1e1,1e2,1e1]);
  Q_contact_force = 0e-4*eye(3);
  %Q_comddot = diag([1e1,1e0,1e2]);
  %Q_contact_force = 1*diag([1e-2,1e-4,1e-2]);
  R = options.kTorque*eye(nu);

  % Create collision avoidance constraints
  if options.enable_collision
    % Consider all bodies
    min_distance_constraint.flight = MinDistanceConstraint(robot,options.min_distance);
    if options.start_from_standing
      % Ignore both feet
      active_collision_options.body_idx = setdiff(1:robot.getNumBodies(),[l_foot,r_foot,l_uleg,r_uleg]);
      min_distance_constraint.double_stance = MinDistanceConstraint(robot,options.min_distance,active_collision_options);
      % Ignore left foot
      active_collision_options.body_idx = setdiff(1:robot.getNumBodies(),[l_foot]);
      min_distance_constraint.stance = MinDistanceConstraint(robot,options.min_distance,active_collision_options);
    else
      % Ignore left foot
      active_collision_options.body_idx = setdiff(1:robot.getNumBodies(),l_foot);
      min_distance_constraint.stance = MinDistanceConstraint(robot,options.min_distance,active_collision_options);
    end
    if options.n_interp_points > 0
      interp_min_distance_constraint.flight = ...
        generateInterpolatedMinDistanceConstraint(min_distance_constraint.stance,(1:options.n_interp_points)/(options.n_interp_points+1));
      interp_min_distance_constraint.stance = ...
        generateInterpolatedMinDistanceConstraint(min_distance_constraint.stance,(1:options.n_interp_points)/(options.n_interp_points+1));
      if options.start_from_standing
        interp_min_distance_constraint.double_stance = ...
          generateInterpolatedMinDistanceConstraint(min_distance_constraint.double_stance,(1:options.n_interp_points)/(options.n_interp_points+1));
      end
    end
  end

  % Create gaze constraint
  if options.constrain_head_gaze
    gaze_constraint = WorldGazeDirConstraint(robot,head,[1;0;0],[1;0;0],options.head_gaze_tol);
  end

  % Set up linearized friction cone edges
  num_edges = 3;
  FC_angles = linspace(0,2*pi,num_edges+1);FC_angles(end) = [];
  FC_axis = [0;0;1];
  FC_perp1 = rotx(pi/2)*FC_axis;
  FC_perp2 = cross(FC_axis,FC_perp1);
  FC_edge = bsxfun(@plus,FC_axis,mu*(bsxfun(@times,cos(FC_angles),FC_perp1) + ...
    bsxfun(@times,sin(FC_angles),FC_perp2)));
  FC_edge = robot.getMass()*norm(robot.getGravity)*bsxfun(@rdivide,FC_edge,sqrt(sum(FC_edge.^2,1)));

  % Create stance constraints
  if options.start_from_standing
      in_stance.right = 1:11;
      in_stance.toe  = 1:17;
      in_stance.heel = 1:14;
  else
    in_stance.right = [];
    if options.toe_first
      in_stance.toe = 5:13;
      in_stance.heel = 8:10;
    else
      %in_stance.toe = 5:13;
      %in_stance.heel = 5:10;
      heel_down_idx = ceil(N/4);
      toe_down_idx = ceil(N/4);
      heel_off_idx = ceil(3*N/4);
      toe_off_idx = ceil(4*N/5);
      in_stance.toe = toe_down_idx:toe_off_idx;
      in_stance.heel = heel_down_idx:heel_off_idx;
    end
  end
  in_stance.left = union(in_stance.toe,in_stance.heel);
  in_stance.left = setdiff(in_stance.left,in_stance.right);
  in_stance.total = union(in_stance.left,in_stance.right);
  in_stance.double = intersect(in_stance.left,in_stance.right);
  in_flight = setdiff(1:N,in_stance.total);

  contact_wrench_struct(1).active_knot = in_stance.toe(2:end);
  contact_wrench_struct(1).cw = ...
    LinearFrictionConeWrench(robot,l_foot,l_foot_toe,FC_edge);
  contact_wrench_struct(2).active_knot = in_stance.heel(2:end);
  contact_wrench_struct(2).cw = ...
    LinearFrictionConeWrench(robot,l_foot,l_foot_heel,FC_edge);
  if options.start_from_standing
    contact_wrench_struct(3).active_knot = in_stance.right;
    contact_wrench_struct(3).cw = ...
      LinearFrictionConeWrench(robot,r_foot,r_foot_bottom,FC_edge);
  end

  % Kinematic constraints
  lb = repmat([NaN; NaN; 0],1,2);
  ub = repmat([NaN; NaN; 0],1,2);
  position_cnstr.toe   = WorldPositionConstraint(robot,l_foot,l_foot_toe,lb,ub);
  position_cnstr.heel   = WorldPositionConstraint(robot,l_foot,l_foot_heel,lb,ub);
  stationary_cnstr.toe = WorldFixedPositionConstraint(robot,l_foot,l_foot_toe);
  stationary_cnstr.heel = WorldFixedPositionConstraint(robot,l_foot,l_foot_heel);
  stance_clearance_cnstr.toe = WorldPositionConstraint(robot,l_foot,l_foot_toe,lb,NaN(size(lb)));
  stance_clearance_cnstr.heel = WorldPositionConstraint(robot,l_foot,l_foot_heel,lb,NaN(size(lb)));
  if options.start_from_standing
    lb = repmat([NaN; NaN; 0],1,4);
    ub = repmat([NaN; NaN; 0],1,4);
    position_cnstr.right   = WorldPositionConstraint(robot,r_foot,r_foot_bottom,lb,ub);
    stationary_cnstr.right = WorldFixedBodyPoseConstraint(robot,r_foot);
  end

  % Create trajectory optimization
  cdfkp = ComDynamicsFullKinematicsPlanner(robot,N,tf_range,Q_comddot,Qv,Q,q_nom,Q_contact_force,contact_wrench_struct,options);
  if options.visualize
    %cdfkp = cdfkp.addDisplayFunction(@(q)displayCallback(in_stance.total,N,q),[cdfkp.h_inds(:);reshape(cdfkp.com_inds(3,:),[],1);reshape(cdfkp.comdot_inds(1,:),[],1);cdfkp.q_inds(:)]);
    cdfkp = cdfkp.addDisplayFunction(@(q)displayCallback(plan_publisher,in_stance.total,N,q),[cdfkp.h_inds(:);reshape(cdfkp.com_inds(3,:),[],1);reshape(cdfkp.comdot_inds(1,:),[],1);cdfkp.q_inds(:)]);
  end

  % constrain back
%cdfkp = cdfkp.addConstraint(ConstantConstraint(zeros(N,1)),cdfkp.q_inds(8,:));
  % Add cost on angular momentum
  Q_H = diag([1,1,1]);
  cdfkp = cdfkp.addCost(QuadraticConstraint(-Inf,Inf,kron(eye(cdfkp.N),Q_H),zeros(3*cdfkp.N,1)),cdfkp.H_inds);
  
  % Add cost on actuator work
  %R1 = drakeFunction.frames.realCoordinateSpace(1);
  %R3 = drakeFunction.frames.realCoordinateSpace(3);
  %kinetic_energy = drakeFunction.kinematic.KineticEnergy(robot);
  %potential_energy = drakeFunction.Linear(R3,R1,-robot.getMass()*robot.getGravity()');
  %kinetic_energy_all = kinetic_energy.duplicate(N);
  %potential_energy_all = potential_energy.duplicate(N);
  %mechanical_energy_all = kinetic_energy_all + potential_energy_all;
  %diff_mechanical_energy = compose(drakeFunction.Difference(R1,N),mechanical_energy_all);
  %approx_actuator_work = compose(drakeFunction.euclidean.NormSquared(diff_mechanical_energy.getOutputFrame),diff_mechanical_energy);
  %approx_actuator_work = compose(drakeFunction.euclidean.SmoothNorm(diff_mechanical_energy.getOutputFrame,1e-6),diff_mechanical_energy);
  %kW = 1e1;
  %cdfkp = cdfkp.addCost(DrakeFunctionConstraint(-Inf,Inf,kW*approx_actuator_work),[cdfkp.x_inds(:);cdfkp.com_inds(:)]);

  % Add Timestep bounds
  cdfkp = cdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(h_min*ones(numel(in_flight)-1,1),h_max_flight*ones(numel(in_flight)-1,1)),cdfkp.h_inds(in_flight(1:end-1)));
  cdfkp = cdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(h_min*ones(numel(in_stance.total),1),h_max_stance*ones(numel(in_stance.total),1)),cdfkp.h_inds(in_stance.total(1:end)));
  cdfkp = cdfkp.addCost(QuadraticConstraint(-Inf,Inf,eye(N-1),zeros(N-1,1)),cdfkp.h_inds);
  %cdfkp = cdfkp.addCost(LinearConstraint(-Inf,Inf,-ones(1,N-1)),cdfkp.h_inds);

  % Add GRF bounds
  for i = 1:numel(cdfkp.lambda_inds)
    cdfkp = cdfkp.addConstraint(BoundingBoxConstraint(zeros(numel(cdfkp.lambda_inds{i}),1),options.max_lambda*ones(numel(cdfkp.lambda_inds{i}),1)),cdfkp.lambda_inds{i});
  end

  %cdfkp = cdfkp.addConstraint(BoundingBoxConstraint(30*pi/180*ones(2,N),60*pi/180*ones(2,N)),cdfkp.q_inds(knee_idx,:));

  % Add Velocity constriants
  if options.enable_velocity_limits
    arm_v_max = pi;
    cdfkp = cdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(-arm_v_max*ones(numel(arm_idx),N),arm_v_max*ones(numel(arm_idx),N)),cdfkp.v_inds(arm_idx,:));
    leg_v_max = 3*pi;
    cdfkp = cdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(-leg_v_max*ones(numel(leg_idx),N),leg_v_max*ones(numel(leg_idx),N)),cdfkp.v_inds(leg_idx,:));
    back_v_max = 0.1*pi;
    %back_v_max = 0.1*pi;
    cdfkp = cdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(-back_v_max*ones(numel(back_idx),N),back_v_max*ones(numel(back_idx),N)),cdfkp.v_inds(back_idx,:));
    fb_v_max = 5;
    cdfkp = cdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(-fb_v_max*ones(3,N),fb_v_max*ones(3,N)),cdfkp.v_inds(1:3,:));
  end

  if options.fix_q
    assert(~isempty(seed));
    x_seed = seed.x_sol;
    q_seed = reshape(x_seed(cdfkp.q_inds(:)),[nq,N]);
    com_seed = reshape(x_seed(cdfkp.com_inds(:)),[3,N]);
    cdfkp = cdfkp.addConstraint(ConstantConstraint(q_seed),cdfkp.q_inds);
    cdfkp = cdfkp.addConstraint(ConstantConstraint(com_seed),cdfkp.com_inds);
  else
    % Add pitch limits
    cdfkp = cdfkp.addConstraint(BoundingBoxConstraint(-pi/4*ones(N,1),pi/4*ones(N,1)),cdfkp.q_inds(5,:));
  end

  % Add initial contition constraints
  if options.start_from_standing
    cdfkp = cdfkp.addConstraint(ConstantConstraint([0;0]), ...
      cdfkp.com_inds(1:2,1));
    cdfkp = cdfkp.addConstraint(ConstantConstraint(q0(3:end)), ...
      cdfkp.q_inds(3:end,1));
    cdfkp = cdfkp.addConstraint(ConstantConstraint(zeros(3,1)), ...
      cdfkp.comdot_inds(1:3,1));
  else
    if options.add_obstacle
      %options.com_tol = 0.2;
      %cdfkp = cdfkp.addConstraint(BoundingBoxConstraint(q0(3:end)-options.joint_tol,q0(3:end)+options.joint_tol), ...
      %cdfkp.q_inds(3:end,1));
      not_arm_idx = setdiff(3:robot.getNumPositions(),arm_idx);
      cdfkp = cdfkp.addConstraint(BoundingBoxConstraint(q0(arm_idx)-2*options.joint_tol,q0(arm_idx)+2*options.joint_tol), ...
        cdfkp.q_inds(arm_idx,1));
      cdfkp = cdfkp.addConstraint(BoundingBoxConstraint(q0(not_arm_idx)-options.joint_tol,q0(not_arm_idx)+options.joint_tol), ...
        cdfkp.q_inds(not_arm_idx,1));
      cdfkp = cdfkp.addConstraint(ConstantConstraint(H0), ...
        cdfkp.H_inds(:,1));
      cdfkp = cdfkp.addConstraint(ConstantConstraint(Hdot0), ...
        cdfkp.Hdot_inds(:,1));
      cdfkp = cdfkp.addConstraint(BoundingBoxConstraint(-Inf,q0(1)), ...
        cdfkp.q_inds(1,1));
      cdfkp = cdfkp.addConstraint(BoundingBoxConstraint(options.comdot_0 - options.com_tol*abs(options.comdot_0),options.comdot_0 + options.com_tol*abs(options.comdot_0)), ...
        cdfkp.comdot_inds(:,1));
      %cdfkp = cdfkp.addConstraint(LinearConstraint(0,Inf,[1,-1]), ...
      %cdfkp.comdot_inds(3,[1,2]));
      cdfkp = cdfkp.addConstraint(BoundingBoxConstraint(options.com_0(3) - options.com_tol*abs(options.com_0(3)),options.com_0(3) + options.com_tol*abs(options.com_0(3))), ...
        cdfkp.com_inds(3,1));
    else
      cdfkp = cdfkp.addConstraint(ConstantConstraint([0;0]), ...
        cdfkp.com_inds(1:2,1));
    end
    cdfkp = cdfkp.addConstraint(ConstantConstraint(0), ...
      cdfkp.comdot_inds(3,1));
  end

  % Add final condition constraints
  if options.start_from_standing
    cdfkp = cdfkp.addConstraint(ConstantConstraint(options.comdot_apex), ...
      cdfkp.comdot_inds(:,end));
    cdfkp = cdfkp.addConstraint(ConstantConstraint(options.q_apex(2:end)), ...
      cdfkp.q_inds(2:end,end));
  elseif options.add_obstacle
    cdfkp = cdfkp.addConstraint(BoundingBoxConstraint(qf(arm_idx)-2*options.joint_tol,qf(arm_idx)+2*options.joint_tol), ...
      cdfkp.q_inds(arm_idx,end));
    cdfkp = cdfkp.addConstraint(BoundingBoxConstraint(qf(not_arm_idx)-options.joint_tol,qf(not_arm_idx)+options.joint_tol), ...
      cdfkp.q_inds(not_arm_idx,end));
    cdfkp = cdfkp.addConstraint(BoundingBoxConstraint(options.comdot_apex - options.com_tol*abs(options.comdot_apex),options.comdot_apex + options.com_tol*abs(options.comdot_apex)), ...
      cdfkp.comdot_inds(:,end));
    cdfkp = cdfkp.addConstraint(BoundingBoxConstraint(qf(1),Inf), ...
      cdfkp.q_inds(1,end));
    cdfkp = cdfkp.addConstraint(ConstantConstraint(Hf), ...
      cdfkp.H_inds(:,end));
    cdfkp = cdfkp.addConstraint(ConstantConstraint(Hdotf), ...
      cdfkp.Hdot_inds(:,end));
    cdfkp = cdfkp.addConstraint(BoundingBoxConstraint(options.com_apex(3) - options.com_tol*abs(options.com_apex(3)),options.com_apex(3) + options.com_tol*abs(options.com_apex(3))), ...
      cdfkp.com_inds(3,end));
  else
    input_frame = drakeFunction.frames.realCoordinateSpace(N);
    output_frame = drakeFunction.frames.realCoordinateSpace(1);
    avg_speed_constraint_fun = drakeFunction.Linear(input_frame,output_frame,[1,-options.speed*ones(1,N-1)]);
    %cdfkp = cdfkp.addConstraint(ConstantConstraint(stride_length/2), ...
      %cdfkp.com_inds(1,end));
      cdfkp = cdfkp.addConstraint(DrakeFunctionConstraint(0,0,avg_speed_constraint_fun),[cdfkp.com_inds(1,end);cdfkp.h_inds]);
      %cdfkp = cdfkp.addConstraint(DrakeFunctionConstraint(-0.5*options.speed,0.5*options.speed,avg_speed_constraint_fun),[cdfkp.com_inds(1,end);cdfkp.h_inds]);
  end

  % Immobilize specified joints
  if options.freeze_neck
    cdfkp = cdfkp.addConstraint(ConstantConstraint(repmat(q0(freeze_idx),N,1)),cdfkp.q_inds(freeze_idx,:));
  end

  % Add gaze constraint
  if options.constrain_head_gaze
    cdfkp = cdfkp.addRigidBodyConstraint(gaze_constraint,num2cell(1:N));
  end

  % Constrain arms
  if options.start_from_standing
    %cdfkp = cdfkp.addConstraint(ConstantConstraint(linspacevec(q0(wrist_idx),qf(wrist_idx),N-2)),cdfkp.q_inds(wrist_idx,2:end-1));
    %cdfkp = cdfkp.addConstraint(ConstantConstraint(linspacevec(q0(arm_idx),qf(arm_idx),N-2)),cdfkp.q_inds(arm_idx,2:end-1));
  end

  % Add periodicity constraints
  if ~(options.start_from_standing || options.add_obstacle)
    half_periodic_constraint = halfPeriodicConstraint(robot);
    cdfkp = cdfkp.addConstraint(half_periodic_constraint,cdfkp.q_inds(:,[1,end]));
    cdfkp = cdfkp.addConstraint(half_periodic_constraint,cdfkp.v_inds(:,[1,end]));
    cdfkp = cdfkp.addConstraint(LinearConstraint(0,0,[1,-1]),cdfkp.v_inds(1,[2,end]));
    cdfkp = cdfkp.addConstraint(LinearConstraint(zeros(3,1),zeros(3,1),[eye(3),diag([-1,1,1])]), ...
      cdfkp.comdot_inds(1:3,[1,end]));
    %cdfkp = cdfkp.addConstraint(LinearConstraint(zeros(3,1),zeros(3,1),[eye(3),diag([-1,1,1])]), ...
      %cdfkp.comddot_inds(1:3,[1,end]));
    cdfkp = cdfkp.addConstraint(LinearConstraint(zeros(3,1),zeros(3,1),[eye(3),diag([1,-1,1])]), ...
      cdfkp.H_inds(1:3,[1,end]));
    cdfkp = cdfkp.addConstraint(LinearConstraint(zeros(3,1),zeros(3,1),[eye(3),diag([1,-1,1])]), ...
      cdfkp.Hdot_inds(1:3,[1,end]));
  end

  % Add stance kinematic constraints
  cdfkp = cdfkp.addRigidBodyConstraint(position_cnstr.toe, ...
    num2cell(in_stance.toe));
  cdfkp = cdfkp.addRigidBodyConstraint(position_cnstr.heel, ...
    num2cell(in_stance.heel));
  cdfkp = cdfkp.addRigidBodyConstraint(stationary_cnstr.toe, ...
    {in_stance.toe});
  cdfkp = cdfkp.addRigidBodyConstraint(stationary_cnstr.heel, ...
    {in_stance.heel});
  if options.start_from_standing
    cdfkp = cdfkp.addRigidBodyConstraint(position_cnstr.right, ...
      num2cell(in_stance.right));
    cdfkp = cdfkp.addRigidBodyConstraint(stationary_cnstr.right, ...
      {in_stance.right});
  end
  toe_clearance_idx = setdiff(in_stance.total,in_stance.toe);
  heel_clearance_idx = setdiff(in_stance.total,in_stance.heel);
  if ~isempty(toe_clearance_idx)
    cdfkp = cdfkp.addRigidBodyConstraint(stance_clearance_cnstr.toe, ...
      num2cell(toe_clearance_idx));
  end
  if ~isempty(heel_clearance_idx)
    cdfkp = cdfkp.addRigidBodyConstraint(stance_clearance_cnstr.heel, ...
      num2cell(heel_clearance_idx));
  end

  % Add collision avoidance constraints
  if options.enable_collision
    cdfkp = cdfkp.addRigidBodyConstraint(min_distance_constraint.flight,num2cell(in_flight));
    %cdfkp = cdfkp.addRigidBodyConstraint(min_distance_constraint.stance,num2cell(in_flight));
    cdfkp = cdfkp.addRigidBodyConstraint(min_distance_constraint.stance,num2cell(in_stance.left));
    if options.start_from_standing
      cdfkp = cdfkp.addRigidBodyConstraint(min_distance_constraint.double_stance,num2cell(in_stance.double));
    end
    if options.n_interp_points > 0
      for i = in_flight(1:end-1)
        for j = 1:numel(interp_min_distance_constraint.flight)
          cdfkp = cdfkp.addConstraint(interp_min_distance_constraint.flight{j},{cdfkp.q_inds(:,i),cdfkp.q_inds(:,i+1)});
        end
      end
      for i = in_stance.left(1:end-1)
        for j = 1:numel(interp_min_distance_constraint.stance)
          cdfkp = cdfkp.addConstraint(interp_min_distance_constraint.stance{j},{cdfkp.q_inds(:,i),cdfkp.q_inds(:,i+1)});
        end
      end
      if ~isempty(in_stance.double)
        for i = in_stance.double(1:end-1)
          for j = 1:numel(interp_min_distance_constraint.double_stance)
            cdfkp = cdfkp.addConstraint(interp_min_distance_constraint.double_stance{j},{cdfkp.q_inds(:,i),cdfkp.q_inds(:,i+1)});
          end
        end
      end
    end
  end

  % TODO: Set up seed
  if isempty(seed)
    x_seed = zeros(cdfkp.num_vars,1);
    q_seed = linspacevec(q0,qf,N);
    h_seed = T/N;
    v_seed = gradient(q_seed,h_seed);
    com_seed = linspacevec(com_0,com_f,N);
    comdot_seed = gradient(com_seed,h_seed);
    comddot_seed = gradient(comdot_seed,h_seed);
    lambda_seed = sqrt(2)/24;
    x_seed(cdfkp.h_inds) = h_seed;
    x_seed(cdfkp.q_inds(:)) = reshape(q_seed,[],1);
    x_seed(cdfkp.v_inds(:)) = reshape(v_seed,[],1);
    x_seed(cdfkp.com_inds(:)) = reshape(com_seed,[],1);
    x_seed(cdfkp.comdot_inds(:)) = reshape(comdot_seed,[],1);
    x_seed(cdfkp.comddot_inds(:)) = reshape(comddot_seed,[],1);
    for i = 1:numel(cdfkp.lambda_inds)
      x_seed(cdfkp.lambda_inds{i}(:)) = lambda_seed;
    end
  else
    x_seed = seed.x_sol;
    q_seed = reshape(x_seed(cdfkp.q_inds(:)),[nq,N]);
    v_seed = reshape(x_seed(cdfkp.v_inds(:)),[nq,N]);
  end
  h_seed = x_seed(cdfkp.h_inds);
  H = {}; C = {}; J = {};
  contact_wrench = cdfkp.contactWrench(x_seed);
  joint_torque_fun_cell = {};
  qdd_fun = drakeFunction.FiniteDifferenceDerivative(robot.getVelocityFrame(),N);
  qdd_frame = robot.getVelocityFrame();
  tau_frame = drakeFunction.frames.realCoordinateSpace(nu);
  % Assumes only one contact body
  lambda_frame = drakeFunction.frames.realCoordinateSpace(size(reshape(cdfkp.lambda_inds{1},[],N),1));
  Hqdd_fun_cell = {};
  nv = robot.getNumVelocities();
  nx = nq+nv;
  nlambda = lambda_frame.dim;
  for i = 2:N
    kinsol = robot.doKinematics(q_seed(:,i),true);
    [H,C,~,dH,dC] = robot.manipulatorDynamics(q_seed(:,i),v_seed(:,i));
    [~,J,dJ] = robot.forwardKin(kinsol,contact_wrench(i).body,contact_wrench(i).body_pts);
    input_frame = MultiCoordinateFrame.constructFrame({qdd_frame,lambda_frame});
    Hqdd_fun_cell{i-1} = drakeFunction.Linear(qdd_frame,tau_frame,H(7:end,:));
    dH_reshaped = reshape(dH,[nq,nq,nx]);
    dH_flat = reshape(permute(dH_reshaped(7:end,:,:),[2,1,3]),[nq,nu*nx])';
    qdd = (v_seed(:,i)-v_seed(:,i-1))/h_seed(i-1);
    dHqdd_dx = reshape(dH_flat*qdd,[nu,nx]);
    dJTF_dx = [reshape(dJ'*reshape(contact_wrench(i).force,[],1),nq,nq),zeros(nq)];
    dJTF_dx = dJTF_dx(7:end,:);
    dC = dC(7:end,:);
    C = C(7:end);
    dtau_dx_fun_cell{i-1} = drakeFunction.Affine(robot.getStateFrame(),tau_frame,dHqdd_dx+dC-dJTF_dx,C-(dHqdd_dx+dC-dJTF_dx)*[q_seed(:,i);v_seed(:,i)]);
    JtransposeF_fun_cell{i-1} = drakeFunction.Linear(lambda_frame,tau_frame,J(:,7:end)'*kron(eye(size(contact_wrench(i).body_pts,2)),FC_edge));

    %joint_torque_fun_cell{i-1} = drakeFunction.Affine(input_frame,tau_frame,[H,J'*kron(eye(size(contact_wrench(i).body_pts,2)),FC_edge)],C);
  end
  %joint_torque_fun = vertcat(joint_torque_fun_cell{:});
  if options.penalize_approx_torque || options.constrain_approx_torque
    Hqdd_fun = vertcat(Hqdd_fun_cell{:});
    dtau_dx_fun = vertcat(dtau_dx_fun_cell{:});
    JtransposeF_fun = vertcat(JtransposeF_fun_cell{:});
    joint_torque_fun = dtau_dx_fun + Hqdd_fun(qdd_fun) - JtransposeF_fun;
    tmp = eye(2*N); tmp = tmp(2:2:end,:);
    x_to_v = kron(tmp,eye(nv));
    A = [zeros((N-1)*nx,N-1),zeros((N-1)*nx,nx),eye((N-1)*nx),zeros((N-1)*nx,(N-1)*nlambda); ...
         eye(N-1,N*nx+(N-1)*(nlambda+1)); ...
         zeros(N*nv,N-1), x_to_v, zeros(N*nv,(N-1)*nlambda); ...
         zeros((N-1)*nlambda,N*(nx+1)-1), eye((N-1)*nlambda)];
    %input_frame = MultiCoordinateFrame({qdd_fun.getInputFrame(),JtransposeF_fun.getInputFrame()});
    input_frame = drakeFunction.frames.realCoordinateSpace(N*nx+(N-1)*(1+nlambda));
    joint_torque_fun = compose(joint_torque_fun,drakeFunction.Linear(input_frame,joint_torque_fun.getInputFrame(),A));
    joint_torque_inds = [cdfkp.h_inds(:);cdfkp.x_inds(:);reshape(cdfkp.lambda_inds{1}(:,:,2:end),[],1)];
    %keyboard

    if options.penalize_approx_torque
      joint_torque_cost_fun = compose(drakeFunction.euclidean.NormSquared(joint_torque_fun.output_frame,kron(speye(N-1),R)),joint_torque_fun);
      cdfkp = cdfkp.addCost(DrakeFunctionConstraint(-Inf,Inf,joint_torque_cost_fun),joint_torque_inds);
    end

    if options.constrain_approx_torque
      joint_torque_lb = robot.getB()*robot.umin;
      joint_torque_ub = robot.getB()*robot.umax;
      joint_torque_lb = joint_torque_lb(7:end);
      joint_torque_ub = joint_torque_ub(7:end);
      joint_torque_lb = repmat(joint_torque_lb,N-1,1);
      joint_torque_ub = repmat(joint_torque_ub,N-1,1);
      %keyboard
      %cdfkp = cdfkp.addConstraint(LinearConstraint(joint_torque_lb,joint_torque_ub,joint_torque_fun.A),joint_torque_inds);
      %cdfkp = cdfkp.addConstraint(BoundingBoxConstraint(q_seed-0.2,q_seed+0.2),cdfkp.q_inds(:));
      cdfkp = cdfkp.addConstraint(DrakeFunctionConstraint(joint_torque_lb,joint_torque_ub,joint_torque_fun),joint_torque_inds);
    end
    %keyboard
  end

  %keyboard
  if options.constrain_torque || options.penalize_torque
    joint_torque_lb = robot.getB()*robot.umin;
    joint_torque_ub = robot.getB()*robot.umax;
    for i = 2:N-1
      contact_wrench_i = cdfkp.contact_wrench(cellfun(@(x) ismember(i,x),cdfkp.contact_wrench_active_knot));
      num_lambdas = size(cdfkp.lambda_inds{1});
      num_lambdas = prod(num_lambdas(1:2));
      joint_torque_fun = drakeFunction.JointTorques(robot,contact_wrench_i,num_lambdas);
      joint_torque_inds = [cdfkp.h_inds(i-1:i);cdfkp.q_inds(:,i); ...
                           reshape(cdfkp.v_inds(:,i:i+1),[],1); ...
                           reshape(cdfkp.lambda_inds{1}(:,:,i),[],1)];
      double_joint_torque_fun = compose( ...
        drakeFunction.Linear(joint_torque_fun.getOutputFrame(),MultiCoordinateFrame(repmat({joint_torque_fun.getOutputFrame()},1,2)),[eye(nq);eye(nq)]), ...
        joint_torque_fun);
      if options.constrain_torque
        A = zeros(joint_torque_fun.getNumOutputs(), joint_torque_fun.getNumInputs());
        A(:,1) = 0.5*joint_torque_lb;
        A(:,2) = 0.5*joint_torque_lb;
        joint_torque_lb_h_fun = drakeFunction.Linear(joint_torque_fun.getInputFrame(),joint_torque_fun.getOutputFrame(),A);
        A(:,1) = 0.5*joint_torque_ub;
        A(:,2) = 0.5*joint_torque_ub;
        joint_torque_ub_h_fun = drakeFunction.Linear(joint_torque_fun.getInputFrame(),joint_torque_fun.getOutputFrame(),A);
        joint_torque_bounds_h_fun = concatenate(joint_torque_lb_h_fun,joint_torque_ub_h_fun,true);
        cdfkp = cdfkp.addConstraint( ...
          DrakeFunctionConstraint( ...
          [zeros(size(joint_torque_ub));-Inf(size(joint_torque_ub))], ...
          [Inf(size(joint_torque_ub));zeros(size(joint_torque_ub))], ...
          minus(double_joint_torque_fun,joint_torque_bounds_h_fun,true)),joint_torque_inds);
      end
      if options.penalize_torque
        joint_torque_cost_fun = compose(options.kTorque*drakeFunction.euclidean.NormSquared(joint_torque_fun.getOutputFrame()),joint_torque_fun);
        cdfkp = cdfkp.addCost(DrakeFunctionConstraint(-Inf,Inf,joint_torque_cost_fun),joint_torque_inds);
      end
    end
  end

  % Set up solver options
  cdfkp = cdfkp.setSolverOptions('snopt','iterationslimit',1e6);
  %cdfkp = cdfkp.setSolverOptions('snopt','scaleoption',1);
  cdfkp = cdfkp.setSolverOptions('snopt','majoriterationslimit',major_iteration_limit);
  cdfkp = cdfkp.setSolverOptions('snopt','majorfeasibilitytolerance',1e-5);
  cdfkp = cdfkp.setSolverOptions('snopt','majoroptimalitytolerance',5e-4);
  cdfkp = cdfkp.setSolverOptions('snopt','superbasicslimit',2000);
  cdfkp = cdfkp.setSolverOptions('snopt','linesearchtolerance',0.9);
  cdfkp = cdfkp.setSolverOptions('snopt','print',sprintf('snopt_%s.out',suffix));

  % Solve trajectory optimization
  tic
  %profile on;
  [x_sol,~,~] = cdfkp.solve(x_seed);
  %profile off;
  toc

  % Parse trajectory optimization output
  [sol.q,sol.v,sol.h,sol.t,sol.com,sol.comdot,sol.comddot, ...
    sol.H,sol.Hdot,sol.lambda,sol.wrench] = cdfkp.parseSolution(x_sol);
  sol.x_sol = x_sol;
  sol.xtraj= PPTrajectory(foh(sol.t,[sol.q;sol.v]));
  sol.xtraj= sol.xtraj.setOutputFrame(robot_vis.getStateFrame);
  sol.xtraj_one = halfStrideToFullStride(robot_vis,@mirrorAtlasPositions,sol.xtraj);
  sol.xtraj_three = oneStrideToMultipleStrides(robot_vis,sol.xtraj_one,3);
  sol.xtraj_ten = oneStrideToMultipleStrides(robot_vis,sol.xtraj_one,10);
  sol.options = options;
  sol.FC_basis_vectors = FC_edge;
  sol.contact_wrench = cdfkp.contactWrench(sol.x_sol);
  sol.in_stance = in_stance;

  try
    for i = 2:N-1
      q_i = sol.q(:,i);
      v_r = sol.v(:,i+1);
      v_l = sol.v(:,i);
      h = 0.5*(sol.h(i-1)+sol.h(i));
      kinsol = robot.doKinematics(q_i);
      vd = (v_r-v_l)/h;
      [H,C,B] = robot.manipulatorDynamics(q_i,v_l);
      [~,J] = robot.forwardKin(kinsol,sol.contact_wrench(i).body,sol.contact_wrench(i).body_pts);
      f = sol.contact_wrench(i).force;
      sol.u(:,i) = B(7:end,:)\(H(7:end,:)*vd + C(7:end) - J(:,7:end)'*reshape(f,[],1));
    end
    sol.u(:,1) = sol.u(:,end);
  catch ex
    keyboard
  end

  % Save results
  save(fullfile(getDrakePath,'examples','Atlas','data',sprintf('results_%s',suffix)),'sol');
end

function half_periodic_constraint = halfPeriodicConstraint(robot)
  num_symmetry = 12;
  num_equal = 8;
  nq = robot.getNumPositions();

  symmetric_matrix = zeros(2*num_symmetry,2*nq);
  equal_matrix = zeros(num_equal,2*nq);
  initial_indices = 1:nq;
  final_indices   = nq+(1:nq);

  function sym_mat = addSymmetricPair(sym_mat,rows,idx1,idx2)
    sym_mat(rows(1),[initial_indices(idx1) final_indices(idx2)]) = [1 -1];
    sym_mat(rows(2),[initial_indices(idx2) final_indices(idx1)]) = [1 -1];
  end

  function sym_mat = addAntiSymmetricPair(sym_mat,rows,idx1,idx2)
    sym_mat(rows(1),[initial_indices(idx1) final_indices(idx2)]) = [1 1];
    sym_mat(rows(2),[initial_indices(idx2) final_indices(idx1)]) = [1 1];
  end

  function eq_mat = addEquality(eq_mat,row,idx)
    eq_mat(row,[initial_indices(idx) final_indices(idx)]) = [1 -1];
  end

  function eq_mat = addOpposite(eq_mat,row,idx)
    eq_mat(row,[initial_indices(idx) final_indices(idx)]) = [1 1];
  end

  l_arm_usy_idx = robot.getBody(robot.findJointInd('l_arm_usy')).position_num;
  r_arm_usy_idx = robot.getBody(robot.findJointInd('r_arm_usy')).position_num;
  symmetric_matrix = addSymmetricPair(symmetric_matrix,1:2,l_arm_usy_idx,r_arm_usy_idx);

  l_arm_shx_idx = robot.getBody(robot.findJointInd('l_arm_shx')).position_num;
  r_arm_shx_idx = robot.getBody(robot.findJointInd('r_arm_shx')).position_num;
  symmetric_matrix = addAntiSymmetricPair(symmetric_matrix,3:4,l_arm_shx_idx,r_arm_shx_idx);

  l_arm_ely_idx = robot.getBody(robot.findJointInd('l_arm_ely')).position_num;
  r_arm_ely_idx = robot.getBody(robot.findJointInd('r_arm_ely')).position_num;
  symmetric_matrix = addSymmetricPair(symmetric_matrix,5:6,l_arm_ely_idx,r_arm_ely_idx);

  l_arm_elx_idx = robot.getBody(robot.findJointInd('l_arm_elx')).position_num;
  r_arm_elx_idx = robot.getBody(robot.findJointInd('r_arm_elx')).position_num;
  symmetric_matrix = addAntiSymmetricPair(symmetric_matrix,7:8,l_arm_elx_idx,r_arm_elx_idx);

  l_arm_uwy_idx = robot.getBody(robot.findJointInd('l_arm_uwy')).position_num;
  r_arm_uwy_idx = robot.getBody(robot.findJointInd('r_arm_uwy')).position_num;
  symmetric_matrix = addSymmetricPair(symmetric_matrix,9:10,l_arm_uwy_idx,r_arm_uwy_idx);

  l_arm_mwx_idx = robot.getBody(robot.findJointInd('l_arm_mwx')).position_num;
  r_arm_mwx_idx = robot.getBody(robot.findJointInd('r_arm_mwx')).position_num;
  symmetric_matrix = addAntiSymmetricPair(symmetric_matrix,11:12,l_arm_mwx_idx,r_arm_mwx_idx);

  l_leg_hpz_idx = robot.getBody(robot.findJointInd('l_leg_hpz')).position_num;
  r_leg_hpz_idx = robot.getBody(robot.findJointInd('r_leg_hpz')).position_num;
  symmetric_matrix = addAntiSymmetricPair(symmetric_matrix,13:14,l_leg_hpz_idx,r_leg_hpz_idx);

  l_leg_hpx_idx = robot.getBody(robot.findJointInd('l_leg_hpx')).position_num;
  r_leg_hpx_idx = robot.getBody(robot.findJointInd('r_leg_hpx')).position_num;
  symmetric_matrix = addAntiSymmetricPair(symmetric_matrix,15:16,l_leg_hpx_idx,r_leg_hpx_idx);

  l_leg_hpy_idx = robot.getBody(robot.findJointInd('l_leg_hpy')).position_num;
  r_leg_hpy_idx = robot.getBody(robot.findJointInd('r_leg_hpy')).position_num;
  symmetric_matrix = addSymmetricPair(symmetric_matrix,17:18,l_leg_hpy_idx,r_leg_hpy_idx);

  l_leg_kny_idx = robot.getBody(robot.findJointInd('l_leg_kny')).position_num;
  r_leg_kny_idx = robot.getBody(robot.findJointInd('r_leg_kny')).position_num;
  symmetric_matrix = addSymmetricPair(symmetric_matrix,19:20,l_leg_kny_idx,r_leg_kny_idx);

  l_leg_akx_idx = robot.getBody(robot.findJointInd('l_leg_akx')).position_num;
  r_leg_akx_idx = robot.getBody(robot.findJointInd('r_leg_akx')).position_num;
  symmetric_matrix = addAntiSymmetricPair(symmetric_matrix,21:22,l_leg_akx_idx,r_leg_akx_idx);

  l_leg_aky_idx = robot.getBody(robot.findJointInd('l_leg_aky')).position_num;
  r_leg_aky_idx = robot.getBody(robot.findJointInd('r_leg_aky')).position_num;
  symmetric_matrix = addSymmetricPair(symmetric_matrix,23:24,l_leg_aky_idx,r_leg_aky_idx);

  %base_y = findJointIndices(robot,'base_y'); base_y = base_y(1);
  base_y = 2;
  equal_matrix = addOpposite(equal_matrix,1,base_y);

  base_z = 3;
  equal_matrix = addEquality(equal_matrix,2,base_z);

  base_roll = 4;
  equal_matrix = addOpposite(equal_matrix,3,base_roll);

  base_pitch = 5;
  equal_matrix = addEquality(equal_matrix,4,base_pitch);

  base_yaw = 6;
  equal_matrix = addOpposite(equal_matrix,5,base_yaw);

  back_bkz = robot.getBody(robot.findJointInd('back_bkz')).position_num;
  equal_matrix = addOpposite(equal_matrix,6,back_bkz);

  back_bky = robot.getBody(robot.findJointInd('back_bky')).position_num;
  equal_matrix = addEquality(equal_matrix,7,back_bky);

  back_bkx = robot.getBody(robot.findJointInd('back_bkx')).position_num;
  equal_matrix = addOpposite(equal_matrix,8,back_bkx);

  lb = zeros(2*num_symmetry+num_equal,1);
  ub = lb;
  half_periodic_constraint = LinearConstraint(lb,ub,[symmetric_matrix;equal_matrix]);
end

function displayCallback(publisher,in_stance,N,x)
  h = x(1:N-1);
  ts = [0;cumsum(h)];
  com_z = x(N-1+(1:N));
  comdot_x = x(2*N-1+(1:N));
  q = reshape(x(3*N:end),[],N);
  x_data = [zeros(2,numel(ts));q;0*q];
  utime = now() * 24 * 60 * 60;
  snopt_info_vector = ones(1, size(x_data,2));
  publisher.publish(x_data, ts, utime, snopt_info_vector);
  sfigure(7); 
  subplot(2,1,1);
  plot(ts,com_z,'bo-'); 
  hold on
  plot(ts(in_stance),com_z(in_stance),'ro-'); 
  title('COM Height')
  xlabel('t (s)')
  ylabel('z (m)')
  hold off
  subplot(2,1,2);
  plot(ts,comdot_x,'bo-'); 
  hold on
  plot(ts(in_stance),comdot_x(in_stance),'ro-'); 
  title('COM velocity')
  xlabel('t (s)')
  ylabel('xdot (m/s)')
  hold off
  drawnow;
end
%function displayCallback(in_stance,N,x)
  %h = x(1:N-1);
  %ts = [0;cumsum(h)];
  %com_z = x(N-1+(1:N));
  %comdot_x = x(2*N-1+(1:N));
  %q = reshape(x(3*N:end),[],N);
  %x_data = [zeros(2,numel(ts));q;0*q];
  %utime = now() * 24 * 60 * 60;
  %snopt_info_vector = ones(1, size(x_data,2));
  %sfigure(7);
  %subplot(2,1,1);
  %plot(ts,com_z,'bo-');
  %hold on
  %plot(ts(in_stance),com_z(in_stance),'ro-');
  %title('COM Height')
  %xlabel('t (s)')
  %ylabel('z (m)')
  %hold off
  %subplot(2,1,2);
  %plot(ts,comdot_x,'bo-');
  %hold on
  %plot(ts(in_stance),comdot_x(in_stance),'ro-');
  %title('COM velocity')
  %xlabel('t (s)')
  %ylabel('xdot (m/s)')
  %hold off
  %drawnow;
%end

function robot = addObstacle(robot,obstacle_x_position)
  radius = 0.1;
  len = 5;
  height = 1;
  beam = RigidBodyCapsule(radius,len,[obstacle_x_position,-1,height],[4*pi/6;0;0]);
  wall1 = RigidBodyBox(2*[0.1; 1.235; 1.0],[obstacle_x_position-0.2,-0.565-1,1],[0;0;0]);
  wall2 = RigidBodyBox(2*[0.1; 0.15; 1.0],[obstacle_x_position-0.2,1.65-1,1],[0;0;0]);
  wall3 = RigidBodyBox(2*[0.1; 1.8; 0.25],[obstacle_x_position-0.2,0-1,2.25],[0;0;0]);
  robot = robot.addShapeToBody('world',beam);
  robot = robot.addShapeToBody('world',wall1);
  robot = robot.addShapeToBody('world',wall2);
  robot = robot.addShapeToBody('world',wall3);
  robot = compile(robot);
end

function options = defaultOptionsStruct()
  %options.major_feasibility_tolerance = 1e-5;
  %options.major_optimality = 1e-5;
  options.visualize = true;
  options.toe_first = false;
  options.n_interp_points = 0;
  options.speed = 2;
  options.constrain_head_gaze = true;
  options.head_gaze_tol = 15*pi/180;
  options.freeze_neck = true;
  options.start_from_standing = false;
  options.stride_filename = 'results_ss_1p5m_2mps_22knots_A.mat';
  options.enable_collision = true;
  options.enable_velocity_limits = false;
  options.add_obstacle = false;
  options.min_distance = 0.03;
  options.N = 16;
  options.time_option = 2;
  options.joint_tol = 10*pi/180;
  options.com_tol = 0.1;
  options.penalize_approx_torque = false;
  options.constrain_approx_torque = false;
  options.constrain_torque = false;
  options.penalize_torque = false;
  options.kTorque = 1;
  options.fix_q = false;
  options.max_lambda = 1;
end

function options = parseOptionsStruct(options_in)
  options = defaultOptionsStruct();
  for fieldname_cell = fields(options_in)'
    fieldname = fieldname_cell{1};
    if isfield(options,fieldname)
      options.(fieldname) = options_in.(fieldname);
    end
  end
end
