function testSlantedSteps()
  checkDependency('lcmgl');
  snopt_output_filename = 'test_slanted_steps.out';
  if nargin < 1, x_seed = []; end;
  w = warning('off','Drake:RigidBody:SimplifiedCollisionGeometry');
  warning('off','Drake:RigidBody:NonPositiveInertiaMatrix');
  warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
  warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
  warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
  options.floating = true;
  options.terrain = RigidBodyFlatTerrain;
  atlas_urdf = [getDrakePath,'/examples/Atlas/urdf/atlas_convex_hull.urdf'];
  scene_urdf = [getDrakePath,'/solvers/trajectoryOptimization/test/SextupleStep.urdf'];
  robot = RigidBodyManipulator(atlas_urdf,options);
  robot = addRobotFromURDF(robot,scene_urdf);

  v = constructVisualizer(robot);
  lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'asdf');
  warning(w);

  nomdata = load([getDrakePath,'/examples/Atlas/data/atlas_fp.mat']);
  nq = robot.getNumPositions();
  qstar = nomdata.xstar(1:nq);
  kinsol_star = robot.doKinematics(qstar);
  com_star = robot.getCOM(kinsol_star);
  nv = robot.getNumDOF();
  vstar = zeros(nv,1);

  world = robot.findLinkInd('world');
  l_foot = robot.findLinkInd('l_foot');
  r_foot = robot.findLinkInd('r_foot');
  bky_idx = robot.getBody(robot.findJointInd('back_bky')).dofnum;

  mu = 1.16; % rubber on rubber

  n_knots_per_mode = 2;
  nT = 9*n_knots_per_mode;

  tf_range = [0.1 5];
  q_nom = bsxfun(@times,qstar,ones(1,nT));
  Q_comddot = eye(3);
  Q = eye(nq);
  Q(1,1) = 0;
  Q(2,2) = 0;
  Q(3,3) = 0;
  Q(6,6) = 0;
  Q(bky_idx,bky_idx) = 100*Q(bky_idx,bky_idx);
  Qv = Q;

  l_foot_bottom = robot.getBody(l_foot).getTerrainContactPoints();
  r_foot_bottom = robot.getBody(r_foot).getTerrainContactPoints();

  %active_collision_options.body_idx = setdiff(1:robot.getNumBodies(),[world]);
  active_collision_options.body_idx = setdiff(1:robot.getNumBodies(),[l_foot,r_foot]);
  min_distance_constraint_cell{1} = MinDistanceConstraint(robot,0.05,[-inf,inf],active_collision_options);
  active_collision_options.body_idx = [l_foot,r_foot];
  active_collision_options.collision_groups = {'default'};
  min_distance_constraint_cell{2} = MinDistanceConstraint(robot,0.02,[-inf,inf],active_collision_options);

  ground = robot.getBody(world).getContactShapes('terrain');
  first_platform =  robot.getBody(world).getContactShapes('first_step');

  supporting_faces{1} = [0;0;1];
  supporting_faces{2} = [0;-1;0];

  % Left foot on ground
  stance_knots{1} = 1:n_knots_per_mode;
  [wrench_cnstr{1},on_face_cnstr{1},stationary_cnstr{1}] = generateStanceOnBoxInWorldConstraints(robot,l_foot,l_foot_bottom,ground{1},[0,0,1],mu);

  % Right foot on ground
  stance_knots{2} = 1:2*n_knots_per_mode;
  [wrench_cnstr{2},on_face_cnstr{2},stationary_cnstr{2}] = generateStanceOnBoxInWorldConstraints(robot,r_foot,r_foot_bottom,ground{1},[0,0,1],mu);
  
  % Left foot on first platform
  stance_knots{3} = 4*n_knots_per_mode:5*n_knots_per_mode;
  [wrench_cnstr{3},on_face_cnstr{3},stationary_cnstr{3}] = generateStanceOnBoxInWorldConstraints(robot,l_foot,l_foot_bottom,first_platform{1},[0,-1,0],mu);

  % Right foot on ground
  stance_knots{4} = 7*n_knots_per_mode:9*n_knots_per_mode;
  [wrench_cnstr{4},on_face_cnstr{4},stationary_cnstr{4}] = generateStanceOnBoxInWorldConstraints(robot,r_foot,r_foot_bottom,ground{1},[0,0,1],mu);

  % Left foot on ground
  stance_knots{5} = 8*n_knots_per_mode:9*n_knots_per_mode;
  [wrench_cnstr{5},on_face_cnstr{5},stationary_cnstr{5}] = generateStanceOnBoxInWorldConstraints(robot,l_foot,l_foot_bottom,ground{1},[0,0,1],mu);

  qsc = QuasiStaticConstraint(robot);
  qsc = qsc.addContact(l_foot,l_foot_bottom,r_foot,r_foot_bottom);
  qsc = qsc.setActive(true);
  start_pose_prog = InverseKin(robot,qstar,on_face_cnstr{1},on_face_cnstr{2},qsc,min_distance_constraint_cell{:});
  start_pose_prog = start_pose_prog.setSolverOptions('snopt','print',snopt_output_filename);
  [q0,F,info] = start_pose_prog.solve(qstar);
  kinsol = doKinematics(robot,q0);
  com0 = getCOM(robot,kinsol);
  v.draw(0,q0);

  q_nom = bsxfun(@times,q0,ones(1,nT));

  x0 = [q0; vstar];

  % Setup contact wrench struct
  contact_wrench_struct = struct('active_knot',{},'cw',{});
  for i = 1:numel(stance_knots)
    contact_wrench_struct(i).cw = wrench_cnstr{i};
    contact_wrench_struct(i).active_knot = stance_knots{i};
  end

  g = 9.81;
  Q_contact_force = 10/(robot.getMass*g)^2*eye(3);
  % Find initial forces
  start_force_prog = ComDynamicsFullKinematicsPlanner(robot,2,[1,1],Q_comddot,Qv,Q,q_nom(:,1:2),Q_contact_force,contact_wrench_struct);
  start_force_prog = start_force_prog.setSolverOptions('snopt','print',snopt_output_filename);

  start_force_prog = start_force_prog.addStateConstraint(ConstantConstraint(x0),{1,2});
  start_force_prog = start_force_prog.addConstraint(ConstantConstraint(repmat(com0,2,1)),start_force_prog.com_inds(:));
  for i = 1:2
    start_force_prog = start_force_prog.addCost(QuadraticConstraint(0,0,eye(numel(start_force_prog.lambda_inds{i})),zeros(numel(start_force_prog.lambda_inds{i}),1)),start_force_prog.lambda_inds{i}(:));
  end
  if isempty(x_seed)
    x_seed = zeros(start_force_prog.num_vars,1);
    x_seed(start_force_prog.h_inds) = 0.1;
    x_seed(start_force_prog.q_inds(:)) = reshape(bsxfun(@times,q0,ones(1,2)),[],1);
    x_seed(start_force_prog.com_inds(:)) = reshape(bsxfun(@times,com0,ones(1,2)),[],1);
    x_seed(start_force_prog.lambda_inds{1}(:)) = reshape(1*ones(contact_wrench_struct(1).cw.num_pt_F,4,2),[],1);
    x_seed(start_force_prog.lambda_inds{2}(:)) = reshape(1*ones(contact_wrench_struct(2).cw.num_pt_F,4,2),[],1);
  end
  [x_sol,F,info] = start_force_prog.solve(x_seed);
  lambda_sol = cell(2,1);
  lambda_sol{1} = reshape(x_sol(start_force_prog.lambda_inds{1}),size(start_force_prog.lambda_inds{1},1),[],2);
  lambda_sol{2} = reshape(x_sol(start_force_prog.lambda_inds{2}),size(start_force_prog.lambda_inds{2},1),[],2);
  lambda0{1} = lambda_sol{1}(:,:,1);
  lambda0{2} = lambda_sol{2}(:,:,1);

  % Setup contact wrench struct
  contact_wrench_struct = struct('active_knot',{},'cw',{});
  for i = 1:numel(stance_knots)
    contact_wrench_struct(i).cw = wrench_cnstr{i};
    contact_wrench_struct(i).active_knot = stance_knots{i};
  end

  cdfkp = ComDynamicsFullKinematicsPlanner(robot,nT,tf_range,Q_comddot,Qv,Q,q_nom,Q_contact_force,contact_wrench_struct);

  % Stance constraints
  for i = 1:numel(stance_knots)
    cdfkp = cdfkp.addRigidBodyConstraint(on_face_cnstr{i},num2cell(stance_knots{i}));
    cdfkp = cdfkp.addRigidBodyConstraint(stationary_cnstr{i},stance_knots(i));
  end

  % Timestep bounds
  cdfkp = cdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(0.1*ones(nT-1,1),0.5*ones(nT-1,1)),cdfkp.h_inds);
  cdfkp = cdfkp.addBoundingBoxConstraint(BoundingBoxConstraint([vstar;vstar],[vstar;vstar]),[cdfkp.v_inds(:,1);cdfkp.v_inds(:,nT)]);

  % Initial conditions
  %cdfkp = cdfkp.addConstraint(ConstantConstraint(q0([1,4:end])),cdfkp.q_inds([1,4:end],1));
  cdfkp = cdfkp.addConstraint(ConstantConstraint(q0),cdfkp.q_inds(:,1));

  % Final conditions
  cdfkp = cdfkp.addConstraint(ConstantConstraint(q0(4:end)),cdfkp.q_inds(4:end,nT));
  
  % Collision avoidance constraints
  cdfkp = cdfkp.addRigidBodyConstraint(min_distance_constraint_cell{1},num2cell(1:nT));
  %cdfkp = cdfkp.addRigidBodyConstraint(min_distance_constraint_cell{2},num2cell(1:nT));

if true
  lambda_seed{1} = repmat(lambda0{1},[1 1 nT]);
  lambda_seed{2} = repmat(lambda0{2},[1 1 nT]);
  %lambda_seed{1}(:,:,~any(l_foot_stance,1)) = 0;
  %lambda_seed{2}(:,:,~any(r_foot_stance,1)) = 0;
  x_seed = zeros(cdfkp.num_vars,1);
  x_seed(cdfkp.h_inds) = 0.1;
  x_seed(cdfkp.q_inds(:)) = reshape(bsxfun(@times,q0,ones(1,nT)),[],1);
  x_seed(cdfkp.com_inds(:)) = reshape(bsxfun(@times,com0,ones(1,nT)),[],1);
  x_seed(cdfkp.lambda_inds{1}(:)) = reshape(lambda_seed{1},[],1);
  x_seed(cdfkp.lambda_inds{2}(:)) = reshape(lambda_seed{2},[],1);
end

cdfkp = cdfkp.setSolverOptions('snopt','iterationslimit',1e6);
cdfkp = cdfkp.setSolverOptions('snopt','majoriterationslimit',50);
cdfkp = cdfkp.setSolverOptions('snopt','majorfeasibilitytolerance',1e-4);
cdfkp = cdfkp.setSolverOptions('snopt','minorfeasibilitytolerance',1e-4);
cdfkp = cdfkp.setSolverOptions('snopt','majoroptimalitytolerance',1e-3);
cdfkp = cdfkp.setSolverOptions('snopt','superbasicslimit',2000);
cdfkp = cdfkp.setSolverOptions('snopt','linesearchtolerance',0.99);
cdfkp = cdfkp.setSolverOptions('snopt','print',snopt_output_filename);

tic
%profile on;
[x_sol,F,info] = cdfkp.solve(x_seed);
%profile off;
toc
q_sol = reshape(x_sol(cdfkp.q_inds(:)),nq,nT);
v_sol = reshape(x_sol(cdfkp.v_inds(:)),nq,nT);
h_sol = reshape(x_sol(cdfkp.h_inds),1,[]);
t_sol = cumsum([0 h_sol]);
com_sol = reshape(x_sol(cdfkp.com_inds),3,[]);
comdot_sol = reshape(x_sol(cdfkp.comdot_inds),3,[]);
comddot_sol = reshape(x_sol(cdfkp.comddot_inds),3,[]);
H_sol = reshape(x_sol(cdfkp.H_inds),3,[]);
Hdot_sol = reshape(x_sol(cdfkp.Hdot_inds),3,[]);
lambda_sol = cell(2,1);
lambda_sol{1} = reshape(x_sol(cdfkp.lambda_inds{1}),size(cdfkp.lambda_inds{1},1),[],nT);
lambda_sol{2} = reshape(x_sol(cdfkp.lambda_inds{1}),size(cdfkp.lambda_inds{2},1),[],nT);
xtraj_sol = PPTrajectory(foh(cumsum([0 h_sol]),[q_sol;v_sol]));
xtraj_sol = xtraj_sol.setOutputFrame(robot.getStateFrame);
  keyboard
end

function [wrench_cnstr,on_face_cnstr,stationary_cnstr] = generateStanceOnBoxInWorldConstraints(robot,body_id,pts,box,face,mu)
  % [wrench_cnstr,cnstr] = StandOnBoxInWorldConstraint(robot,body_id,pts,box,face) returns the wrench and kinematic constraints required to have the robot stand on a box.
  body = robot.parseBodyID(body_id);
  T_frame_to_world = box.T;

  num_edges = 3;
  FC_angles = linspace(0,2*pi,num_edges+1);FC_angles(end) = [];
  FC_axis = T_frame_to_world(1:3,1:3)*[0;-1;0];
  FC_perp1 = rotx(pi/2)*FC_axis;
  FC_perp2 = cross(FC_axis,FC_perp1);
  FC_edge = bsxfun(@plus,FC_axis,mu*(bsxfun(@times,cos(FC_angles),FC_perp1) + ...
    bsxfun(@times,sin(FC_angles),FC_perp2)));
  FC_edge = robot.getMass()*norm(robot.getGravity)*bsxfun(@rdivide,FC_edge,sqrt(sum(FC_edge.^2,1)));

  wrench_cnstr = LinearFrictionConeWrench(robot,body,pts,FC_edge);
  n_pts = size(pts,2);
  pts = pts(:,1:n_pts);
  tol = 1e-3;

  lb = reshape(face,3,1); lb(face==0) = -1;
  ub = reshape(face,3,1); ub(face==0) = 1;
  lb = repmat(0.5*lb.*box.size,1,n_pts);
  ub = repmat(0.5*ub.*box.size,1,n_pts);
  on_face_cnstr = WorldPositionInFrameConstraint(robot,body,pts,T_frame_to_world,lb,ub);

  stationary_cnstr = WorldFixedBodyPoseConstraint(robot,body);

  %lb = 0.5*[NaN;NaN;1].*box.size+0.0811;
  %lb = [NaN;NaN;0-tol];
  %ub = [NaN;NaN;0+tol];
  %cnstr{2} = WorldPositionConstraint(robot,robot.parseBodyID(body_id),[0;0;0],lb,ub);
end
