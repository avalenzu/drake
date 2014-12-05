function [q_traj,info,v,V,parent] = ikRRT(goal_bias,options)
if nargin < 1 || isempty(goal_bias), goal_bias = 0.05; end
w = warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
options.floating = true;
urdf = fullfile(getDrakePath(),'examples','Atlas','urdf','atlas_convex_hull.urdf');
r = RigidBodyManipulator(urdf,options);
nq = r.getNumPositions();
S = load([getDrakePath(), '/examples/Atlas/data/atlas_fp.mat']);
q_nom = S.xstar(1:nq);
q_zero = zeros(nq, 1);
r_world = RigidBodyManipulator(fullfile(getDrakePath(),'systems','plants','test','FallingBrick.urdf'),options);
r_world = r_world.removeCollisionGroupsExcept({});

% Create short name for R^3
R3 = drakeFunction.frames.realCoordinateSpace(3);

world = r.findLinkInd('world');
l_foot = r.findLinkInd('l_foot');
r_foot = r.findLinkInd('r_foot');
l_hand = r.findLinkInd('l_hand');
r_hand = r.findLinkInd('r_hand');
utorso = r.findLinkInd('utorso');
pelvis = r.findLinkInd('pelvis');
l_foot_pts = r.getBody(l_foot).getTerrainContactPoints();
r_foot_pts = r.getBody(r_foot).getTerrainContactPoints();
joints = Point(r.getStateFrame, (1:r.getStateFrame.dim)');

% Add obstacles
collision_object = RigidBodyCapsule(0.05,1,[0.95,0.22,0.35],[0,pi/2,0]);
collision_object.c = [0.5;0.4;0.3];
r = addGeometryToBody(r, world, collision_object);
r_world = addGeometryToBody(r_world, world, collision_object);

collision_object = RigidBodyCapsule(0.05,1,[0.95,-0.22,0.35],[0,pi/2,0]);
collision_object.c = [0.5;0.4;0.3];
r = addGeometryToBody(r, world, collision_object);
r_world = addGeometryToBody(r_world, world, collision_object);

collision_object = RigidBodyCapsule(0.05,1,[0.8,-0.05,0.35],[-pi/4,0,0]);
collision_object.c = [0.5;0.4;0.3];
r = addGeometryToBody(r, world, collision_object);
r_world = addGeometryToBody(r_world, world, collision_object);

collision_object = RigidBodyCapsule(0.05,1,[0.45,-0.05,0.35],[-pi/4,0,0]);
collision_object.c = [0.5;0.4;0.3];
r = addGeometryToBody(r, world, collision_object);
r_world = addGeometryToBody(r_world, world, collision_object);

collision_object = RigidBodyCapsule(0.05,1,[0.35,0.25,0],[0,pi/2,0]);
collision_object.c = [0.5;0.4;0.3];
r = addGeometryToBody(r, l_hand, collision_object);
r_world = addGeometryToBody(r_world, 2, collision_object);

r = r.compile();
r_world = r_world.compile();
warning(w);

reach_start = [0.0;0.0;0.6997733529873572;0.0;0.0;0.0;-0.34500497579574585;0.0872664600610733;-0.05188410356640816;-1.5616828203201294;-1.2109966278076172;0.037477169185876846;0.1360016167163849;2.779356002807617;-0.00010280924470862374;0.06855140626430511;-0.8285990953445435;1.6747099161148071;-0.8452405333518982;-0.0735296830534935;0.5894911289215088;0.27000001072883606;1.3300000429153442;2.0999999046325684;-0.5;0.0;9.328441228717566e-05;-0.06855178624391556;-0.8285998702049255;1.6747100353240967;-0.8453657627105713;0.07352013885974884;0.0;3.9785470562492264e-07];

v = r.constructVisualizer();
v.draw(0,reach_start);

var_names = [strcat({'x','y','z'},'_hand')'; strcat('w',num2cellStr(1:4))';r.getPositionFrame().coordinates];
problem = MotionPlanningProblem(nq+7,var_names);

% IK constraints
qsc_constraint_0 = QuasiStaticConstraint(r, [-inf, inf], 1);
qsc_constraint_0 = qsc_constraint_0.setShrinkFactor(0.2);
qsc_constraint_0 = qsc_constraint_0.setActive(true);
qsc_constraint_0 = qsc_constraint_0.addContact(l_foot, l_foot_pts);
qsc_constraint_0 = qsc_constraint_0.addContact(r_foot, r_foot_pts);

point_in_link_frame = [0.0; 0.0; 0.0];
ref_frame = [0.99999962214379723, 3.8873668451910772e-05, 0.00086844752325226373, -0.024113362129690341; -4.319650228383918e-05, 0.99998760778828055, 0.0049781928381826216, 0.13142880655433892; -0.00086824324064880729, -0.0049782284710370005, 0.99998723161596681, 0.081845132612297311; 0.0, 0.0, 0.0, 1.0];
lower_bounds = [0.0; 0.0; 0.0] + [0.0; 0.0; 0.0];
upper_bounds = [0.0; 0.0; 0.0] + [0.0; 0.0; 0.0];
ref_frame = inv(ref_frame);
position_constraint_1 = WorldPositionConstraint(r, l_foot, ref_frame(1:3,:)*[point_in_link_frame;1], lower_bounds, upper_bounds, [0.0, 1.0]);


quat_constraint_2 = WorldQuatConstraint(r, l_foot, [0.99999680768841015; -0.0024891132733300568; 0.00043417407699420605; -2.0517608182535892e-05], 0.0, [0.0, 1.0]);


point_in_link_frame = [0.0; 0.0; 0.0];
ref_frame = [0.99999972333813658, -3.8603987442147522e-05, 0.00074285488657430923, -0.024113358389590833; 4.2294235092508014e-05, 0.99998765711726534, -0.0049682818277853539, -0.13142881299268941; -0.00074265392211426647, 0.0049683118717304582, 0.99998738209154281, 0.081845129013906948; 0.0, 0.0, 0.0, 1.0];
lower_bounds = [0.0; 0.0; 0.0] + [0.0; 0.0; 0.0];
upper_bounds = [0.0; 0.0; 0.0] + [0.0; 0.0; 0.0];
ref_frame = inv(ref_frame);
position_constraint_3 = WorldPositionConstraint(r, r_foot, ref_frame(1:3,:)*[point_in_link_frame;1], lower_bounds, upper_bounds, [0.0, 1.0]);


quat_constraint_4 = WorldQuatConstraint(r, r_foot, [0.99999684531339206; 0.0024841562616134435; 0.00037137837375452614; 2.0224619435999976e-05], 0.0, [0.0, 1.0]);

posture_constraint_5 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.back_bkx; joints.back_bky; joints.back_bkz];
joints_lower_limit = q_zero(joint_inds) + [-0.08726646259971647; -0.08726646259971647; -inf];
joints_upper_limit = q_zero(joint_inds) + [0.08726646259971647; 0.08726646259971647; inf];
posture_constraint_5 = posture_constraint_5.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);

posture_constraint_6 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.base_x; joints.base_y; joints.base_roll; joints.base_pitch; joints.base_yaw];
joints_lower_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0];
posture_constraint_6 = posture_constraint_6.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);

point_in_link_frame = [0.35; 0.24449999999999988; 0.011200000000000071];
%point_in_link_frame = [0; 0.24449999999999988; 0.011200000000000071];
ref_frame = [0.10040853387866658, 0.30507204666777654, 0.94702121025152886, 0.19671872655867628; -0.070421541493923046, 0.95162340926777023, -0.29908810311880585, 1.0145817508809061; -0.9924509725008871, -0.036659695518642732, 0.11703475512224609, 0.9; 0.0, 0.0, 0.0, 1.0];
lower_bounds = [0.0; 0.0; 0.0] + [-0; -0; -0];
upper_bounds = [0.0; 0.0; 0.0] + [0.0; 0.0; 0.0];
%position_constraint_7 = WorldPositionInFrameConstraint(r, l_hand, point_in_link_frame, ref_frame, lower_bounds, upper_bounds, [1.0, 1.0]);
ref_frame = inv(ref_frame);
position_constraint_7 = WorldPositionConstraint(r, l_hand, ref_frame(1:3,:)*[point_in_link_frame;1], lower_bounds, upper_bounds, [1.0, 1.0]);


quat_constraint_8 = WorldQuatConstraint(r, l_hand, [0.73638758447380859; 0.089093166809596377; 0.6584413641826542; -0.1274782451791375], 10*pi/180, [1.0, 1.0]);

min_distance = 0.02;
min_distance_2 = 0.02;
collision_constraint = MinDistanceConstraint(r, min_distance);
collision_constraint_world = drakeFunction.kinematic.SmoothDistancePenalty(r_world,min_distance);
collision_constraint_2 = MinDistanceConstraint(r, min_distance_2);
collision_constraint_2 = collision_constraint_2.generateConstraint();
problem = problem.addConstraint(collision_constraint_2{1},8:nq+7)

base_constraints = {qsc_constraint_0, position_constraint_1, quat_constraint_2, position_constraint_3, quat_constraint_4, posture_constraint_5,posture_constraint_6};
active_constraints = [base_constraints, {position_constraint_7,quat_constraint_8}];

ik_seed_pose = q_nom;
ik_nominal_pose = q_nom;
ikoptions = IKoptions(r);
ikoptions = ikoptions.setMajorIterationsLimit(10);


[q_end, info, infeasible_constraint] = inverseKin(r, ik_seed_pose, ik_nominal_pose, active_constraints{:}, ikoptions);

orientation_weight = 2e0;
posture_weight = 1e0;

options.distance_metric_fcn = @(q1,q2) poseDistance(q1(1:7,:),q2(1:7,:),orientation_weight);% + posture_weight*sqrt(sum(bsxfun(@minus,q1(8:end,:),q2(8:end,:)).^2,1));
options.max_length_between_constraint_checks = 0.05;
%options.max_edge_length = 0.1;
options.interpolation_fcn = @ikInterpolation;
options.display_after_every = 1;
options.goal_bias = goal_bias;

v.draw(0,q_end);
kinsol = r.doKinematics(reach_start);
xyz_quat_start = r.forwardKin(kinsol,l_hand,point_in_link_frame,2);
kinsol = r.doKinematics(q_end);
xyz_quat_goal = r.forwardKin(kinsol,l_hand,point_in_link_frame,2);

position_constraint_7 = WorldPositionConstraint(r, l_hand, point_in_link_frame, xyz_quat_start(1:3), xyz_quat_start(1:3), [1.0, 1.0]);

quat_constraint_8 = WorldQuatConstraint(r, l_hand, xyz_quat_start(4:7), 0, [1.0, 1.0]);

active_constraints = [base_constraints,{position_constraint_7,quat_constraint_8}];

[q_start, info, infeasible_constraint] = inverseKin(r, ik_seed_pose, ik_nominal_pose, active_constraints{:}, ikoptions);

x_start = [xyz_quat_start;q_start];
x_goal = [xyz_quat_goal;q_end];
xyz_min = xyz_quat_start(1:3) - 1;
xyz_max = xyz_quat_start(1:3) + 1;
n_ee_poses_tried = 1;
%sample_prog = InverseKinematics(r,ik_nominal_pose,base_constraints{:},collision_constraint);
sample_prog = InverseKinematics(r,ik_nominal_pose,base_constraints{:});
sample_prog = sample_prog.setQ(0.1*ikoptions.Q);
sample_prog = sample_prog.setSolverOptions('snopt','MajorIterationsLimit',ikoptions.SNOPT_IterationsLimit);
sample_prog.setSolverOptions('snopt','MajorFeasibilityTolerance',ikoptions.SNOPT_MajorFeasibilityTolerance);
sample_prog.setSolverOptions('snopt','MajorOptimalityTolerance',1e-3);
[xtraj,info,V,parent] = problem.rrt(x_start,x_goal,@ikRandomSample,options);

%if info == 1
x_data = xtraj.eval(xtraj.getBreaks());
q_data = x_data(8:end,:);
q_traj = PPTrajectory(foh(linspace(0,1,numel(xtraj.getBreaks())),q_data));
q_traj = q_traj.setOutputFrame(r.getPositionFrame());
%else
  %q_traj = []
%end


function x = ikRandomSample()
  have_good_sample = false;
  while (~have_good_sample)
    %fprintf('Generating posture for pose no. %d ...\n',n_ee_poses_tried);
    have_good_quat = false;
    while ~have_good_quat
      xyz = (xyz_max-xyz_min).*rand(3,1)+xyz_min;
      quat = uniformlyRandomQuat();
      have_good_quat = (0 == eval(collision_constraint_world,[xyz;quat2rpy(quat)]));
      have_good_quat = have_good_quat && (quaternionDistance(quat,xyz_quat_start(4:7)) < 0.2);
      %have_good_quat = true;
    end

    tol = 1e-1;
    position_constraint_7 = Point2PointDistanceConstraint(r, l_hand,1,point_in_link_frame, xyz, 0, tol, [1.0, 1.0]);

    quat_constraint_8 = WorldQuatConstraint(r, l_hand, quat, 30*pi/180, [1.0, 1.0]);
    %active_constraints = [base_constraints,{position_constraint_7,quat_constraint_8,collision_constraint}];
    quat_objective = quat_constraint_8.generateObjective();
    xyz_constraint = position_constraint_7.generateConstraint();
    prog = sample_prog.addCost(xyz_constraint{1},sample_prog.q_idx,sample_prog.kinsol_dataind);
    prog = prog.addCost(quat_objective{1},prog.q_idx,prog.kinsol_dataind);
    %[q, info, infeasible_constraint] = inverseKin(r, ik_seed_pose, ik_nominal_pose, active_constraints{:}, ikoptions);
    [q, info, infeasible_constraint] = prog.solve(ik_seed_pose);
    if info < 10, 
      have_good_sample = true; 
      display('Found posture')
    else
      fprintf('Failed to find posture (%d)\n',info)
    end
    n_ee_poses_tried = n_ee_poses_tried + 1;
  end

  x = [xyz; quat; q];
end

function [x_interp,valid_interp] = ikInterpolation(x1,x2,alpha)
  xyz = (1-alpha)*x1(1:3)+alpha*x2(1:3);
  quat = (1-alpha)*x1(4:7)+alpha*x2(4:7); %yes, I know this isn't the right way to do this
  %q_nom = (1-alpha)*x1(8:end)+alpha*x2(8:end);
  q_nom = x2(8:end);
  quat = quat/norm(quat);
  %quat = x1(4:7);
  if abs(quat) < 1e-6
    quat = quat1;
  else
    quat = quat/norm(quat);
  end

  position_constraint_7 = WorldPositionConstraint(r, l_hand, point_in_link_frame, xyz, xyz, [1.0, 1.0]);

  quat_constraint_8 = WorldQuatConstraint(r, l_hand, quat, 0, [1.0, 1.0]);

  %active_constraints = [base_constraints,{position_constraint_7,quat_constraint_8,collision_constraint}];
  active_constraints = [base_constraints,{position_constraint_7,quat_constraint_8}];
  [q, info, infeasible_constraint] = inverseKin(r, q_nom, q_nom, active_constraints{:}, ikoptions);

  x_interp = [xyz; quat; q];
  
  valid_interp = (info < 10);
  if valid_interp
    display('Valid interpolation');
    v.draw(0,q);
  else
    display(info);
  end
end
end

function dist = poseDistance(xyz_quat1,xyz_quat2,orientation_weight)
xyz1 = xyz_quat1(1:3,:);
xyz2 = xyz_quat2(1:3,:);
xyz_dist = sum(sqrt(bsxfun(@minus,xyz1,xyz2).^2),1);
quat_dist = quaternionDistance(xyz_quat1(4:7,:),xyz_quat2(4:7,:));
dist = xyz_dist + orientation_weight*quat_dist;
end
