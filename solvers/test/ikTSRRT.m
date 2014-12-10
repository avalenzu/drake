function [q_traj,info,v,V,parent] = ikTSRRT(goal_bias,options,rng_seed)
if nargin < 3, rng; else, rng(rng_seed); end
if nargin < 1 || isempty(goal_bias), goal_bias = 0.05; end
w = warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
options.floating = true;
options.terrain = RigidBodyFlatTerrain();
if ~isfield(options,'n_smoothing_passes'), options.n_smoothing_passes = 1; end;
urdf = fullfile(getDrakePath(),'examples','Atlas','urdf','atlas_convex_hull.urdf');
r = RigidBodyManipulator(urdf,options);
nq = r.getNumPositions();
S = load([getDrakePath(), '/examples/Atlas/data/atlas_fp.mat']);
q_nom = S.xstar(1:nq);
q_zero = zeros(nq, 1);
[jlmin,jlmax] = r.getJointLimits();
jlmin(1:2) = q_nom(1:2) - 0.2;
jlmax(1:2) = q_nom(1:2) + 0.2;
jlmin(3) = q_nom(3) - 0.4;
jlmax(3) = q_nom(3) + 0.0;
jlmin(4:6) = q_nom(4:6) - 10*pi/180;
jlmax(4:6) = q_nom(4:6) + 10*pi/180;
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
collision_object = RigidBodyCapsule(0.05,1,[0;0;0],[0,pi/2,0]);
collision_object.c = [0.5;0.4;0.3];
r_world = addGeometryToBody(r_world, 2, collision_object);

r = r.compile();
r_world = r_world.compile();
warning(w);

reach_start = [0.0;0.0;0.6997733529873572;0.0;0.0;0.0;-0.34500497579574585;0.0872664600610733;-0.05188410356640816;-1.5616828203201294;-1.2109966278076172;0.037477169185876846;0.1360016167163849;2.779356002807617;-0.00010280924470862374;0.06855140626430511;-0.8285990953445435;1.6747099161148071;-0.8452405333518982;-0.0735296830534935;0.5894911289215088;0.27000001072883606;1.3300000429153442;2.0999999046325684;-0.5;0.0;9.328441228717566e-05;-0.06855178624391556;-0.8285998702049255;1.6747100353240967;-0.8453657627105713;0.07352013885974884;0.0;3.9785470562492264e-07];

v = r.constructVisualizer();
v.draw(0,reach_start);

var_names = [strcat({'x','y','z'},'_hand')'; strcat('w',num2cellStr(1:4))';r.getPositionFrame().coordinates];
problem = MotionPlanningProblem(7+nq,var_names);

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

% fixed right arm
posture_constraint_7 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.r_arm_usy; joints.r_arm_shx; joints.r_arm_ely; joints.r_arm_elx; joints.r_arm_uwy; joints.r_arm_mwx; joints.neck_ay];
joints_lower_limit = reach_start(joint_inds);
joints_upper_limit = reach_start(joint_inds);
posture_constraint_7 = posture_constraint_7.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);
jlmin(joint_inds) = joints_lower_limit;
jlmax(joint_inds) = joints_upper_limit;

posture_constraint_8 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.l_leg_kny;joints.r_leg_kny];
joints_lower_limit = 30*pi/180*[1;1];
joints_upper_limit = 120*pi/180*[1;1];
posture_constraint_8 = posture_constraint_8.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);

point_in_link_frame = [0.35; 0.24449999999999988; 0.011200000000000071];
%point_in_link_frame = [0; 0.24449999999999988; 0.011200000000000071];
ref_frame = [0.10040853387866658, 0.30507204666777654, 0.94702121025152886, 0.19671872655867628; -0.070421541493923046, 0.95162340926777023, -0.29908810311880585, 1.0145817508809061; -0.9924509725008871, -0.036659695518642732, 0.11703475512224609, 0.9; 0.0, 0.0, 0.0, 1.0];
lower_bounds = [0.0; 0.0; 0.0] + [-0; -0; -0];
upper_bounds = [0.0; 0.0; 0.0] + [0.0; 0.0; 0.0];
%position_constraint_7 = WorldPositionInFrameConstraint(r, l_hand, point_in_link_frame, ref_frame, lower_bounds, upper_bounds, [1.0, 1.0]);
ref_frame = inv(ref_frame);
position_constraint_7 = WorldPositionConstraint(r, l_hand, ref_frame(1:3,:)*[point_in_link_frame;1], lower_bounds, upper_bounds, [1.0, 1.0]);


quat_constraint_8 = WorldQuatConstraint(r, l_hand, [0.73638758447380859; 0.089093166809596377; 0.6584413641826542; -0.1274782451791375], 10*pi/180, [1.0, 1.0]);

min_distance_world = 0.03;
min_distance = 0.01;
active_collision_options.body_idx = setdiff(1:r.getNumBodies(),[l_foot,r_foot]);
collision_constraint = MinDistanceConstraint(r, min_distance);
collision_constraint_world = drakeFunction.kinematic.SmoothDistancePenalty(r_world,min_distance_world);
collision_constraint_2 = MinDistanceConstraint(r, min_distance,active_collision_options);
collision_constraint_2 = collision_constraint_2.generateConstraint();
problem = problem.addConstraint(collision_constraint_2{1},7+(1:nq));

base_constraints = {qsc_constraint_0, position_constraint_1, quat_constraint_2, position_constraint_3, quat_constraint_4, posture_constraint_5,posture_constraint_6,posture_constraint_7,posture_constraint_8};
active_constraints = [base_constraints, {position_constraint_7,quat_constraint_8}];

ik_seed_pose = q_nom;
ik_nominal_pose = q_nom;
cost = Point(r.getPositionFrame(),10);
for i = r.getNumBodies():-1:1
  if all(r.getBody(i).parent > 0) && all(r.getBody(r.getBody(i).parent).position_num > 0)
    cost(r.getBody(r.getBody(i).parent).position_num) = ...
      cost(r.getBody(r.getBody(i).parent).position_num) + cost(r.getBody(i).position_num);
  end
end
cost = cost/min(cost);
% cost = cost.^2;
Q = diag(cost);
% Q = eye(nq);
ikoptions = IKoptions(r);
ikoptions = ikoptions.setMajorIterationsLimit(500);
ikoptions = ikoptions.setQ(Q);
ikoptions = ikoptions.setMajorOptimalityTolerance(1e-3);
% ikoptions = ikoptions.setMajorFeasibilityTolerance(1e-3);


[q_end, info, infeasible_constraint] = inverseKin(r, ik_seed_pose, ik_nominal_pose, active_constraints{:}, ikoptions);

v.draw(0,q_end);
kinsol = r.doKinematics(reach_start);
xyz_quat_start = r.forwardKin(kinsol,l_hand,point_in_link_frame,2);
kinsol = r.doKinematics(q_end);
xyz_quat_goal = r.forwardKin(kinsol,l_hand,point_in_link_frame,2);

% v_world = r_world.constructVisualizer();
position_constraint_7 = WorldPositionConstraint(r, l_hand, point_in_link_frame, xyz_quat_start(1:3), xyz_quat_start(1:3), [1.0, 1.0]);

quat_constraint_8 = WorldQuatConstraint(r, l_hand, xyz_quat_start(4:7), 0, [1.0, 1.0]);

active_constraints = [base_constraints,{position_constraint_7,quat_constraint_8}];

[q_start, info, infeasible_constraint] = inverseKin(r, ik_seed_pose, ik_nominal_pose, active_constraints{:}, ikoptions);

x_start = [xyz_quat_start;q_start];
x_goal = [xyz_quat_goal;q_end];
xyz_box_edge_length = 2;
xyz_min = min(xyz_quat_start(1:3),xyz_quat_goal(1:3)) - xyz_box_edge_length/2;
xyz_max = max(xyz_quat_start(1:3),xyz_quat_goal(1:3)) + xyz_box_edge_length/2;

orientation_weight = max(abs(xyz_max-xyz_min));%xyz_box_edge_length;
posture_weight = 1e0;

options.distance_metric_fcn = @(q1,q2) poseDistance(q1(1:7,:),q2(1:7,:),orientation_weight);
% options.max_length_between_constraint_checks = 0.1;
options.max_edge_length = 0.1;
options.interpolation_fcn = @ikInterpolation;
options.display_after_every = 10;
options.display_fcn = @displayFun;
options.goal_bias = goal_bias;

% n_ee_poses_tried = 1;
%sample_prog = InverseKinematics(r,ik_nominal_pose,base_constraints{:},collision_constraint);
% sample_prog = InverseKinematics(r,ik_nominal_pose,base_constraints{:});
% sample_prog = sample_prog.setQ(0.1*ikoptions.Q);
% sample_prog = sample_prog.setSolverOptions('snopt','MajorIterationsLimit',ikoptions.SNOPT_IterationsLimit);
% sample_prog.setSolverOptions('snopt','MajorFeasibilityTolerance',ikoptions.SNOPT_MajorFeasibilityTolerance);
% sample_prog.setSolverOptions('snopt','MajorOptimalityTolerance',1e-3);
rrt_time = tic;
[xtraj,info,V,parent] = problem.rrt(x_start,x_goal,@sampleFunction,options);
display('RRT time:')
toc(rrt_time);

%if info == 1
smoothing_time = tic;
x_data = xtraj.eval(xtraj.getBreaks());
x_data_smoothed = x_data;
for i = 1:options.n_smoothing_passes
  x_data_smoothed = smoothPath(x_data_smoothed);
end
dist = zeros(1,size(x_data_smoothed,2)-1);
t = zeros(1,size(x_data_smoothed,2)-1);
for i = 1:numel(t)
  dist(i) = options.distance_metric_fcn(x_data_smoothed(:,i),x_data_smoothed(:,i+1));
end
velocity = 1;%-diff(0.5*cos(linspace(0,pi,numel(dist)+1)));
t = 5*cumsum(dist./velocity)/sum(dist./velocity);
display('Smoothing time:')
toc(smoothing_time);
hold on;
plot(x_data_smoothed(1,:),x_data_smoothed(2,:),'ro-');
hold off;
% q_traj = PPTrajectory(foh(linspace(0,1,numel(xtraj.getBreaks())),x_data(8:end,:)));
q_traj = PPTrajectory(foh([0,t],x_data_smoothed(8:end,:)));
q_traj = q_traj.setOutputFrame(r.getPositionFrame());
%else
%q_traj = []
%end
  function displayFun(V,parent,last_drawn_edge_num)
    v.draw(0,V(8:end,end));
%     v_world.draw(0,[V(1:3,end);quat2rpy(V(4:7,end))]);
    MotionPlanningProblem.drawFirstTwoCoordinates(V,parent,last_drawn_edge_num);
  end

  function [x_interp,valid_interp] = ikInterpolation(x1,x2,alpha,tol,ang_tol)
    if nargin < 4, tol = 0.01; end
    if nargin < 5, ang_tol = 10*pi/180; end
    x_interp = [];
    xyz = (1-alpha)*x1(1:3)+alpha*x2(1:3);
    quat = (1-alpha)*x1(4:7)+alpha*x2(4:7); %yes, I know this isn't the right way to do this
    quat = quat/norm(quat);
    q_nom_local = x1(8:end);
    
    if eval(collision_constraint_world,[xyz;quat2rpy(quat)]) == 0
      position_constraint_7 = Point2PointDistanceConstraint(r, l_hand,1,point_in_link_frame, xyz, 0, tol, [1.0, 1.0]);
      
      quat_constraint_8 = WorldQuatConstraint(r, l_hand, quat, ang_tol, [1.0, 1.0]);
      
      [q, info, infeasible_constraint] = inverseKin(r, q_nom, q_nom, base_constraints{:}, position_constraint_7,quat_constraint_8,ikoptions);
      
      valid_interp = (info < 10);
      if valid_interp
        kinsol = r.doKinematics(q);
        xyz_quat = r.forwardKin(kinsol,l_hand,point_in_link_frame,2);
        x_interp = [xyz_quat; q];
%       else
%         display(info);
      end
    else
      valid_interp = false;
    end
    %   if valid_interp% && alpha > 0.5
    % %     display('Valid interpolation');
    %     v.draw(0,q);
    % %   else
    % %     display(info);
    %   end
  end

  function x_data_smoothed = smoothPath(x_data)
    if size(x_data,2) <= 2;
      x_data_smoothed = x_data;
    else
      x0 = x_data(:,1);
      xf = x_data(:,end);
      valid_shortcut = true;
      num_interpolated_checks = ...
        ceil(options.distance_metric_fcn(x0,xf)/(1*options.max_edge_length));
      x_data_smoothed = zeros(size(x_data,1),num_interpolated_checks);
      alpha = linspace(0,1,num_interpolated_checks);
      for ii = 1:num_interpolated_checks
        [x_interp,valid_shortcut] = ikInterpolation(x0,xf,alpha(ii),0,0);
        valid_shortcut = valid_shortcut && problem.checkConstraints(x_interp);
        if valid_shortcut
          x_data_smoothed(:,ii) = x_interp;
        else
          break; 
        end
      end
      if ~valid_shortcut
        mid_index = ceil(size(x_data,2)/2);
        x_data_smoothed_1 = smoothPath(x_data(:,1:mid_index));
        x_data_smoothed_2 = smoothPath(x_data(:,mid_index:end));
        x_data_smoothed = [x_data_smoothed_1(:,1:end-1),x_data_smoothed_2];
      end
    end
  end
  function x_sample = sampleFunction()
    xyz = (xyz_max-xyz_min).*rand(3,1)+xyz_min;
    quat = uniformlyRandomQuat();
%     have_good_quat = false;
%     while ~have_good_quat
%       quat = uniformlyRandomQuat();
%       have_good_quat = quaternionDistance(quat,xyz_quat_start(4:7)) < 0.2;
%     end
    x_sample = [xyz; quat; NaN(nq,1)];
  end
end

function dist = poseDistance(xyz_quat1,xyz_quat2,orientation_weight)
xyz1 = xyz_quat1(1:3,:);
xyz2 = xyz_quat2(1:3,:);
xyz_dist = sum(sqrt(bsxfun(@minus,xyz1,xyz2).^2),1);
quat_dist = quaternionDistance(xyz_quat1(4:7,:),xyz_quat2(4:7,:));
dist = xyz_dist + orientation_weight*quat_dist;
end
