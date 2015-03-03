function max_coord_scratch
import drakeFunction.kinematic.*
import drakeFunction.euclidean.*
import drakeFunction.frames.*
options = struct();
options.floating = true;
options.use_new_kinsol = true;
%urdf = fullfile(getDrakePath(), 'examples', 'Atlas', 'urdf', 'atlas_convex_hull.urdf');
urdf = fullfile(getenv('DRC_BASE'), 'software', 'models', 'atlas_v4', 'model_minimal_contact.urdf');

w = warning('off', 'Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
warning('off', 'Drake:RigidBodyManipulator:UnsupportedContactPoints');
r = RigidBodyManipulator(urdf, options);
warning(w);

n_bodies = r.getNumBodies();
%S = load(fullfile(getDrakePath(), 'examples', 'Atlas', 'data', 'atlas_fp.mat'));
S = load(fullfile(getenv('DRC_BASE'), 'software', 'control', 'matlab', 'data', 'atlas_v4_fp.mat'));
q_star = S.xstar(1:r.getNumPositions());
%q_star(r.getBody(r.findLinkId('l_larm')).position_num) = -q_star(r.getBody(r.findLinkId('l_larm')).position_num);
%q_star(r.getBody(r.findLinkId('l_larm')).position_num) = 0;

[jlmin, jlmax] = r.getJointLimits();
inf_idx = isinf(jlmin) | isinf(jlmax);
q_mid = q_star;
q_mid(~inf_idx) = mean([jlmin(~inf_idx), jlmax(~inf_idx)], 2);
jl_half = (jlmax-jlmin)/2;
jl_range = jlmax-jlmin;

options.floating = 'quat';
options.rotation_type = 2;
options.base_or_frame_id = 1;
%options.terrain = RigidBodyFlatTerrain();
options.ignore_self_collisions = true;
r_max = RigidBodyManipulator([], options);
q_nom = zeros(7, n_bodies-1);
q_mid_max = zeros(7, n_bodies-1);
[r_max, robotnum] = r_max.addEmptyRobot('test');
kinsol = r.doKinematics(q_star);
parent_name = cell(n_bodies-1, 1);
child_name = cell(n_bodies-1, 1);
child_xyz = cell(n_bodies-1, 1);
child_joint_axis = cell(n_bodies-1, 1);
for i = 2:n_bodies
  body = r.getBody(i);
  body.robotnum = robotnum;
  r_max = r_max.addLink(body);
  r_max = r_max.addJoint([body.linkname, '_pose'], 'floating_quat', 1, i, [0;0;0], [0;0;0]);
  q_nom(:,i-1) = r.forwardKin(kinsol, i, [0;0;0], options);
  parent_name{i-1} = r.getLinkName(body.parent); 
  child_name{i-1} = body.linkname; 
  child_xyz{i-1} = body.Ttree(1:3,4);
  child_joint_axis{i-1} = body.joint_axis;
end
r_max = r_max.compile();

kinsol = r.doKinematics(q_mid);
for i = 2:n_bodies
  q_mid_max(:,i-1) = r.forwardKin(kinsol, i, [0;0;0], 2);
end

kinsol = r.doKinematics(q_star);
options.rotation_type = 2;
quat_des = cell(n_bodies-1, 1);
%threshold = cell(n_bodies-1, 1);
for i = 2:n_bodies-1
  if r.findLinkId(parent_name{i}) ~= 1
    options.base_or_frame_id = r.findLinkId(parent_name{i});
    xyz_quat = r.forwardKin(kinsol, r.findLinkId(child_name{i}), [0;0;0], options);
    %quat_des{i} = quatConjugate(xyz_quat(4:7));
    quat_des{i} = xyz_quat(4:7);
    %threshold{i} = min(pi,jl_half(r.getBody(r.findLinkId(child_name{i})).position_num));
  end
end


q_nom = q_nom(:);
q0 = getRandomConfiguration(r_max);
q0(3:7:end) = q0(3:7:end)+1;
v0 = zeros(r_max.getNumVelocities(), 1);

v_max = r_max.constructVisualizer();
v_max.draw(0,q0)

relative_position_fun = cell(n_bodies-1, 1);
gaze_constraint = cell(n_bodies-1, 1);
position_constraint = cell(n_bodies-1, 1);
conethreshold = 0;
position_tol = 1e-6;
lb = -position_tol*ones(3,1);
ub = position_tol*ones(3,1);
R3 = realCoordinateSpace(3);
poseExp2poseQuat = [drakeFunction.Identity(R3); drakeFunction.geometry.Exp2Quat()];
postureExp2postureQuat = poseExp2poseQuat.duplicate(r_max.getNumBodies() - 1);
postureExp2postureQuat = compose(drakeFunction.Linear(postureExp2postureQuat.output_frame, r_max.getPositionFrame(), eye(r_max.getNumPositions())), postureExp2postureQuat);
for i = 1:numel(relative_position_fun)
  if r.findLinkId(parent_name{i}) ~= 1
    relative_position_fun{i} = RelativePosition(r_max, parent_name{i}, child_name{i}, child_xyz{i});
    relative_position_fun{i} = relative_position_fun{i}(postureExp2postureQuat);
    %position_constraint{i} = Point2PointDistanceConstraint(r_max, parent_name{i}, child_name{i}, child_xyz{i}, zeros(3,1), 0, position_tol);
    %position_constraint{i} = RelativePositionConstraint(r_max, child_xyz{i}, lb, ub, parent_name{i}, child_name{i});
    %gaze_constraint{i} = RelativeGazeDirConstraint(r_max, child_name{i}, parent_name{i}, child_joint_axis{i}, child_joint_axis{i}, conethreshold);
  end
end
gaze_constraint = gaze_constraint(~cellfun(@isempty, gaze_constraint));
position_constraint = position_constraint(~cellfun(@isempty, position_constraint));

l_foot = r_max.findLinkId('l_foot');
r_foot = r_max.findLinkId('r_foot');
l_hand = r_max.findLinkId('l_hand');
r_hand = r_max.findLinkId('r_hand');
l_foot_points = r.getFrame(r.findFrameId('l_foot_sole')).T(1:3,4);
r_foot_points = r.getFrame(r.findFrameId('r_foot_sole')).T(1:3,4);
lb = repmat([NaN; NaN; 0]-1e-4, 1, size(l_foot_points,2));
ub = repmat([NaN; NaN; 0]+1e-4, 1, size(l_foot_points,2));
l_foot_position = WorldPositionConstraint(r_max, l_foot, l_foot_points, lb, ub);
%l_foot_position = WorldPositionConstraint(r_max, l_foot, l_foot_points, [NaN; NaN; 0.1], [NaN; NaN; NaN]);
r_foot_position = WorldPositionConstraint(r_max, r_foot, r_foot_points, lb, ub);
l_foot_gaze = WorldGazeDirConstraint(r_max, l_foot, [0; 0; 1],[0; 0; 1], 1e-4);
r_foot_gaze = WorldGazeDirConstraint(r_max, r_foot, [0; 0; 1],[0; 0; 1], 1e-4);
target = [1; 0.5; 1.5];
tol = 0.01;
l_hand_position = WorldPositionConstraint(r_max, l_hand, [0; 0.3; 0], target-tol*ones(3,1), target+tol*ones(3,1));

qsc = QuasiStaticConstraint(r_max);
qsc = qsc.setShrinkFactor(0.5);
qsc = qsc.addContact(l_foot, l_foot_points);
qsc = qsc.addContact(r_foot, r_foot_points);
qsc = qsc.setActive(true);

%prog = InverseKinematics(r_max, q_nom, gaze_constraint{:}, position_constraint{:}, qsc, l_foot_position, r_foot_position, l_foot_gaze, r_foot_gaze);
%prog = InverseKinematics(r_max, q_nom, gaze_constraint{:}, position_constraint{:},  qsc, l_foot_position, l_foot_gaze, r_foot_position, r_foot_gaze, l_hand_position);
%prog = InverseKinematics(r_max, q_nom, gaze_constraint{:});
prog = NonlinearProgram(6*(r_max.getNumBodies()-1));
prog = prog.setSolverOptions('snopt', 'LineSearchTolerance', 0.9);
prog = prog.setSolverOptions('snopt', 'MajorFeasibilityTolerance', 1e-4);
prog = prog.setSolverOptions('snopt', 'MajorOptimalityTolerance', 1e-2);
prog = prog.setSolverOptions('snopt', 'print', 'snopt.out');
prog = prog.addDisplayFunction(@(q) displayFun(v_max,q), prog.q_idx);
lb = -1e-4*ones(3,1);
ub = 1e-4*ones(3,1);
quat_idx = reshape(prog.q_idx, 7, []);
quat_idx(1:3,:) = [];
norm_squared = NormSquared(Quaternion());
%cost = diag(prog.Q);
%cost(:) = 1;
%cost(quat_idx(:)) = 0;
%cost(1:3) = 0;
prog = prog.setQ(0*prog.Q);

for i = 1:numel(relative_position_fun)
  if r.findLinkId(parent_name{i}) ~= 1
    %prog = prog.addConstraint(DrakeFunctionConstraint(lb, ub, relative_position_fun{i}), prog.q_idx);
    prog = prog.addConstraint(DrakeFunctionConstraint(1, 1, norm_squared), quat_idx(:,i));
  end
end

squared = drakeFunction.ConstantPower(realCoordinateSpace(1), 2);
for i = 2:numel(child_name)
  quat_diff = RelativeQuaternion(r_max,child_name{i},parent_name{i});
  quat_diff_cost1 = drakeFunction.Linear(Quaternion(), realCoordinateSpace(1), quat_des{i}');
  quat_diff_cost = -1/(r_max.getNumBodies()-1)*squared(quat_diff_cost1(quat_diff));
  cost = DrakeFunctionConstraint(-Inf, Inf, quat_diff_cost);
  prog = prog.addCost(cost, prog.q_idx);
  %[f,df_taylorvar] = geval(@(q) eval(quat_diff,q),q0,struct('grad_method','taylorvar'));
  %[f,df_user] = geval(@(q) eval(quat_diff,q),q0,struct('grad_method','user'));
  %plot((df_taylorvar-df_user)','.');
  %drawnow;
end
prog = prog.addConstraint(ConstantConstraint(q_nom(1:2)), prog.q_idx(1:2));
%hand_constraint = DrakeFunctionConstraint(target - tol, target + tol, WorldPosition(r_max, 'r_hand', [0; -0.3; 0]));
%prog = prog.addConstraint(hand_constraint, prog.q_idx);
keyboard
end

function displayFun(v, q)
  v.draw(0,q);
end

