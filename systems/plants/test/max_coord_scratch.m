function max_coord_scratch
import drakeFunction.kinematic.*
import drakeFunction.euclidean.*
import drakeFunction.frames.*
options = struct();
options.floating = true;
options.use_new_kinsol = true;
urdf = fullfile(getDrakePath(), 'examples', 'Atlas', 'urdf', 'atlas_convex_hull.urdf');

w = warning('off', 'Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
warning('off', 'Drake:RigidBodyManipulator:UnsupportedContactPoints');
r = RigidBodyManipulator(urdf, options);
warning(w);

n_bodies = r.getNumBodies();
S = load(fullfile(getDrakePath(), 'examples', 'Atlas', 'data', 'atlas_fp.mat'));
q_star = S.xstar(1:r.getNumPositions());

[jlmin, jlmax] = r.getJointLimits();
inf_idx = isinf(jlmin) | isinf(jlmax);
q_mid = q_star;
q_mid(~inf_idx) = mean([jlmin(~inf_idx), jlmax(~inf_idx)], 2);
%jl_half = (jlmax-jlmin)/2;
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
  %r_max = r_max.addFloatingBase(1, i, [0;0;0], [0;0;0], 'quat');
  r_max = r_max.addJoint([body.linkname, '_pose'], 'floating_quat', 1, i, [0;0;0], [0;0;0]);
  q_nom(:,i-1) = r.forwardKin(kinsol, i, [0;0;0], options);
  %if (6 <= i && i <= 9) || i == 17
    %disp(i)
    %q_nom(4:7,i-1) = [1; 0; 0; 0];
  %end
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

kinsol = r.doKinematics(q_mid);
options.rotation_type = 2;
quat_des = cell(n_bodies-1, 1);
threshold = cell(n_bodies-1, 1);
for i = 2:n_bodies-1
  if r.findLinkId(parent_name{i}) ~= 1
    options.base_or_frame_id = r.findLinkId(parent_name{i});
    xyz_quat = r.forwardKin(kinsol, r.findLinkId(child_name{i}), [0;0;0], options);
    quat_des{i} = quatConjugate(xyz_quat(4:7));
    %quat_des{i} = xyz_quat(4:7);
    if strcmp(child_name{i}, 'l_larm')
      %quat_des{i} = xyz_quat(4:7);
      %quat_des{i} = [1;0;0;0];
      %threshold{i} = 0;
      threshold{i} = min(pi,jl_range(r.getBody(r.findLinkId(child_name{i})).position_num));
    else
      threshold{i} = min(pi,jl_range(r.getBody(r.findLinkId(child_name{i})).position_num));
    end
    %threshold{i} = 0;
  end
end


q_nom = q_nom(:);
%q_nom = q_nom + 0.1*rand(size(q_nom));
q0 = 2*rand(size(q_nom))-1;
q0(3:7:end) = q0(3:7:end)+1;
%q0 = q_nom;
%q0(1:7:end) = q0(1:7:end)+rand(r_max.getNumBodies()-1,1);
v0 = zeros(r_max.getNumVelocities(), 1);

v_max = r_max.constructVisualizer();
v_max.draw(0,q0)

relative_position_fun = cell(n_bodies-1, 1);
gaze_constraint = cell(n_bodies-1, 1);
for i = 1:numel(relative_position_fun)
  if r.findLinkId(parent_name{i}) ~= 1
    relative_position_fun{i} = RelativePosition(r_max, parent_name{i}, child_name{i}, child_xyz{i});
    if strcmp(child_name{i}, 'l_larm')
      %gaze_constraint{i} = RelativeGazeOrientConstraint(r_max, [-Inf, Inf], r_max.findLinkId(parent_name{i}), r_max.findLinkId(child_name{i}), [0;0;1], quat_des{i}, threshold{i}, 1e-6);
    else
      gaze_constraint{i} = RelativeGazeOrientConstraint(r_max, [-Inf, Inf], r_max.findLinkId(parent_name{i}), r_max.findLinkId(child_name{i}), child_joint_axis{i}, quat_des{i}, threshold{i}, 1e-6);
    end
  end
end
gaze_constraint = gaze_constraint(~cellfun(@isempty, gaze_constraint));

%prog = InverseKinematics(r_max, q0, gaze_constraint{:});
%prog = InverseKinematics(r_max, q_nom);
prog = InverseKinematics(r_max, q_nom, gaze_constraint{:});
prog = prog.setSolverOptions('snopt', 'LineSearchTolerance', 0.9);
prog = prog.setSolverOptions('snopt', 'MajorFeasibilityTolerance', 1e-4);
prog = prog.setSolverOptions('snopt', 'MajorOptimalityTolerance', 1e-4);
prog = prog.addDisplayFunction(@(q) displayFun(v_max,q), prog.q_idx);
lb = -1e-6*ones(3,1);
ub = 1e-6*ones(3,1);
quat_idx = reshape(prog.q_idx, 7, []);
quat_idx(1:3,:) = [];
norm_squared = NormSquared(Quaternion());

for i = 1:numel(relative_position_fun)
  if r.findLinkId(parent_name{i}) ~= 1
    prog = prog.addConstraint(DrakeFunctionConstraint(lb, ub, relative_position_fun{i}), prog.q_idx);
    prog = prog.addConstraint(DrakeFunctionConstraint(1, 1, norm_squared), quat_idx(:,i));
  end
end
prog = prog.addConstraint(ConstantConstraint(q_nom(1:7)), prog.q_idx(1:7));
keyboard
end

function displayFun(v, q)
  %pause(0.);
  v.draw(0,q);
end

