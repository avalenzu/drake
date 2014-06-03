function testFixedFootYawCoMPlanning
warning('off','Drake:RigidBody:SimplifiedCollisionGeometry');
warning('off','Drake:RigidBody:NonPositiveInertiaMatrix');
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');

robot = RigidBodyManipulator([getDrakePath,'/examples/Atlas/urdf/atlas_minimal_contact.urdf'],struct('floating',true));
nomdata = load([getDrakePath,'/examples/Atlas/data/atlas_fp.mat']);
nq = robot.getNumDOF;
xstar = nomdata.xstar;
qstar = xstar(1:nq);

l_foot = robot.findLinkInd('l_foot');
r_foot = robot.findLinkInd('r_foot');
l_foot_pt = [0;0;0];
r_foot_pt = [0;0;0];
l_foot_shapes = robot.getBody(l_foot).getContactShapes;
r_foot_shapes = robot.getBody(r_foot).getContactShapes;
l_foot_contact_pts = [];
r_foot_contact_pts = [];
for i=1:length(l_foot_shapes),
  l_foot_contact_pts = [l_foot_contact_pts robot.getBody(l_foot).getContactShapes{i}.getPoints];
end
for i=1:length(r_foot_shapes),
  r_foot_contact_pts = [r_foot_contact_pts robot.getBody(r_foot).getContactShapes{i}.getPoints];
end

robot_mass = robot.getMass();
kinsol_star = robot.doKinematics(qstar,false,false);
com_star = robot.getCOM(kinsol_star);
com0 = com_star;

lfoot_pos_star = robot.forwardKin(kinsol_star,l_foot,l_foot_pt,1);
rfoot_pos_star = robot.forwardKin(kinsol_star,r_foot,r_foot_pt,1);

step_time = 0.4;
stand_time = 0.6;
num_steps = 6;
[lfoot_fsrc_cnstr,rfoot_fsrc_cnstr,tspan] = generateFSRC(robot,num_steps,step_time,stand_time,l_foot,r_foot,l_foot_pt,r_foot_pt,lfoot_pos_star(3),rfoot_pos_star(3),l_foot_contact_pts,r_foot_contact_pts);
com1 = com0+[num_steps*0.6;0;0.0];
lambda = -2*eye(3);
t = tspan(1):0.05:tspan(2);
nT = length(t);
lfoot_yaw = 0;
rfoot_yaw = 0;
yaw = [lfoot_yaw*ones(num_steps,1);rfoot_yaw*ones(num_steps,1)];
Q_comddot = 1*diag([1;0;1]);
c_margin = 0.1;
robot_inertia = robot.getCMM(kinsol_star);
robot_inertia = robot_inertia(1:3,4:6);
robot_dim = 1;
com_traj_order = 3;
planning = FixedFootYawCoMPlanning(robot_mass,robot_dim,robot_inertia,t,lambda,c_margin,Q_comddot,[lfoot_fsrc_cnstr,rfoot_fsrc_cnstr],yaw',com_traj_order);
g = 9.8;
planning = planning.addCoMBounds([1 nT],[com0 com1],[com0 com1]);
planning = planning.addCoMBounds(1:nT,[-inf(2,nT);0.8*ones(1,nT)],[inf(2,nT);1*ones(1,nT)]);
planning = planning.addCoMdotBounds([1,nT],zeros(3,2),zeros(3,2));
planning = planning.addH0Bounds(zeros(3,1),zeros(3,1));
planning = planning.addCoMdotBounds(2:nT-1,bsxfun(@times,ones(1,nT-2),[-1;-1;-0.5]),bsxfun(@times,ones(1,nT-2),[1;1;0.5]));
planning = planning.addCoMddotBounds(1:nT,bsxfun(@times,ones(1,nT),[-g;-g;-0.99*g]),bsxfun(@times,ones(1,nT),[g;g;g]));
% planning = planning.addHBounds(nT,zeros(3,1),zeros(3,1));
com_rfoot_vertices = [(-1).^([0 0 0 0 1 1 1 1]);(-1).^([0 0 1 1 0 0 1 1]);(-1).^([0 1 0 1 0 1 0 1])].*bsxfun(@times,ones(1,8),[0.3;0.1;0.15])+bsxfun(@times,ones(1,8),[0;0.15;0.9]);
com_lfoot_vertices = [(-1).^([0 0 0 0 1 1 1 1]);(-1).^([0 0 1 1 0 0 1 1]);(-1).^([0 1 0 1 0 1 0 1])].*bsxfun(@times,ones(1,8),[0.3;0.1;0.15])+bsxfun(@times,ones(1,8),[0;-0.15;0.9]);
for i = 1:num_steps
  planning = planning.addCoMFootPolygon(num_steps+i,com_rfoot_vertices);
  planning = planning.addCoMFootPolygon(i,com_lfoot_vertices);
end
planning.nlp_step = planning.nlp_step.setSolverOptions('snopt','superbasicslimit',3000);
r2l_xy_polygon = RelativeFootPositionPolygon([0.6 0.6 0.2 0.2;-0.1 -0.4 -0.1 -0.4]);
[A_polygon,b_polygon] = r2l_xy_polygon.halfspace(lfoot_yaw);
for i = 1:num_steps-1
planning = planning.addFootPolygon([i num_steps+i+1],A_polygon,b_polygon);
end

planning = planning.addFootPositionConstraint(1,BoundingBoxConstraint(lfoot_pos_star(1:2),lfoot_pos_star(1:2)));
planning = planning.addFootPositionConstraint(num_steps+1,BoundingBoxConstraint(rfoot_pos_star(1:2),rfoot_pos_star(1:2)));
lfoot_pos_end = com1-com0+lfoot_pos_star(1:3);
rfoot_pos_end = com1-com0+rfoot_pos_star(1:3);
planning = planning.addFootPositionConstraint(num_steps,BoundingBoxConstraint(lfoot_pos_end(1:2),lfoot_pos_end(1:2)));
planning = planning.addFootPositionConstraint(2*num_steps,BoundingBoxConstraint(rfoot_pos_end(1:2),rfoot_pos_end(1:2)));
[com_sol,comp_sol,compp_sol,foot_pos_sol,Hdot_sol,H_sol,F_sol,foot_position_cnstr,foot_quat_cnstr] = planning.solve(robot,zeros(3,1));

end

function [lfoot_fsrc_cnstr,rfoot_fsrc_cnstr,tspan] = generateFSRC(robot,num_steps,step_time,stand_time,l_foot,r_foot,l_foot_pt,r_foot_pt,l_foot_z_offset,r_foot_z_offset,l_foot_contact_pts,r_foot_contact_pts)
A_iris = [0 1 0;0 -1 0;1 0 0;-1 0 0];
b_iris = [100;100;100;100];
C_iris = 10 *eye(3);
d_iris = zeros(3,1);
mu = 1;
num_edges = 5;

% right foot as leading foot
if(stand_time<=step_time)
  error('stand time should be larger than step time');
end


l_land_time = 0;
r_land_time = (stand_time-step_time)/2;
lfoot_fsrc_cnstr = FootStepRegionContactConstraint.empty(num_steps,0);
rfoot_fsrc_cnstr = FootStepRegionContactConstraint.empty(num_steps,0);
r_fsr = FootStepRegionConstraint(robot,A_iris,b_iris,C_iris,d_iris,r_foot,r_foot_pt,r_foot_z_offset,[0;0;1],[0;0;0],[0,(stand_time-step_time)/2]);
rfoot_fsrc_cnstr(1) = FootStepRegionContactConstraint(r_fsr,mu,num_edges,r_foot_contact_pts);
r_land_time = r_land_time+step_time;
for i = 1:num_steps-1
  l_fsr = FootStepRegionConstraint(robot,A_iris,b_iris,C_iris,d_iris,l_foot,l_foot_pt,l_foot_z_offset,[0;0;1],[0;0;0],[l_land_time,l_land_time+stand_time]);
  lfoot_fsrc_cnstr(i) = FootStepRegionContactConstraint(l_fsr,mu,num_edges,l_foot_contact_pts);
  l_land_time = l_land_time+step_time+stand_time;
  r_fsr = FootStepRegionConstraint(robot,A_iris,b_iris,C_iris,d_iris,r_foot,r_foot_pt,r_foot_z_offset,[0;0;1],[0;0;0],[r_land_time,r_land_time+stand_time]);
  rfoot_fsrc_cnstr(i+1) = FootStepRegionContactConstraint(r_fsr,mu,num_edges,r_foot_contact_pts);
  r_land_time = r_land_time+step_time+stand_time;
end
l_fsr = FootStepRegionConstraint(robot,A_iris,b_iris,C_iris,d_iris,l_foot,l_foot_pt,l_foot_z_offset,[0;0;1],[0;0;0],[l_land_time,rfoot_fsrc_cnstr(num_steps).foot_step_region_cnstr.tspan(end)]);
lfoot_fsrc_cnstr(num_steps) = FootStepRegionContactConstraint(l_fsr,mu,num_edges,l_foot_contact_pts);
tspan = [0 rfoot_fsrc_cnstr(num_steps).foot_step_region_cnstr.tspan(end)];
end