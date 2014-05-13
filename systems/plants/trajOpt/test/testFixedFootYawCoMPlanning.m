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

% test F step of the bilinear alternation
step_time = 0.4;
stand_time = 0.6;
num_steps = 4;
[lfoot_fsrc_cnstr,rfoot_fsrc_cnstr,tspan] = generateFSRC(robot,num_steps,step_time,stand_time,l_foot,r_foot,l_foot_pt,r_foot_pt,lfoot_pos_star(3),rfoot_pos_star(3),l_foot_contact_pts,r_foot_contact_pts);
com1 = com0+[num_steps*0.5;0.2;0.05];
lambda = [-1 0 0;1 -1 0;2 3 -1];
t = tspan(1):0.05:tspan(2);
nT = length(t);
lfoot_yaw = 0;
rfoot_yaw = 0;
yaw = [lfoot_yaw*ones(num_steps,1);rfoot_yaw*ones(num_steps,1)];
Q_comddot = 1*diag([1;0;1]);
c_margin = 0.1;
dt_max = 0.2;
sdot_max = 10;
Q_H = eye(3);
Q_Hdot = eye(3);
robot_dim = 1;
planning = FixedFootYawCoMPlanning(robot_mass,robot_dim,t,lambda,c_margin,dt_max,sdot_max,Q_comddot,[lfoot_fsrc_cnstr,rfoot_fsrc_cnstr],yaw');
planning.p_step = planning.p_step.addBoundingBoxConstraint(BoundingBoxConstraint(com0,com0),planning.p_step.com_idx(:,1));
planning.p_step = planning.p_step.addBoundingBoxConstraint(BoundingBoxConstraint(com1,com1),planning.p_step.com_idx(:,end));
planning.p_step = planning.p_step.setCoMVelocityBounds([1,nT],zeros(3,2),zeros(3,2));
planning.p_step = planning.p_step.addBoundingBoxConstraint(BoundingBoxConstraint(zeros(3,1),zeros(3,1)),planning.p_step.H_idx(:,1));
planning.p_step = planning.p_step.addBoundingBoxConstraint(BoundingBoxConstraint(0.5*ones(planning.p_step.nT,1),2*ones(planning.p_step.nT,1)),planning.p_step.com_idx(3,:));
planning.f_step = planning.f_step.addBoundingBoxConstraint(BoundingBoxConstraint(zeros(3,1),zeros(3,1)),planning.f_step.H_idx(:,1));
planning.seed_step = planning.seed_step.addBoundingBoxConstraint(BoundingBoxConstraint(com0,com0),planning.seed_step.com_idx(:,1));
planning.seed_step = planning.seed_step.addBoundingBoxConstraint(BoundingBoxConstraint(com1,com1),planning.seed_step.com_idx(:,end));
planning.seed_step = planning.seed_step.addBoundingBoxConstraint(BoundingBoxConstraint(0.5*ones(planning.seed_step.nT,1),2*ones(planning.seed_step.nT,1)),planning.seed_step.com_idx(3,:));
planning.seed_step = planning.seed_step.setCoMVelocityBounds([1,nT],zeros(3,2),zeros(3,2));
planning.nlp_step = planning.nlp_step.addBoundingBoxConstraint(BoundingBoxConstraint(com0,com0),planning.nlp_step.com_idx(:,1));
planning.nlp_step = planning.nlp_step.addBoundingBoxConstraint(BoundingBoxConstraint(com1,com1),planning.nlp_step.com_idx(:,end));
planning.nlp_step = planning.nlp_step.addBoundingBoxConstraint(BoundingBoxConstraint(0.5*ones(planning.nlp_step.nT,1),2*ones(planning.nlp_step.nT,1)),planning.seed_step.com_idx(3,:));
planning.nlp_step = planning.nlp_step.addBoundingBoxConstraint(BoundingBoxConstraint(zeros(3,1),zeros(3,1)),planning.nlp_step.comp_idx(:,1));
planning.nlp_step = planning.nlp_step.addBoundingBoxConstraint(BoundingBoxConstraint(zeros(3,1),zeros(3,1)),planning.nlp_step.comp_idx(:,end));
planning.nlp_step = planning.nlp_step.setSolverOptions('snopt','superbasicslimit',3000);
r2l_xy_polygon = RelativeFootPositionPolygon([-0.1 -0.1 0.1 0.1;0.2 0.5 0.5 0.2]);
[A_polygon,b_polygon] = r2l_xy_polygon.halfspace(lfoot_yaw);
for i = 1:num_steps-1
planning.p_step = planning.p_step.addKinematicPolygon([i num_steps+i+1],A_polygon,b_polygon);
planning.seed_step = planning.seed_step.addKinematicPolygon([i num_steps+i+1],A_polygon,b_polygon);
end

sdot0 = ones(1,nT);
planning_nlp = CoMForcePlanning(robot_mass,t,lambda,c_margin,dt_max,sdot_max,Q_comddot,[lfoot_fsrc_cnstr,rfoot_fsrc_cnstr]);
planning_nlp = planning_nlp.addBoundingBoxConstraint(BoundingBoxConstraint(com0,com0),planning_nlp.com_idx(:,1));
planning_nlp = planning_nlp.addBoundingBoxConstraint(BoundingBoxConstraint(com1,com1),planning_nlp.com_idx(:,end));
planning_nlp = planning_nlp.addBoundingBoxConstraint(BoundingBoxConstraint(yaw,yaw),planning_nlp.yaw_idx);


% [com,comp,compp,foot_pos,F,sdotsquare] = planning.seed_step.solve(sdot0);
% [Hbar,Hdot,sigma] = planning.seed_step.angularMomentum(com,foot_pos,F,zeros(3,1));
% planning_nlp.solve(com,comp,compp,foot_pos,yaw,F,sqrt(sdotsquare),Hdot,Hbar);
[com_sol,comp_sol,compp_sol,foot_pos_sol,Hdot_sol,F_sol] = planning.solve(sdot0,zeros(3,1));

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
lfoot_fsrc_cnstr = cell(1,num_steps);
rfoot_fsrc_cnstr = cell(1,num_steps);
r_fsr = FootStepRegionConstraint(robot,A_iris,b_iris,C_iris,d_iris,r_foot,r_foot_pt,r_foot_z_offset,[0;0;1],[0;0;0],[0,(stand_time-step_time)/2]);
rfoot_fsrc_cnstr{1} = FootStepRegionContactConstraint(r_fsr,mu,num_edges,r_foot_contact_pts);
r_land_time = r_land_time+step_time;
for i = 1:num_steps-1
  l_fsr = FootStepRegionConstraint(robot,A_iris,b_iris,C_iris,d_iris,l_foot,l_foot_pt,l_foot_z_offset,[0;0;1],[0;0;0],[l_land_time,l_land_time+stand_time]);
  lfoot_fsrc_cnstr{i} = FootStepRegionContactConstraint(l_fsr,mu,num_edges,l_foot_contact_pts);
  l_land_time = l_land_time+step_time+stand_time;
  r_fsr = FootStepRegionConstraint(robot,A_iris,b_iris,C_iris,d_iris,r_foot,r_foot_pt,r_foot_z_offset,[0;0;1],[0;0;0],[r_land_time,r_land_time+stand_time]);
  rfoot_fsrc_cnstr{i+1} = FootStepRegionContactConstraint(r_fsr,mu,num_edges,r_foot_contact_pts);
  r_land_time = r_land_time+step_time+stand_time;
end
l_fsr = FootStepRegionConstraint(robot,A_iris,b_iris,C_iris,d_iris,l_foot,l_foot_pt,l_foot_z_offset,[0;0;1],[0;0;0],[l_land_time,rfoot_fsrc_cnstr{end}.foot_step_region_cnstr.tspan(end)]);
lfoot_fsrc_cnstr{end} = FootStepRegionContactConstraint(l_fsr,mu,num_edges,l_foot_contact_pts);
tspan = [0 rfoot_fsrc_cnstr{end}.foot_step_region_cnstr.tspan(end)];
end