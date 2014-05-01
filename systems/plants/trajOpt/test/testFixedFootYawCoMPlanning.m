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
com1 = com0+[0.1;0;0.1];
lfoot_pos_star = robot.forwardKin(kinsol_star,l_foot,l_foot_pt,1);
rfoot_pos_star = robot.forwardKin(kinsol_star,r_foot,r_foot_pt,1);

% test F step of the bilinear alternation
tspan = [0,1];
A_iris = [0 1 0;0 -1 0;1 0 0;-1 0 0];
b_iris = [10;10;10;10];
C_iris = 10 *eye(3);
d_iris = zeros(3,1);
fsr_cnstr1 = FootStepRegionConstraint(robot,A_iris,b_iris,C_iris,d_iris,l_foot,l_foot_pt,lfoot_pos_star(3),[0;0;1],[0;0;0],tspan);
fsr_cnstr2 = FootStepRegionConstraint(robot,A_iris,b_iris,C_iris,d_iris,r_foot,r_foot_pt,rfoot_pos_star(3),[0;0;1],[0;0;0],tspan);
mu = 1;
num_edges = 5;
lfoot_fsrc_cnstr = FootStepRegionContactConstraint(fsr_cnstr1,mu,num_edges,l_foot_contact_pts);
rfoot_fsrc_cnstr = FootStepRegionContactConstraint(fsr_cnstr2,mu,num_edges,r_foot_contact_pts);
lambda = [-1 0 0;1 -1 0;2 3 -1];
nT = 21;
t = linspace(tspan(1),tspan(2),nT);
lfoot_yaw = 0;
rfoot_yaw = 0;
Q_comddot = 0*eye(3);
c_margin = 0.1;
dt_max = 0.2;
sdot_max = 10;
planning = FixedFootYawCoMPlanning(robot_mass,t,lambda,c_margin,dt_max,sdot_max,Q_comddot,lfoot_fsrc_cnstr,lfoot_yaw,rfoot_fsrc_cnstr,rfoot_yaw);
planning.p_step = planning.p_step.setVarBounds(com0,com0,planning.p_step.com_idx(:,1));
planning.p_step = planning.p_step.setVarBounds(com1,com1,planning.p_step.com_idx(:,end));
planning.p_step = planning.p_step.setCoMVelocityBounds([1,nT],-ones(3,2),ones(3,2));
planning.p_step = planning.p_step.setVarBounds(zeros(3,1),zeros(3,1),planning.p_step.H_idx(:,1));
planning.f_step = planning.f_step.setVarBounds(zeros(3,1),zeros(3,1),planning.f_step.H_idx(:,1));
F_edge = robot_mass*9.81/(2*4*num_edges);
F = repmat({[{F_edge*ones(num_edges,4)};{F_edge*ones(num_edges,4)}]},1,length(t));
sigma = 1e10;
r2l_xy_polygon = RelativeFootPositionPolygon([0.2 0.5 0.5 0.2;-0.1 -0.1 0.1 0.1]);
[A_polygon,b_polygon] = r2l_xy_polygon.halfspace(lfoot_yaw);
planning.p_step = planning.p_step.addKinematicPolygon([1 2],A_polygon,b_polygon);
d_com = (com1-com0)/(nT-1);
com = bsxfun(@times,com0,ones(1,nT))+bsxfun(@times,d_com,(0:nT-1));
comp = diff(com,1,2)*(nT-1);
comp = [zeros(3,1) comp];
compp = diff(comp,1,2)*(nT-1);
compp = [zeros(3,1) compp];
foot_pos = [-0.1 0.15;-0.03 -0.03];

planning_nlp = CoMForcePlanning(robot_mass,t,lambda,c_margin,dt_max,sdot_max,Q_comddot,lfoot_fsrc_cnstr,rfoot_fsrc_cnstr);
[com_sol,comp_sol,compp_sol,foot_pos_sol,Hdot_sol,F_sol] = planning.solve(com,comp,compp,foot_pos,sigma);

end