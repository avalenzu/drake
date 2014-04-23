function testFixedFootYawCoMPlanning
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
b_iris = [10;-10;10;10];
C_iris = 0.1*eye(3);
d_iris = zeros(3,1);
fsr_cnstr1 = FootStepRegionConstraint(robot,A_iris,b_iris,C_iris,d_iris,l_foot,l_foot_pt,lfoot_pos_star(3),[0;0;1],[0;0;0],tspan);
fsr_cnstr2 = FootStepRegionConstraint(robot,A_iris,b_iris,C_iris,d_iris,r_foot,r_foot_pt,rfoot_pos_star(3),[0;0;1],[0;0;0],tspan);
end