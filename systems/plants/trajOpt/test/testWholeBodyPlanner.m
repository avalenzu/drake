checkDependency('lcmgl');
urdf = drakeExamplePath('Acrobot/Acrobot.urdf');
r = RigidBodyManipulator(urdf);
v = r.constructVisualizer();
lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(), ...
                                  'testWholeBodyPlanner');
nq = r.getNumPositions();
%nv = r.getNumVelocities();
nv = nq;
nx = nq + nv;
tspan = [0,2];
nT = 5;
t = linspace(tspan(1),tspan(2),nT);

target = [1;0;2];
end_effector = [0; 0; -2];

q0 = zeros(nq,1);
v0 = zeros(nv,1);
x0 = [q0; v0];

q_nom = q0;

lb = target; lb(2) = NaN;
ub = target; ub(2) = NaN;
grasp = WorldPositionConstraint(r,3,end_effector,lb,ub,tspan([2,2]));

initial_pos_constraint = PostureConstraint(r,tspan([1,1]));
initial_pos_constraint = initial_pos_constraint.setJointLimits([1;2],q0,q0);

%prog_end = InverseKin(r, q_nom, grasp);
%[q_end,F_end,info_end,infeasible_constraint_end] = solve(prog_end,q0);

q_end = q0;

kinsol = doKinematics(r,q_end);
end_effector_end_in_world = forwardKin(r,kinsol,3,end_effector);
lcmgl.glDrawAxes();
% Draw target
lcmgl.glColor4f(0,1,0,0.5);
lcmgl.sphere(target,0.2,20,20);
% Draw end effector
lcmgl.glColor4f(0,0,0,1);
lcmgl.sphere(end_effector_end_in_world,0.1,20,20);
lcmgl.switchBuffers();

v.draw(0,[q_end;0*q_end]);

q_nom_traj = PPTrajectory(foh(tspan, [q0,q_end]));

prog_reach = WholeBodyPlanner(r,t,q_nom_traj,false,x0,grasp,initial_pos_constraint);
prog_reach = prog_reach.setTrackingError(diag([1,1]),diag([1,1]));
prog_reach = setCheckGrad(prog_reach,true);
[x_traj_reach,F_reach,info_reach,infeasible_constraint_reach] = solve(prog_reach,q_nom_traj);
