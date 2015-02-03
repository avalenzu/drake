function rrt_piano_mover_translation(n_obstacles)
  if nargin < 1 || isempty(n_obstacles)
    n_obstacles = 50;
  end
  urdf = fullfile(getDrakePath, 'systems', 'plants', 'test', 'FallingBrick.urdf');
  options.floating = true;
  r = RigidBodyManipulator(urdf, options);
  box_size = [1;1;1];
  xyz_min = [-5; -5; 0];
  xyz_max = [15; 15; 10];
  for i = 1:n_obstacles
    xyz = xyz_min + rand(3,1).*(xyz_max - xyz_min);
    rpy = uniformlyRandomRPY();
    obstacle = RigidBodyBox(box_size, xyz, rpy);
    r = r.addGeometryToBody('world', obstacle);
  end
  robot_geom = RigidBodyBox([1, 10, 0.5], [0;0;0], [0;pi/2;0]);
  r = r.addGeometryToBody(2, robot_geom);
  robot_geom = RigidBodyBox([1, 10, 0.5], [0;0;0], [0;pi/2;pi/2]);
  r = r.addGeometryToBody(2, robot_geom);
  r = r.compile();
  q0 = [5; 5; -2];
  qf = [5; 5; 12];

  prob = MotionPlanningProblem(3);
  min_distance = 0.2;
  min_distance_penalty = drakeFunction.kinematic.SmoothDistancePenalty(r, min_distance);
  v = r.constructVisualizer();
  v.draw(0,[q0; zeros(3,1)]);
  TA = R3MotionPlanningTree(@collisionConstraint);
  TA.max_edge_length = 0.5;
  TA.max_length_between_constraint_checks = 0.1;
  TA.lb = [0; 0; -2];
  TA.ub = [10; 10; 12];

  TB = TA;

  TA = TA.setLCMGL('TA',[1,0,0]);
  TB = TB.setLCMGL('TB',[0,0,1]);

  options.display_after_every = 50;
  [TA, TB,  path_ids_A, path_ids_B, info] = prob.rrtConnect(q0, qf, TA, TB, options);
  %[TA, path_ids_A, info] = prob.rrtNew(q0, qf, TA, options);
  if info == 1
    q_path = extractPath(TA, path_ids_A, TB, path_ids_B);
    %q_path = extractPath(TA, path_ids_A);
    path_length = size(q_path,2);
    rpy_path = zeros(3, path_length);
    traj = PPTrajectory(foh(linspace(0,20,path_length), [q_path(1:3,:); rpy_path]));
    traj = traj.setOutputFrame(r.getPositionFrame());
    v.playback(traj, struct('slider', true));
  end

  function valid = collisionConstraint(q)
    xyz = q(1:3);
    rpy = zeros(3,1);
    valid = (abs(eval(min_distance_penalty, [xyz; rpy])) < 1e-6);
    %pts = r.allCollisions([xyz;rpy]);
    %valid = isempty(pts);
  end

end
