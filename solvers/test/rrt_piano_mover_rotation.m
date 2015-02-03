function rrt_piano_mover_rotation(n_obstacles)
  if nargin < 1 || isempty(n_obstacles)
    n_obstacles = 50;
  end
  urdf = fullfile(getDrakePath, 'systems', 'plants', 'test', 'FallingBrick.urdf');
  options.floating = true;
  r = RigidBodyManipulator(urdf, options);
  box_size = [1;1;1];
  xyz_min = [-5; -5; -5];
  xyz_max = [5; 5; 5];
  robot_geom = RigidBodyBox([1, 5, 0.5], [0;2.5;0], [0;pi/2;0]);
  r = r.addGeometryToBody(2, robot_geom);
  r = r.compile();
  xyz0 = [0; 0; 0];
  q0 = rpy2quat([0; 0; 0]);
  qf = rpy2quat([pi/2; 0; 0]);
  rpy0 = quat2rpy(q0);
  rpyf = quat2rpy(qf);

  prob = MotionPlanningProblem(3);
  min_distance = 0.2;
  min_distance_penalty = drakeFunction.kinematic.SmoothDistancePenalty(r, min_distance);
  i = 0;
  while i < n_obstacles
    xyz = xyz_min + rand(3,1).*(xyz_max - xyz_min);
    rpy = uniformlyRandomRPY();
    obstacle = RigidBodyBox(box_size, xyz, rpy);
    r_new = r.addGeometryToBody('world', obstacle);
    r_new = r_new.compile();
    min_distance_penalty = min_distance_penalty.setRigidBodyManipulator(r_new);
    if collisionConstraint(q0) && collisionConstraint(qf)
      r = r_new;
      i = i + 1;
    end
  end
  r = r.compile();
  min_distance_penalty = min_distance_penalty.setRigidBodyManipulator(r);

  v = r.constructVisualizer();
  v.draw(0,[xyz0; rpy0]);
  TA = SO3MotionPlanningTree(@collisionConstraint);
  TA.max_edge_length = 0.01;
  TA.max_length_between_constraint_checks = 0.001;
  TA.visualization_point = [0; 5; 0];
  TA.visualization_origin = xyz0;

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
    xyz_path = repmat(xyz0, 1, path_length);
    rpy_path = NaN(3, path_length);
    for i = 1:path_length
      rpy_path(:,i) = quat2rpy(q_path(:,i));
    end
    traj = PPTrajectory(foh(linspace(0,20,path_length), [xyz_path; rpy_path]));
    traj = traj.setOutputFrame(r.getPositionFrame());
    v.playback(traj, struct('slider', true));
  end

  function valid = collisionConstraint(q)
    rpy = quat2rpy(q);
    valid = (abs(eval(min_distance_penalty, [xyz0; rpy])) < 1e-6);
    %pts = r.allCollisions([xyz;rpy]);
    %valid = isempty(pts);
  end

end
