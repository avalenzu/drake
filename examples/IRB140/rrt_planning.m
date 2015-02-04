function r = rrt_planning(planning_mode, r, n_smoothing_passes)
  %rng(1)
  if nargin < 1 || isempty(planning_mode), planning_mode = 'rrt'; end
  if nargin < 2
    urdf = fullfile(getDrakePath(), 'examples', 'IRB140', 'irb_140.urdf');
    options = struct();
    options.terrain = RigidBodyFlatTerrain();
    r = RigidBodyManipulator([],options);
    r = r.addRobotFromURDF(urdf, [], [-pi/2; 0; 0]);
    wall = RigidBodyBox([1, 0.1, 0.3], [0.7; 0.1; 0.7], [0; 0; 0]);
    r = r.addGeometryToBody('world',wall);
    wall = RigidBodyBox([0.3, 0.8, 0.1], [-0.0; -0.5; 0.9], [0; 0; 0]);
    r = r.addGeometryToBody('world',wall);
    r = r.compile;
  end
  if nargin < 3 || isempty(n_smoothing_passes), n_smoothing_passes = 10; end
  nq = r.getNumPositions();
  q0 = zeros(nq, 1);
  q0(1) = pi/2;
  qf = q0;
  qf(1) = -pi/2;

  TA = ConfigurationSpaceMotionPlanningTree(r);
  %TA.active_collision_options.body_idx = setdiff(1:r.getNumBodies(), 2);
  TA.min_distance = 0.02;
  TA.visualization_point.body = r.getNumBodies();
  TA.max_edge_length = 0.2;
  TA.max_length_between_constraint_checks = 0.2;
  TA = TA.compile();
  assert(TA.checkConstraints(q0))
  assert(TA.checkConstraints(qf))
  TB = TA;
  %v = TA.rbm.constructVisualizer();

  TA = TA.setLCMGL('TA',[1,0,0]);
  TB = TB.setLCMGL('TB',[0,0,1]);
  
  options.display_after_every = 50;
  options.N = 10000;
  rrt_timer = tic;
  switch planning_mode
    case 'rrt'
      [TA, path_ids_A, info] = TA.rrtNew(q0, qf, TA, options);
    case 'rrt_connect'
      [TA, TB,  path_ids_A, path_ids_B, info] = TA.rrtConnect(q0, qf, TA, TB, options);
      if info == 1
        [TA, id_last] = TA.addPath(TB.extractPath(fliplr(path_ids_B(1:end-1))), path_ids_A(end));
        path_ids_A = TA.getPathToVertex(id_last);
      end
  end
  rrt_time = toc(rrt_timer);
  fprintf('Timing:\n');
  fprintf('  RRT:       %5.2f s\n', rrt_time);
  if info == 1
    smoothing_timer = tic;
    [TA_smooth, id_last] = TA.recursiveConnectSmoothing(path_ids_A, n_smoothing_passes);
    path_ids_A = TA_smooth.getPathToVertex(id_last);
    smoothing_time = toc(smoothing_timer);
    fprintf('  Smoothing: %5.2f s\n', smoothing_time);

    TA_smooth = TA_smooth.setLCMGL('TA_smooth', TA_smooth.line_color);
    drawTree(TA);
    drawTree(TB);
    drawPath(TA_smooth, path_ids_A);

    %q_path = extractPath(TA_smooth, path_ids_A, TB_smooth, path_ids_B);
    q_path = extractPath(TA_smooth, path_ids_A);
    path_length = size(q_path,2);
    traj = PPTrajectory(foh(linspace(0,10,path_length), q_path));
    traj = traj.setOutputFrame(TA.rbm.getPositionFrame());
    %v = r.constructVisualizer();
    v = r.constructVisualizer(struct('use_collision_geometry',true));
    v.playback(traj, struct('slider', true));
  end
end
