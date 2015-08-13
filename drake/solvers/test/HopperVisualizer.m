classdef HopperVisualizer < Visualizer
  properties
    v
    Ftraj
    rHipTraj
    lcmgl
    r_foot_inds = 7:9;
  end
  methods
    function obj = HopperVisualizer(v)
      obj = obj@Visualizer(v.getInputFrame());
      obj.v = v;
      obj.lcmgl = LCMGLClient();
    end

    function playback(obj, qtraj, Ftraj, rHipTraj, varargin)
      obj.Ftraj = Ftraj;
      obj.rHipTraj = rHipTraj;
      playback@Visualizer(obj, qtraj, varargin{:});
    end

    function draw(obj, t, q)
      obj.lcmgl.glColor3f(0, 1, 0);
      F = eval(obj.Ftraj, t);
      F = [F(1); 0; F(2)];
      r_foot = q(obj.r_foot_inds);
      pt = r_foot + F;
      obj.lcmgl.line3(r_foot(1), r_foot(2) ,r_foot(3), pt(1), pt(2), pt(3));
      r_hip = eval(obj.rHipTraj, t);
      r_hip = q(1:3) + [r_hip(1); 0; r_hip(2)];
      obj.lcmgl.glColor3f(1, 0, 0);
      obj.lcmgl.sphere(r_hip, 0.01, 20, 20);
      obj.lcmgl.switchBuffers();
      obj.v.draw(t, q);
    end
  end
end
