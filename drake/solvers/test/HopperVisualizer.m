classdef HopperVisualizer < Visualizer
  properties
    v
    Ftraj
    lcmgl
    r_foot_inds = 7:9;
  end
  methods
    function obj = HopperVisualizer(v)
      obj = obj@Visualizer(v.getInputFrame());
      obj.v = v;
      obj.lcmgl = LCMGLClient();
    end

    function playback(obj, qtraj, Ftraj, varargin)
      obj.Ftraj = Ftraj;
      playback@Visualizer(obj, qtraj, varargin{:});
    end

    function draw(obj, t, q)
      obj.lcmgl.glColor3f(0, 1, 0);
      F = eval(obj.Ftraj, t);
      F = [F(1); 0; F(2)];
      r_foot = q(obj.r_foot_inds);
      pt = r_foot + F;
      obj.lcmgl.line3(r_foot(1), r_foot(2) ,r_foot(3), pt(1), pt(2), pt(3));
      obj.lcmgl.switchBuffers();
      obj.v.draw(t, q);
    end
  end
end
