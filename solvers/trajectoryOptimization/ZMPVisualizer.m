classdef ZMPVisualizer < Visualizer
  properties(SetAccess = protected)
    zmp_samples
    lcmgl_traj
    lcmgl_pt
  end
  methods
    function obj = ZMPVisualizer(zmp_frame)
      obj = obj@Visualizer(zmp_frame);
      obj.lcmgl_traj = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton,'zmp_traj');
      
      obj.lcmgl_pt = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton,'zmp');
      
    end
    
    function draw(obj,t,y)
      obj.lcmgl_traj.glColor3f(0.5,0.5,0);
      obj.lcmgl_traj.glLineWidth(4);
      obj.lcmgl_traj.plot3(obj.zmp_samples(1,:),obj.zmp_samples(2,:),zeros(1,size(obj.zmp_samples,2)));
      obj.lcmgl_traj.switchBuffers();
      obj.lcmgl_pt.glColor3f(1,0,0);
      obj.lcmgl_pt.sphere([y;0],0.02,20,20);
      obj.lcmgl_pt.switchBuffers();
    end
    
    function obj = setZMPSamples(obj,zmp)
      obj.zmp_samples = zmp;
    end
  end
end