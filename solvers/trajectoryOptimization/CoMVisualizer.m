classdef CoMVisualizer < Visualizer
  properties(SetAccess = protected)
    com_samples % A 3 x n matrix. The sample of CoM trajectory to be visualized
    lcmgl_traj
    lcmgl_pt
  end
  methods
    function obj = CoMVisualizer(com_frame)
      obj = obj@Visualizer(com_frame);
      obj.lcmgl_traj = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton,'com_traj');
      
      obj.lcmgl_pt = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton,'com');
      
    end
    
    function obj = setCoMSamples(obj,com)
      sizecheck(com,[3,nan]);
      obj.com_samples = com;
    end
    
    function draw(obj,t,y)
      sizecheck(y,[3,1]);
      obj.lcmgl_traj.glColor3f(1,0,1);
      obj.lcmgl_traj.glPointSize(0.3);
      obj.lcmgl_traj.glLineWidth(2);
      obj.lcmgl_traj.plot3(obj.com_samples(1,:),obj.com_samples(2,:),obj.com_samples(3,:));
      obj.lcmgl_traj.switchBuffers();
      obj.lcmgl_pt.glColor3f(1,0,0);
      obj.lcmgl_pt.sphere(y,0.05,20,20);
      obj.lcmgl_pt.switchBuffers();
    end
  end
end