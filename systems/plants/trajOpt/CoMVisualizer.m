classdef CoMVisualizer < Visualizer
  properties(SetAccess = protected)
    com_samples % A 3 x n matrix. The sample of CoM trajectory to be visualized
    use_lcmgl
  end
  methods
    function obj = CoMVisualizer(com_frame,use_lcmgl)
      if(~islogical(use_lcmgl) || numel(use_lcmgl) ~= 1)
        error('Drake:CoMVisualizer: use_lcmgl should be a boolean');
      end
      obj = obj@Visualizer(com_frame);
      obj.use_lcmgl = use_lcmgl;
    end
    
    function obj = setCoMSamples(obj,com)
      sizecheck(com,[3,nan]);
      obj.com_samples = com;
    end
    
    function draw(obj,t,y)
      sizecheck(y,[3,1]);
      if(obj.use_lcmgl)
        lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton,'com_traj');
        lcmgl.plot3(obj.com_samples(1,:),obj.com_samples(2,:),obj.com_samples(3,:));
        lcmgl.switchBuffers();
        lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton,'com');
        lcmgl.glColor3f(1,0,0);
        lcmgl.sphere(y,0.03,20,20);
        lcmgl.switchBuffers();
      else
        plot3(obj.com_samples(1,:),obj.com_samples(2,:),obj.com_samples(3,:));
        plot3(y(1),y(2),y(3),'o','MarkerSize',10);
        axis equal
        axis([min(obj.com_samples(1,:))-0.5,max(obj.com_samples(1,:))+0.5 min(obj.com_samples(2,:))-0.5,max(obj.com_samples(2,:))+0.5 min(obj.com_samples(3,:))-1.5,max(obj.com_samples(1,:))+0.5]);
      end
    end
  end
end