classdef ZMPVisualizer < Visualizer
  properties(SetAccess = protected)
    use_lcmgl
  end
  methods
    function obj = ZMPVisualizer(zmp_frame,use_lcmgl)
      if(~islogical(use_lcmgl) || numel(use_lcmgl) ~= 1)
        error('Drake:ZMPVisualizer: use_lcmgl should be a boolean');
      end
      obj = obj@Visualizer(zmp_frame);
      obj.use_lcmgl = use_lcmgl;
    end
    
    function draw(obj,t,y)
      if(obj.use_lcmgl)
        lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton,'zmp');
        lcmgl.glColor3f(1,0,0);
        lcmgl.sphere([y;0],0.02,20,20);
        lcmgl.switchBuffers();
      else
        plot3(y(1),y(2),0,'o','MarkerSize',10);
      end
    end
  end
end