function relativePositionTest(visualize)
  import expression.kinematic.*

  if nargin < 1, visualize = false; end

  w = warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
  rbm = RigidBodyManipulator([getDrakePath(),'/examples/Atlas/urdf/atlas_minimal_contact.urdf']);
  warning(w);
  if visualize
    lcmgl = LCMGLClient('relativePositionTest');
    v = rbm.constructVisualizer();
  end

  expr = RelativePosition(rbm,'l_hand','world',[1;1;1]);
  q = zeros(rbm.getNumPositions(),1);
  [pos,J] = expr.eval(q);

  if visualize
    lcmgl.glColor3f(1,0,0);
    lcmgl.sphere(pos,0.05,20,20);
    rbm.drawLCMGLAxes(lcmgl,q,rbm.findLinkInd('l_hand'));
    lcmgl.switchBuffers();
    v.draw(0,q);
  end

  [pos,J_taylorvar] = geval(@expr.eval,q,struct('grad_method','taylorvar'));
  [f,df] = geval(@expr.eval,q,struct('grad_method',{{'user','taylorvar'}},'tol',1e-6));
end
