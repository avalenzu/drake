function testRelativeFootPositionPolygon
vertices = randn(2,7);
rfpp = RelativeFootPositionPolygon(vertices);

num_posB = 10000;
w = rand(num_posB,rfpp.num_vertices);
w = w./bsxfun(@times,sum(w,2),ones(1,rfpp.num_vertices));

yaw = rand()*2*pi;
posA = randn(2,1);
pos_vert = bsxfun(@times,ones(1,rfpp.num_vertices),posA)+[cos(yaw) -sin(yaw);sin(yaw) cos(yaw)]*rfpp.vertices;
posB = pos_vert*w';
[A,b] = rfpp.halfspace(yaw);
val = A*[bsxfun(@times,posA,ones(1,num_posB));posB]-bsxfun(@times,b,ones(1,num_posB));
if(any(any(val>1e-10)))
  error('Incorrect RelativeFootPositionPolygon');
end
rfpp.checkGradient(1e-3,randn(5,1));
end