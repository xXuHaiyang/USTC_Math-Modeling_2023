function graph=pair2graph(pair,frames)

graph = pair;

pointCount = size(pair.matches,2);

graph.f   = frames.focal_length;

graph.Mot(:,:,1) = [eye(3) [0;0;0]];
graph.Mot(:,:,2) = pair.Rt;%存放[R1|t1]与[R2|t2]

graph.Str   = zeros(3,pointCount);

graph.ObsVal = [pair.matches(1:2,:) pair.matches(3:4,:)];%将对应点拉成在一行内,i列对应(i+pointcount)列

graph.ObsIdx = sparse([1:pointCount; pointCount + (1:pointCount)]);

