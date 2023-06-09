function graph=triangulate(graph,frames)

nPts = size(graph.Str,2);

X = zeros(4,nPts);

for i=1:nPts
    
    validCamera = find(full(graph.ObsIdx(:,i)~=0))';%首先将obsidx中的第i列元素取出变为稀疏矩阵,再寻找不为0的元素下标,这是一个找相机个数的过程
    %在SIFT中只有两个相机有对应,但是在mergegraph操作中会存在对应点出现在多张图中
    P=cell (1,length(validCamera));
    x=zeros(2,length(validCamera));
    cnt = 0;
    
    for c=validCamera
        cnt = cnt + 1;
        % x (2-by-K matrix)
        x(:,cnt) = graph.ObsVal(:,graph.ObsIdx(c,i));
        
        % P (K-cell of 3-by-4 matrices)
        P{cnt} = f2K(graph.f) * graph.Mot(:,:,c);
    end  %没什么特别的,还是取出对应点之后计算了K[R1|t1],K[R2|t2]
    
    X(:,i) = vgg_X_from_xP_nonlin(x,P,repmat([frames.imsize(2);frames.imsize(1)],1,length(P)));
    
end

%X(isnan(X(:)))=1;

graph.Str = X(1:3,:) ./ X([4 4 4],:);  %事实上整个triangulate操作都在RtfromE中有了,这一步只是重复了一遍