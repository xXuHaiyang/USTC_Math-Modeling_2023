function Rtbest=RtFromE(pair,frames)

% Decompose Essential Matrix
[R1, R2, t1, t2] = PoseEMat(pair.E); % MVG Page 257-259
%首先对E矩阵做SVD分解并直接做讲义中的矩阵变换UWV'等,t1=U(:,3),t2=U(:,-3)
% Four possible solution about [R|t]
Rt(:,:,1) =[R1 t1];
Rt(:,:,2) =[R1 t2];
Rt(:,:,3) =[R2 t1];
Rt(:,:,4) =[R2 t2];
% ToDo：triangulation 相当于我们将第一个矩阵[R1|t1]=[I|0],则P{1}=K[I|0] (matlab的话 一行代码可能就行)
P{1} = [frames.K, zeros(3,1)];

goodCnt = zeros(1,4);
for i=1:4 % ToDo:计算四种可能，你需要读懂vgg_X_from_xP_nonlin()函数（在相应的.m文件下），PS:如果你够仔细和机智，你会发现有个地方类似的用到了这个函数!!!
    clear X;
    % first:先得到的是相对1的K[R2|t2]
    P{2} = frames.K * Rt(:,:,i); 
    % second:应用vgg_X_from_xP_nonlin()计算
    %X = vgg_X_from_xP_nonlin(reshape(pair.matches(1:4,:),2,2),P,repmat([frames.imsize(2);frames.imsize(1)],1,2));%有待验证imsize给的是否正确
    % 得到四维（齐次）坐标X
    for j=1:size(pair.matches,2)
        X(:,j) = vgg_X_from_xP_nonlin(reshape(pair.matches(1:4,j),2,2),P,repmat([frames.imsize(2);frames.imsize(1)],1,2));
    end 
    X = X(1:3,:) ./ X([4 4 4],:); % 转化为三维坐标
    % third:选择（这步我已经给出了，不需要你们写，你们可以根据这句话思考前面second和下一步怎么写）
    dprd = Rt(3,1:3,i) * ((X(:,:) - repmat(Rt(1:3,4,i),1,size(X,2))));%最后选择的规则即为满足(X-C)*R(3,:)是否大于0
    % forth:计算了满足z>0以及(X-C)*R(3,:)>0的个数
    goodCnt(i) = size(dprd(dprd>0 & X(3,:)>0),2);
end

% pick one solution from the four
fprintf('%d\t%d\t%d\t%d\n',goodCnt);
[~, bestIndex]=max(goodCnt);

Rtbest = Rt(:,:,bestIndex);
end
