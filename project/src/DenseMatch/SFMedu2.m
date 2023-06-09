clc;
disp('SFMedu: Structrue From Motion for Education Purpose');
disp('Version 2 @ 2014');
disp('Written by Jianxiong Xiao');


%% set up things(添加文件路径)
clear;
close all;
addpath(genpath('matchSIFT'));
addpath(genpath('denseMatch'));
addpath(genpath('RtToolbox'));

visualize = true;
outputMot = false;

%% data
datapath = '/face';
frames.images{1}=['images' datapath '/01.jpg'];
frames.images{2}=['images' datapath '/02.jpg'];
frames.images{3}=['images' datapath '/03.jpg'];
frames.images{4}=['images' datapath '/04.jpg'];
frames.images{5}=['images' datapath '/05.jpg'];
%frames.images{6}=['images' datapath '/06.jpeg'];
%frames.images{7}=['images' datapath '/07.jpeg'];


%% data 
frames.length = length(frames.images);

try
    % EXIF格式，提取图片对应的相机内参（focal焦距）
    frames.focal_length = extractFocalFromEXIF(frames.images{1});
catch
end
if ~isfield(frames,'focal_length') || isempty(frames.focal_length)
    fprintf('Warning: cannot find the focal length from the EXIF\n');
    frames.focal_length = 719.5459; % for testing with the B??.jpg sequences
end

% 设置图像长宽上限
maxSize = 1024;
frames.imsize = size(imread(frames.images{1}));
if max(frames.imsize)>maxSize
    scale = maxSize/max(frames.imsize);
    frames.focal_length = frames.focal_length * scale;
    frames.imsize = size(imresize(imread(frames.images{1}),scale));
end

% 内参矩阵
frames.K = diag([frames.focal_length,frames.focal_length,1]); 
disp('intrinsics:');
disp(frames.K);

%% SIFT matching and Fundamental Matrix Estimation
for frame=1:frames.length-1    
    % need to set this random seed to produce exact same result
    s = RandStream('mcg16807','Seed',10); 
    RandStream.setGlobalStream(s);
    
    % keypoint matching % 每一张图和后面一张图配对
    %pair = match2viewSURF(frames, frame, frame+1);
    pair = match2viewSIFT(frames, frame, frame+1);
    % learning-based keypoint matching 导入
        %pair = match2viewALike(frames, frame, frame+1, datapath);
    
    %对应点的信息储存在match中,例如第一列对应着对应点1在i图中的(r1,c1)与在j图中的(r2,c2)
    %if visualize, showMatches(pair,frames); title('raw feature matching'); end
    
    if true % choose between different ways of getting E
        % ToDo：这里我设置默认调用第二种方法，请思考如何实现第一种方法！！！
        % Estimate Fundamental matrix
        pair = estimateF(pair);    
        % Convert Fundamental Matrix to Essential Matrix
        % 话说第二种方法似乎和第一种的结果相差很大
        pair.E = transpose(frames.K) * pair.F * frames.K;
    else
        % Estimate Essential Matrix directly using 5-point algorithm
        pair = estimateE(pair,frames); 
    end
    
    if visualize, showMatches(pair,frames); title('inliers'); end

    % 补充RTFromE()计算出第i张图相对于第i-1张图的[R|t]
    pair.Rt = RtFromE(pair,frames); 
    
    % Convert the pair into the BA format
    Graph{frame} = pair2graph(pair,frames);
    
    % re-triangulation
    Graph{frame} = triangulate(Graph{frame},frames);
    %if visualize, visualizeGraph(Graph{frame},frames); title('triangulation'); end
    
    % outlier rejection
    % Graph{frame} = removeOutlierPts(Graph{frame});
    
    % bundle adjustment
    Graph{frame} = bundleAdjustment(Graph{frame});
    %if visualize, visualizeGraph(Graph{frame},frames); title('after two-view bundle adjustment'); end
end
%全部做完之后得到一个Graph中存放相邻两张图的对应点信息,以及相应的两个[R|t]值(放在Mot中),以及在空间中的点(放在str中)
%在obsval中存放对应点,在obsidx中存放的是对应点所在的下标

%% merge the graphs
%close all
fprintf('\n\nmerging graphs....\n');

mergedGraph = Graph{1};

for frame=2:frames.length-1
    % merge graph 此时得到的两幅图的merge,在Mot中存放共有[R|t],但同时需要修改第三个[R|t](因为采用算法保持一二不变)
    mergedGraph = merge2graphs(mergedGraph,Graph{frame});

    % re-triangulation
    mergedGraph = triangulate(mergedGraph,frames);
    if visualize, visualizeGraph(mergedGraph,frames); title('triangulation'); end

    % bundle adjustment 类似的我们虽然得到了三幅图的[R|t]值，但我们重新将得到的空间坐标转换回各自相机坐标的时候会和观测值不符,因此需要对[R|t]做拟合得到新的R|t与空间坐标
    mergedGraph = bundleAdjustment(mergedGraph);

    % outlier rejection 设置了一个参数,凡是在这个参数半径之外的对应点我们放弃掉,这只会影响结果,类似操作在estimateF中有体现
    mergedGraph = removeOutlierPts(mergedGraph, 10);

    % bundle adjustment
    mergedGraph = bundleAdjustment(mergedGraph);    

    %去掉我们不要的点之后再对所有点做拟合得到[R|t]
    if visualize, visualizeGraph(mergedGraph,frames); title('after bundle adjustment'); end
end

%% 输出相机外参数
if outputMot
for i=1:frames.length
    dlmwrite(['../output' datapath '/Mot' num2str(i) '.txt'], mergedGraph.Mot(:,:,i), 'delimiter', ',');
end
end
%% 输出稀疏点云
points2ply(['../output' datapath '/sparse.ply'], mergedGraph.Str);

if frames.focal_length ~= mergedGraph.f
    disp('Focal length is adjusted by bundle adjustment');
    frames.focal_length = mergedGraph.f;
    frames.K = f2K(frames.focal_length);
    disp(frames.K);
end


%% dense matching

fprintf('dense matching ...\n');
for frame=1:frames.length-1
    Graph{frame} = denseMatch(Graph{frame}, frames, frame, frame+1);
end
%向我们展示了denseMatch函数,直接扩展Graph中的对应点数到一个巨大的规模
%因此上面的mergeGraph过程只是为了得到五张图的[R|t]
%Graph{i}表示第i对图像之间的对应点关系

%% dense reconstruction
fprintf('triangulating dense points ...\n');
for frame=1:frames.length-1
    clear X;
    P{1} = frames.K * mergedGraph.Mot(:,:,frame);
    P{2} = frames.K * mergedGraph.Mot(:,:,frame+1);
    %par
    for j=1:size(Graph{frame}.denseMatch,2)
        X(:,j) = vgg_X_from_xP_nonlin(reshape(Graph{frame}.denseMatch(1:4,j),2,2),P,repmat([frames.imsize(2);frames.imsize(1)],1,2));
    end 
    % X是匹配点的三维齐次坐标，要先齐次化，再分别求出X在两相机拍摄下的像点位置，与匹配时的点进行比较（error）
    X = X(1:3,:) ./ X([4 4 4],:); 
    x1= P{1} * [X; ones(1,size(X,2))];
    x2= P{2} * [X; ones(1,size(X,2))];
    x1 = x1(1:2,:) ./ x1([3 3],:);
    x2 = x2(1:2,:) ./ x2([3 3],:);
    Graph{frame}.denseX = X;
    % 给出稠密重建的error
    Graph{frame}.denseRepError = sum(([x1;x2]-Graph{frame}.denseMatch(1:4,:)).^2,1); % 比较x1，x2相对选出的对应点matches的偏差
    
    Rt1 = mergedGraph.Mot(:, :, frame);
    Rt2 = mergedGraph.Mot(:, :, frame+1);
    % 计算得到相机中心，分别用上面的Rt1和Rt2；
    C1 = -Rt1(1:3,1:3)' * Rt1(:,4);%Rt1(3, 1:3)'+Rt1(1:3, 4);
    C2 = -Rt2(1:3,1:3)' * Rt2(:,4);%Rt2(3, 1:3)'+ Rt2(1:3,4); 
    view_dirs_1 = bsxfun(@minus, X, C1);
    view_dirs_2 = bsxfun(@minus, X, C2);
    view_dirs_1 = bsxfun(@times, view_dirs_1, 1 ./ sqrt(sum(view_dirs_1 .* view_dirs_1)));
    view_dirs_2 = bsxfun(@times, view_dirs_2, 1 ./ sqrt(sum(view_dirs_2 .* view_dirs_2))); 
    % 两个向量求出相机的拍摄角度cos值
    Graph{frame}.cos_angles = sum(view_dirs_1 .* view_dirs_2);
    %Graph{frame}.cos_angles(Graph{frame}.cos_angles>1) = 1;%修正一下过大值？
    
    c_dir1 = Rt1(3, 1:3)';
    c_dir2 = Rt2(3, 1:3)';
    Graph{frame}.visible = (sum(bsxfun(@times, view_dirs_1, c_dir1)) > 0) & (sum(bsxfun(@times, view_dirs_2, c_dir2)) > 0);
end

%% visualize the dense point cloud
if true%visualize
    figure
    for frame=1:frames.length-1
        hold on
        goodPoint =  Graph{frame}.denseRepError < 0.05;
        plot3(Graph{frame}.denseX(1,goodPoint),Graph{frame}.denseX(2,goodPoint),Graph{frame}.denseX(3,goodPoint),'.b','Markersize',1);
    end
    hold on
    plot3(mergedGraph.Str(1,:),mergedGraph.Str(2,:),mergedGraph.Str(3,:),'.r')
    axis equal
    title('dense cloud')
    for i=1:frames.length
        drawCamera(mergedGraph.Mot(:,:,i), frames.imsize(2), frames.imsize(1), frames.K(1,1), 0.001,i*2-1);
    end
    axis tight
end

% output as ply file to open in Meshlab 
plyPoint = [];
plyColor = [];
for frame=1:frames.length-1
    goodPoint =  (Graph{frame}.denseRepError < 0.05) & (Graph{frame}.cos_angles < cos(5 / 180 * pi)) & Graph{frame}.visible;
    X = Graph{frame}.denseX(:,goodPoint);
    % get the color of the point
    P{1} = frames.K * mergedGraph.Mot(:,:,frame);
    x1= P{1} * [X; ones(1,size(X,2))];
    x1 = round(x1(1:2,:) ./ x1([3 3],:));
    x1(1,:) = frames.imsize(2)/2 - x1(1,:);
    x1(2,:) = frames.imsize(1)/2 - x1(2,:);
    indlin = sub2ind(frames.imsize(1:2),x1(2,:),x1(1,:));
    im = imresize(imread(frames.images{frame}),frames.imsize(1:2));
    imR = im(:,:,1);
    imG = im(:,:,2);
    imB = im(:,:,3);
    colorR = imR(indlin);
    colorG = imG(indlin);
    colorB = imB(indlin);
    plyPoint = [plyPoint X];
    plyColor = [plyColor [colorR; colorG; colorB]];
end

% 输出稠密重建
points2ply(['../output' datapath '/dense.ply'],plyPoint,plyColor);
