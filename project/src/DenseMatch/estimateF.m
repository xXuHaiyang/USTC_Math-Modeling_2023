function pair = estimateF(pair)

t = .002;  % Distance threshold for deciding outliers
% ToDo：只需看懂ransacfitfundmatrix()函数，在这里直接调用 并且一行代码即可完成，其实最下面已经给你注释了，所以你甚至可以直接写出来
[F, inliers] = ransacfitfundmatrix(pair.matches(1:2,:),pair.matches(3:4,:),t);
fprintf('%d inliers / %d SIFT matches = %.2f%%\n', length(inliers), size(pair.matches,2), 100*length(inliers)/size(pair.matches,2));
pair.matches = pair.matches(:,inliers);
pair.F = F;
%ransac算法假设在数据点里包含错误数据与正确数据,我们将正确数据放进inliers中,将错误数据放进outliers中,以阈值t分割