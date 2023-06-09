function showMatches(pair,frames)

image_i=imresize(imread(frames.images{pair.frames(1)}),frames.imsize(1:2));
image_j=imresize(imread(frames.images{pair.frames(2)}),frames.imsize(1:2));

figure
im = [image_i,image_j];
imshow(im);
hold on
plot(size(image_i,2)/2-pair.matches(1,:),size(image_i,1)/2-pair.matches(2,:),'bs', 'markersize', 3);
plot(size(image_j,2)/2-pair.matches(3,:)+size(image_i,2),size(image_j,1)/2-pair.matches(4,:),'bs', 'markersize', 3);
for match = (pair.matches)
    plot([size(image_i,2)/2- match(1) size(image_j,2)/2+size(image_i,2)-match(3)] , ...
        [size(image_i,1)/2-match(2) size(image_j,1)/2-match(4)],'Color','g','LineWidth',1)
end
%
%