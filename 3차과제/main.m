img_rgb = imread('문제1.png');

img_hsv = rgb2hsv(img_rgb);

img_hsv_h = img_hsv(:,:,1);
img_hsv_s = img_hsv(:,:,2);
img_hsv_v = img_hsv(:,:,3); 

img_hsv_green = double(zeros(size(img_hsv_h))); 

for i = 1: size(img_hsv_green, 1)

    for j = 1:size(img_hsv_green, 2)

        if (img_hsv_h(i, j) > 0.3 && img_hsv_h(i, j) < 0.4) && (img_hsv_v(i, j) < 0.97) && (img_hsv_s(i,j) > 0.4)
             img_hsv_green(i, j) = 1;

        end

    end

end

img_rgb_green = uint8(zeros([size(img_hsv_green), 3]));

for i = 1:size(img_hsv_green, 1)

    for j = 1:size(img_hsv_green, 2)

        if img_hsv_green(i, j) ~= 0

            img_rgb_green(i, j, :) = img_rgb(i, j, :);

        end

    end

end


I = img_rgb_green;
G = rgb2gray(I);
BW = imbinarize(G);

BW2 = imcomplement(BW);
BW3 = bwpropfilt(BW2,'perimeter',2);



CC = bwconncomp(BW3);
L = labelmatrix(CC);
numObjects = max(L(:));
RGB = label2rgb(L,'gray','k','noshuffle');
GR=rgb2gray(RGB);
BW4 = imbinarize(GR);

imshow(BW4); axis on; hold on;

[y, x] = ndgrid(1:size(BW4, 1), 1:size(BW4, 2));
centroid = mean([x(logical(BW4)), y(logical(BW4))]);
plot(centroid(1), centroid(2),'b+', 'LineWidth', 3);
