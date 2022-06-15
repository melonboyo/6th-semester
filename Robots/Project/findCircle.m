function [found,X,Y] = findCircle(rgbImg)
%findCircle Find the biggest green circle in an image.
%   See summary.

% Filter to reduce noise along edges
filtered = imgaussfilt(rgbImg,4);
figure(4)
imshow(filtered)

% Convert to hsv
hsvImg = rgb2hsv(filtered);

% Check the threshholds for hue, saturation and value
hueInds = hsvImg(:,:,1) >= (70 / 360) & hsvImg(:,:,1) <= (160 / 360);
satInds = hsvImg(:,:,2) >= (20 / 100);
valInds = hsvImg(:,:,3) >= (5 / 100);

% Combine and flip to get the black-white image
BW = hueInds & satInds & valInds;

% Find the connected components
CC = bwconncomp(BW);

% Filter out all components with not enough area
CCSizes = arrayfun(@(c)(length(c{1})), CC.PixelIdxList);
CCSizeValids = CCSizes > 25*25;

% Filter out all components that are not circular enough
CCCircs = [regionprops(CC, "Circularity").Circularity];
CCCircValids = abs(CCCircs - 1) < 0.15; % 0.1 above or below 1 is valid.

% Combine
CCValids = CCSizeValids & CCCircValids;

% Select the biggest circular component
% Sort after size
[~, CCSizeIdx] = sort(CCSizes, 'descend');
% Sort validity bools after size
CCBiggestSizeValids = CCSizeValids(CCSizeIdx)
CCBiggestCircValid = CCCircValids(CCSizeIdx)
CCBiggestValids = CCValids(CCSizeIdx)
% Find the first, and thereby biggest, connected component
CCBiggestValid = find(CCBiggestValids, 1);
if isempty(CCBiggestValid)
    found = false;
    X = 0;
    Y = 0;
    return
end
% Get the original index
CCBiggestValid = CCSizeIdx(CCBiggestValid);
% Get the center
CCCenters = [regionprops(CC, "Centroid").Centroid];
CCCenter = CCCenters((1:2) + (CCBiggestValid - 1) * 2);
found = true;
X = CCCenter(1);
Y = CCCenter(2);

end