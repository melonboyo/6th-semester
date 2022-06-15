function [pLines, angles, clampedDists, dists] = getWalls(points,N)
    if (size(points,1) < 10)    % Make sure lines exist, so 0 division when scaling
        pLines = zeros(2,2,0);
        angles = zeros(1,0);
        clampedDists = zeros(1,0);
        dists = zeros(1,0);
        return
    end
    
    xs = points(:,1);
    ys = points(:,2);

    xoffset = min(xs);
    yoffset = min(ys);

    xscale = max(xs) - min(xs);
    yscale = max(ys) - min(ys);
    scale = max(xscale, yscale);
    resolution = scale/N;

    xi = round((N - 1) * ((xs - xoffset) / scale)) + 1;
    yi = round((N - 1) * ((ys - yoffset) / scale)) + 1;
    inds = sub2ind([N N], yi, xi);

    mat = zeros(N);
    mat(inds) = 1;

    % Hough transform
    [H,theta,rho] = hough(mat);

    % Hough peaks
    minDegreesDiff = 15;
    minDistDiff = 0.3;
    minPointsPerLine = 15;

    minDegreesDiscrete = minDegreesDiff * 2 + 1;
    minDistDiscrete = 2 * floor(min(minDistDiff, scale) / resolution) + 1;
    Nlines = 5; % Note - Nlines peaks/lines !
    P  = houghpeaks(H,Nlines, Threshold = minPointsPerLine, Nhoodsize = [minDistDiscrete minDegreesDiscrete]);
    if (length(P) < 1)    % Make sure peaks exist
        pLines = zeros(2,2,0);
        angles = zeros(1,0);
        clampedDists = zeros(1,0);
        dists = zeros(1,0);
        return
    end

    % Hough lines
    lines = houghlines(mat,theta,rho,P,'FillGap',1/resolution,'MinLength',0.5/resolution);
    if (length(lines) < 1)    % Make sure lines exist
        pLines = zeros(2,2,0);
        angles = zeros(1,0);
        clampedDists = zeros(1,0);
        dists = zeros(1,0);
        return
    end

%     figure(1);
%     imshow(H,[],'XData',theta,'YData',rho,'InitialMagnification','fit');
%     xlabel('\theta'), ylabel('\rho');
%     axis on, axis normal, hold on;
%     plot(theta(P(:,2)),rho(P(:,1)),'s','color','white');
%     hold off;

    shapedLines = reshape([lines.point1; lines.point2], [2 2 length(lines)]);
    relLines = (shapedLines*resolution)+[xoffset yoffset];

    permutedRelLines = permute(relLines,[2 1 3]);

    sub = zeros(2,2,size(permutedRelLines,3));
    sub(:,2,:) = permutedRelLines(:,1,:);
    pLines = permutedRelLines - sub;

    pxs = reshape(pLines(1,1,:), [1 size(pLines,3)]);
    pys = reshape(pLines(2,1,:), [1 size(pLines,3)]);
    rxs = reshape(pLines(1,2,:), [1 size(pLines,3)]);
    rys = reshape(pLines(2,2,:), [1 size(pLines,3)]);

    distTs = cell2mat(arrayfun(@(px,py,rx,ry){closestT([px rx; py ry],[0;0])}, pxs, pys, rxs, rys));
    clampedDistTs = cell2mat(arrayfun(@(T){min(1,max(0,T))},distTs));

    angles1 = abs(atan2(rys,rxs));
    angles2 = abs(atan2(-rys,-rxs));

    %         angDeg1 = rad2deg(angles1)
    %         angDeg2 = rad2deg(angles2)

    angles = cell2mat(arrayfun(@(a1,a2){min(a1,a2)},angles1,angles2));

    clampedDists = cell2mat(arrayfun(@(px,py,rx,ry,T){norm(evalLine([px rx; py ry],T))}, pxs, pys, rxs, rys, clampedDistTs));
    dists = cell2mat(arrayfun(@(px,py,rx,ry,T){norm(evalLine([px rx; py ry],T))}, pxs, pys, rxs, rys, distTs));

    [clampedDists, inds] = sort(clampedDists);
    angles = angles(inds);
    dists = dists(inds);
    pLines = pLines(:,:,inds);

%     figure(2);
%     plot(xs, ys, ".");
%     xlim([0 8]);
%     ylim([-4 4]);
%     hold on;
%     for k = 1:length(lines)
%         xy = [relLines(:,1,k) relLines(:,2,k)];
%         plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');
%         % Plot beginnings and ends of lines
%         plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
%         plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');
%     end
%     hold off

end

