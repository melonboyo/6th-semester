%% Initialization
clc; clear; close all;
rosshutdown;
%%
master_ip = '10.42.0.1'; % IP of the host on the turtlebot (hostname -I in VM)
host_ip = '10.42.0.245'; % IP of host running this script (ipconfig on Windows)
ROS_MASTER_URI = strcat('http://', master_ip, ':11311');

% Connect to turtlebot
setenv('ROS_MASTER_URI',ROS_MASTER_URI);
setenv('ROS_IP',host_ip);
rosinit(ROS_MASTER_URI,'NodeHost',host_ip);
%%
global pose;
pose = [0, 0, 0];

%% Pure pursuit controller
ppcont = controllerPurePursuit;
ppcont.DesiredLinearVelocity = 0.25;
ppcont.MaxAngularVelocity = 2;
ppcont.LookaheadDistance = 0.5;

%%
robotsteering = rospublisher('/mobile_base/commands/velocity');
velmsg = rosmessage(robotsteering);
resetodom = rospublisher('/mobile_base/commands/reset_odometry');
resetcmd = rosmessage(resetodom);
odom = rossubscriber('/odom');
scanSub = rossubscriber('/scan');
%% Reset odom
send(resetodom, resetcmd);
%% Scan 

timer = rateControl(60);
wallTooClose = true;

scanMsg = receive(scanSub);
send(resetodom, resetcmd);
points = readCartesian(scanMsg);
N = 500;
[pLines, angles, clampedDists, dists] = getWalls(points,N);

while true
      
    scanMsg = receive(scanSub);
    odomdata = receive(odom);
    [X, Y, angle] = odom2pose(odomdata);
    pose = [X Y angle];
    points = readCartesian(scanMsg);
    [pLines, angles, clampedDists, dists] = getWalls(points,N);
    
    while ~isempty(clampedDists) && (clampedDists(1) < 2.5) && (dists(1) < 0.7)
        
        wallAngle = angles(1)
        wallDist = dists(1)

        % Generate waypoints from paramtric line
        line = pLines(:,:,1);
        % Transform to world coordinates (using pose information)
        % Create rotation matrix
        R = [cos(angle) -sin(angle); sin(angle) cos(angle)];
        % Rotate start point and direction vector of the parametrix line
        line(:,1) = R * pLines(:,1,1) + [X; Y];
        line(:,2) = R * pLines(:,2,1);

        % Check which direction of the normal vector points towards the
        % robot
        [t, d] = closestT(line, [X Y]);
        n = [-line(2,2) line(1,2)];
        if d > 0
            n = [line(1,2) -line(2,2)];
        end
        n = n / norm(n);

        % Find waypoints for the pure pursuit controller
        p0 = evalLine(line, 0) + (n * 0.5);
        p1 = evalLine(line, 1) + (n * 0.5);
        waypoints = [p0; p1];


        % Plot l
%         figure(3);
%         clf(3);
%         hold on;
%         plot([line(1,1) p0(1)],[line(2,1) p0(2)], 'LineWidth',2 , "Color", "red");
%         plot(waypoints(:,1), waypoints(:,2), 'LineWidth',2 ,'Color','black');
%         hold off;
%         worldPoints = cell2mat(arrayfun(@(x,y){((R * [x y]')') + [X Y]}, points(:,1), points(:,2)));
%         hold on;
%         plot(worldPoints(:,1), worldPoints(:,2), "*", "Color", "blue");
%         plot(X,Y,"*", "Color", "green");
%         plot([X (X + cos(angle) * 0.25)], [Y (Y + sin(angle) * 0.25)], 'LineWidth',2 ,'Color','green');
%         hold off;
%         xlim([-4 4]);
%         ylim([-4 4]);

        % Set the waypoints
        ppcont.Waypoints = waypoints;

        % Use controller
        [speed, omega] = ppcont(pose);
        velmsg.Linear.X = speed;
        velmsg.Angular.Z = omega;
        send(robotsteering, velmsg);

        % Wait
        waitfor(timer);

        % New scan
        scanMsg = receive(scanSub);
        odomdata = receive(odom);
        [X, Y, angle] = odom2pose(odomdata);
        pose = [X Y angle];
        points = readCartesian(scanMsg);
        [pLines, angles, clampedDists, dists] = getWalls(points,N);
    end
    

    velmsg.Linear.X = 0;
    velmsg.Angular.Z = 0;
    send(robotsteering, velmsg);
    
    waitfor(timer);
end