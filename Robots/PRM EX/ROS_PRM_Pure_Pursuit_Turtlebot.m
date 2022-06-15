rosshutdown;
clc;
clear;
%%
master_ip = '192.168.87.186'; % IP of the host on the turtlebot (hostname -I in VM)
host_ip = '192.168.87.146'; % IP of host running this script (ipconfig on Windows)
ROS_MASTER_URI = strcat('http://', master_ip, ':11311');

% Connect to turtlebot
setenv('ROS_MASTER_URI',ROS_MASTER_URI);
setenv('ROS_IP',host_ip);
rosinit(ROS_MASTER_URI,'NodeHost',host_ip);
%%
% Load binary occupancy map
img = imread("shannon_hallway.png");
grayImg = rgb2gray(img);    % Convert to gray scale
bwImg = grayImg < 0.5;      % Convert to black and white image

% Offset the map so origo is moved to where we want it
xOffset = -30;
yOffset = -30;
resolution = 50; % 50 pixels per meter

% Create binary occupancy map
map = binaryOccupancyMap(bwImg, resolution);
map.GridOriginInLocal = [xOffset, yOffset] / resolution;
%%
% Inflate map to compensate for robot size (radius of ~0.18m)
inflate(map, 0.25);
%%
% Make Probabilistic Road Map with 1000 points
prmMap = mobileRobotPRM(map, 1000);

% Define start and goal points
startPoint = [0 0];
goal = [5.7 3.87];

% Use find path function on the PRM map
path = findpath(prmMap,startPoint,goal);
show(prmMap);
%%
ppCtl = controllerPurePursuit;
global pose;

% Set Pure Pursuit controller parameters
ppCtl.Waypoints = path;
ppCtl.DesiredLinearVelocity = 0.25;
ppCtl.MaxAngularVelocity = 2;
ppCtl.LookaheadDistance = 0.3;

% Get ROS odometry subscriber and velocity publisher
robotOdomSub = rossubscriber('/odom');
robotVelPub = rospublisher('/mobile_base/commands/velocity');
velMsg = rosmessage(robotVelPub);
%%
% Reset odometry of the robot
robotResetOdomPub = rospublisher('/mobile_base/commands/reset_odometry');
send(robotResetOdomPub, rosmessage(robotResetOdomPub));
%%
pause(2);
%%
timer = rateControl(60); % 60 Hz timer for the loop
goalReachedDist = 0.1;
running = true;
%%
% Execution loop; loop time determined by rate controler timer
while(running)
    % Get pose from odometry subscriber
    odomMsg = receive(robotOdomSub);
    
    % Convert pose orientation to Euler angles
    pquat = odomMsg.Pose.Pose.Orientation;
    quat = [pquat.W, pquat.X, pquat.Y, pquat.Z];
    angles = quat2eul(quat);
    
    % Save current pose to global variable
    pose = [odomMsg.Pose.Pose.Position.X
            odomMsg.Pose.Pose.Position.Y
            angles(1)];
    fprintf('pos = %d, %d\n', pose(1), pose(2));
    fprintf('orient = %d\n', pose(3));
    
    % Use the Pure Pursuit controller to get linear and angular velocities
    [v, omega] = ppCtl(pose);
    velMsg.Linear.X = v;
    velMsg.Angular.Z = omega;
    
    % Use the publisher to send the velocities
    send(robotVelPub,velMsg);
    
    waitfor(timer);
    
    % Check if goal is reached
    distToGoal = norm(goal - pose(1:2)');
    if(distToGoal < goalReachedDist)
        fprintf('Goal reached!');
        running = false;
    end
end