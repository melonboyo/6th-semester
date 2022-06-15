
%% Initialization
clc; clear; close all;
rosshutdown;

master_ip = '10.42.0.1'; % IP of the host on the turtlebot (hostname -I in VM)
host_ip = '10.42.0.95'; % IP of host ru nning this script (ipconfig on Windows)
ROS_MASTER_URI = strcat('http://', master_ip, ':11311');

% Connect to turtlebot
setenv('ROS_MASTER_URI',ROS_MASTER_URI);
setenv('ROS_IP',host_ip);
rosinit(ROS_MASTER_URI,'NodeHost',host_ip);

%% Pure pursuit controller
ppcont = controllerPurePursuit;
maxLVel = 0.5;
maxAVel = 1.3;
ppcont.DesiredLinearVelocity = maxLVel;
ppcont.MaxAngularVelocity = maxAVel;
ppcont.LookaheadDistance = 0.5;

%% VFH controller
vfhcont = controllerVFH;
vfhcont.UseLidarScan = true;
vfhcont.RobotRadius = 0.17;
vfhcont.SafetyDistance = 1;
vfhcont.MinTurningRadius = 0.4;
vfhcont.CurrentDirectionWeight = 0.7;
vfhcont.PreviousDirectionWeight = 0.7;
% vfhcont.TargetDirectionWeight = 
sensorOffset = 0.09;
vfhcont.DistanceLimits = [(0.17 + sensorOffset) (0.7 + sensorOffset)];

%%
robotsteering = rospublisher('/mobile_base/commands/velocity');
velmsg = rosmessage(robotsteering);
resetodom = rospublisher('/mobile_base/commands/reset_odometry');
resetcmd = rosmessage(resetodom);
odom = rossubscriber('/odom');
scanSub = rossubscriber('/scan');
imageSub = rossubscriber("/camera/rgb/image_raw");
soundPub = rospublisher("/mobile_base/commands/sound");
soundMsg = rosmessage(soundPub);
soundMsg.Value = 5;
global pose;

%% Load occupancy map
img = imread("shannon.sdf(-1590,-728).png");
grayImg = rgb2gray(img);
bwImg = grayImg < 0.5;
xoffset = -2345;
yoffset = -1413;
resolution = 50; % 50 pixels per meter

map = binaryOccupancyMap(bwImg, resolution);
map.GridOriginInLocal = [xoffset, yoffset] / resolution;

% Inflate map to compensate for robot size (radius of 0.3m)
inflatedMap = copy(map);
inflate(inflatedMap, 0.7);

% Make Probabilistic Road Map with 3000 points
prmMap = mobileRobotPRM(inflatedMap, 3200);
prmMap.ConnectionDistance = 3.5;

% Define start and goal points
goals = [-14.5 -4.6; -0.2 -2];

%% Reset odom
send(resetodom, resetcmd);
pose = [-0.8, -4.4, 0];

%% Setup odometry and range scan models
odometryModel = odometryMotionModel;
odometryModel.Noise = [0.5 0.5 0.2 0.2];

rangeFinderModel = likelihoodFieldSensorModel;
rangeFinderModel.SensorLimits = [0.45 8];
rangeFinderModel.Map = map;

%% Setup monte carlo
amcl = monteCarloLocalization;
amcl.UseLidarScan = true;
amcl.MotionModel = odometryModel;
amcl.SensorModel = rangeFinderModel;
amcl.UpdateThresholds = [0.01,0.01,0.01];
amcl.ResamplingInterval = 1;

amcl.GlobalLocalization = false;
amcl.ParticleLimits = [200 200];
amcl.InitialPose = pose;
amcl.InitialCovariance = 0.2;

visualizationHelper = ExampleHelperAMCLVisualization(map);

%% Main loop
timer = rateControl(30);
goalReachedDist = 0.7;

% Initialize monte carlo localization by spinning a bit
for i=1:250
    % Get pose from odometry subscriber
    scanMsg = receive(scanSub);
    odomdata = receive(odom);
    scan = lidarScan(scanMsg);
    [X, Y, angle] = odom2pose(odomdata);
    odomPose = [X Y angle];
    [isUpdated, pose, estimatedCovariance] = amcl(odomPose, scan);
    
    figure(1);
    plotStep(visualizationHelper, amcl, pose, scan, i);

    velmsg.Linear.X = 0;
    velmsg.Angular.Z = 0.5;
    
    % Use the publisher to send the velocities
    send(robotsteering,velmsg);
    
    waitfor(timer);
end

slowdown = 0;
for j = 1:size(goals, 1)
    goal = goals(j,:);
    % Use find path function on the PRM map
    path = findpath(prmMap,pose(1:2),goal);
    figure(2);
    show(prmMap);
    ppcont.Waypoints = path;
    pause(3);
    
    running = true;
    i = 0;
    % Execution loop; loop time determined by rate controler timer
    while(running)
        % Get pose from odometry subscriber
        scanMsg = receive(scanSub);
        odomdata = receive(odom);
        scan = lidarScan(scanMsg);
        [X, Y, angle] = odom2pose(odomdata);
        odomPose = [X Y angle];
        [isUpdated, pose, estimatedCovariance] = amcl(odomPose, scan);
        
        figure(1);
        plotStep(visualizationHelper, amcl, pose, scan, i);
        i = i + 1;
        
        % Check if goal is reached
        distToGoal = norm(goal - pose(1:2));
        if(distToGoal < goalReachedDist)
            fprintf('Goal reached!');
            running = false;
            break;
        end
    
        % Use the Pure Pursuit controller to get linear and angular velocities
        [v, ~, lookaheadPoint] = ppcont(pose);
        
        lpr = lookaheadPoint - pose(1:2);
        targetDir = angdiff(pose(3), atan2(lpr(2), lpr(1)));
        steeringDir = vfhcont(scan, targetDir)
        if isnan(steeringDir)
            v = 0;
            omega = 0.5;
        else
            omega = 2*sin(steeringDir) / ppcont.LookaheadDistance;
            if abs(omega) > ppcont.MaxAngularVelocity
                omega = sign(omega) * ppcont.MaxAngularVelocity;
            end
        end

        velmsg.Linear.X = v;
        velmsg.Angular.Z = omega;

        % Use the publisher to send the velocities
        send(robotsteering,velmsg);
        
        waitfor(timer);
        
    end

    % Do scan for the green circle
    for i=1:500
        imgMsg = receive(imageSub);
        rgbImg = readImage(imgMsg);
        [foundCircle, CX, CY] = findCircle(rgbImg);
        if foundCircle & (abs(CX - double(imgMsg.Width/2)) < (0.01 * imgMsg.Width))
            break;
        end
        
        % Turn
        velmsg.Linear.X = 0;
        velmsg.Angular.Z = 0.2;
        send(robotsteering, velmsg);
        scanMsg = receive(scanSub);
        odomdata = receive(odom);
        scan = lidarScan(scanMsg);
        [X, Y, angle] = odom2pose(odomdata);
        odomPose = [X Y angle];
        [isUpdated, pose, estimatedCovariance] = amcl(odomPose, scan);
        waitfor(timer);
    end
    velmsg.Linear.X = 0;
    velmsg.Angular.Z = 0;
    send(robotsteering, velmsg);
    
    if ~foundCircle
        fprintf("Circle not found!")
        continue;
    end
    
    % Find wall
    scanMsg = receive(scanSub);
    points = readCartesian(scanMsg);
    [pLines, angles, clampedDists, dists] = getWalls(points, 20);
    
    while dists(1) > 0.7
        % Find wall
        scanMsg = receive(scanSub);
        points = readCartesian(scanMsg);
        [pLines, angles, clampedDists, dists] = getWalls(points, 20);
        
        velmsg.Linear.X = 0.1;
        velmsg.Angular.Z = 0;
        send(robotsteering, velmsg);
        waitfor(timer);
    end
    velmsg.Linear.X = 0;
    velmsg.Angular.Z = 0;
    send(robotsteering, velmsg);
    send(soundPub, soundMsg)
%     
%     % Turn towards circle
%     for i=1:500
%         imgMsg = receive(imageSub);
%         rgbImg = readImage(imgMsg);
%         [foundCircle, CX, CY] = findCircle(rgbImg);
%         if foundCircle & (abs(CX - double(imgMsg.Width/2)) < (0.01 * imgMsg.Width))
%             break;
%         end
%         
%         % Turn
%         velmsg.Linear.X = 0;
%         velmsg.Angular.Z = 0.5;
%         send(robotsteering, velmsg);
%         scanMsg = receive(scanSub);
%         odomdata = receive(odom);
%         scan = lidarScan(scanMsg);
%         [X, Y, angle] = odom2pose(odomdata);
%         odomPose = [X Y angle];
%         [isUpdated, pose, estimatedCovariance] = amcl(odomPose, scan);
%         waitfor(timer);
%     end
%     velmsg.Linear.X = 0;
%     velmsg.Angular.Z = 0;
%     send(robotsteering, velmsg);
end

