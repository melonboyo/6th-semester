%% Initialization
clc; clear; close all;
rosshutdown;

master_ip = '10.42.0.1'; % IP of the host on the turtlebot (hostname -I in VM)
host_ip = '10.42.0.82'; % IP of host running this script (ipconfig on Windows)
ROS_MASTER_URI = strcat('http://', master_ip, ':11311');

% Connect to turtlebot
setenv('ROS_MASTER_URI',ROS_MASTER_URI);
setenv('ROS_IP',host_ip);
rosinit(ROS_MASTER_URI,'NodeHost',host_ip);

%% ROS
robotsteering = rospublisher('/mobile_base/commands/velocity');
velmsg = rosmessage(robotsteering);
resetodom = rospublisher('/mobile_base/commands/reset_odometry');
resetcmd = rosmessage(resetodom);
odom = rossubscriber('/odom');
scanSub = rossubscriber('/scan');
global pose;

%%
robotsteering = rospublisher('/mobile_base/commands/velocity');
velmsg = rosmessage(robotsteering);
resetodom = rospublisher('/mobile_base/commands/reset_odometry');
resetcmd = rosmessage(resetodom);
odom = rossubscriber('/odom');
scanSub = rossubscriber('/scan');
global pose;
%% Main loop
timer = rateControl(1000);
goalReachedDist = 0.5;
running = true;

while(running)
    % Get pose from odometry subscriber
    targetDir = 0; %Radians
    scanMsg = receive(scanSub);
    odomdata = receive(odom);
    scan = lidarScan(scanMsg);
    steeringDir = findWayAroundObstacle(scan, targetDir)
    steeringDirDeg = steeringDir*180/pi
    waitfor(timer);
end