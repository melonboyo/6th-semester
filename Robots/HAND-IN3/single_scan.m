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
odom = rossubscriber('/odom');
scanSub = rossubscriber('/scan');
%% Scan 

while true
    scanMsg = receive(scanSub);
    points = readCartesian(scanMsg);
    N = 100;
    [pLines, angles, clampedDists, dists] = getWalls(points,N);
    
    angle = rad2deg(angles)
    segDist = clampedDists
    lineDist = dists

    pause(1);
end
