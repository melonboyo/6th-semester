
%% Initialization
clc; clear; close all;
rosshutdown;

master_ip = '10.42.0.1'; % IP of the host on the turtlebot (hostname -I in VM)
host_ip = '10.42.0.245'; % IP of host running this script (ipconfig on Windows)
ROS_MASTER_URI = strcat('http://', master_ip, ':11311');

% Connect to turtlebot
setenv('ROS_MASTER_URI',ROS_MASTER_URI);
setenv('ROS_IP',host_ip);
rosinit(ROS_MASTER_URI,'NodeHost',host_ip);

%% Prepare image message subscriber
imageSub = rossubscriber("/camera/rgb/image_raw");

%% Scan 500 times

for i=1:500
    % Load image and convert to HSV
    rgbImg = readImage(receive(imageSub));
    [found, X, Y] = findCircle(rgbImg);
    
    imshow(rgbImg)
    
    if found
        hold on;
        plot(X, Y, "*");
        hold off;
    end

    pause(0.25)
end