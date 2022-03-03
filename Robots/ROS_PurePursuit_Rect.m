rosshutdown;
%%
master_ip = '192.168.87.186'; % Find this with hostname -I in your VM
host_ip = '192.168.87.146'; % Use ipconfig on windows or something

ROS_MASTER_URI = strcat('http://', master_ip, ':11311');
setenv('ROS_MASTER_URI',ROS_MASTER_URI);
setenv('ROS_IP',host_ip);
rosinit(ROS_MASTER_URI,'NodeHost',host_ip);
%%
pp = controllerPurePursuit;
global pos;
global orient;
path = [0.00    0.00;
        2.00    0.00;
        2.00    5.00;
        0.00    5.00];
robotInitialLocation = path(1,:);
robotGoal = path(end,:);
initialOrientation = 0;

robotCurrentPose = [robotInitialLocation initialOrientation]';

pp.Waypoints = path;
pp.DesiredLinearVelocity = 0.2;
pp.MaxAngularVelocity = 2;
pp.LookaheadDistance = 0.3;
   
robotodom = rossubscriber('/odom');
robotvel = rospublisher('/mobile_base/commands/velocity');
%%
% Load occupancy map
img = imread("shannon.sdf(-1590,-728).png");
grayImg = rgb2gray(img);
bwImg = grayImg < 0.5;
xoffset = -1590;
yoffset = -728;
resolution = 50; % 100 pixels per meter

map = binaryOccupancyMap(bwImg, resolution);
map.GridOriginInLocal = [xoffset, yoffset] / resolution;
%%
pause(2);
%%
time = 0.0;
delta_time = 0.5;
sim_time = 5.0;
running = true;

while(running)
    % Get pose from odometry subscriber
    odom_msg = receive(robotodom);
    vel_msg = rosmessage(robotvel);
    
    pos = [odom_msg.Pose.Pose.Position.X
           odom_msg.Pose.Pose.Position.Y
           odom_msg.Pose.Pose.Position.Z];
    orient = odom_msg.Pose.Pose.Orientation.Z;
    fprintf('pos = %d, %d, %d\n', pos(1), pos(2), pos(3));
    fprintf('orient = %d\n', orient);
    
    vel_msg.Linear.X = -0.2;
    vel_msg.Angular.Z = 0.3;
    send(robotvel,vel_msg);
    
    % Compute the controller outputs, i.e., the inputs to the robot
    % [v, omega] = controller(robotCurrentPose);
    
    pause(delta_time);
    time = time + delta_time;
    if(time >= sim_time)
        running = false;
    end
end