function [steeringDir] = findWayAroundObstacle(scan,targetDir)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
sensorOffset = 0.09; %9cm offset from center
vfhCtrl = controllerVFH;
vfhCtrl.UseLidarScan = true;
vfhCtrl.RobotRadius = 0.17+sensorOffset; %17cm radius
vfhCtrl.SafetyDistance = 0.37+sensorOffset; 
vfhCtrl.MinTurningRadius = 0.1;
vfhCtrl.DistanceLimits = [0.17+sensorOffset 3+sensorOffset];
steeringDir = vfhCtrl(scan, targetDir);
end

