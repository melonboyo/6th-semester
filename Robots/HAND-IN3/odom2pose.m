function [X,Y,angle] = odom2pose(odomdata)
%odom2pose Extracts X and Y coordinates from odom and pose angle in rad
pose = odomdata.Pose.Pose;
X = pose.Position.X;
Y = pose.Position.Y;
eulAngles = quat2eul([pose.Orientation.W pose.Orientation.X pose.Orientation.Y pose.Orientation.Z]);
angle = eulAngles(1);
end

