clear

%% Drive with robot
figure(1)
axis([-100 100 -200 200])
hold all
initpose = [0 0 0]'; % [x y angle]
% rob = ExampleHelperDifferentialDriveRobot(initpose);
rob = differentialDriveKinematics;

v = 20; % linear velocity (m/s)
w = 1; % angular velocity (rad/s)
sampleTime = 0.1;
pose = initpose;

for i=1:200
    vel = derivative(rob, pose, [v,w]);
    pose = pose + vel*sampleTime;
    
    plot(pose(1), pose(2), '-');
    
    w = w - 0.1;
end
w = 1;
hold off
%% Plot x-direction position, velocity, acceleration

% position
x = rob.Trajectory(:,1);
% velocity
vx = (x(2:end)-x(1:end-1)) / dt;
% acceleration
ax = (vx(2:end)-vx(1:end-1)) / dt;

figure()
subplot(311), plot(x), axis tight
xlabel('Time (s)'), ylabel('X-position (m)')
subplot(312), plot(vx)
xlabel('Time (s)'), ylabel('X-velocity (m/s)')
subplot(313), plot(ax)
xlabel('Time (s)'), ylabel('X-acceleration (m/s{^2})')


