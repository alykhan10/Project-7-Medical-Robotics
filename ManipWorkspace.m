% Load Robot, Kinova Gen 3 Robot Used as it Most Closely Resembles our Design.

robot = loadrobot("kinovaGen3", DataFormat="column");

% Plot Robot and Workspace in 3D.

show(robot)
[x, y, z] = sphere(50);
sphereRadius = 1.2;
hold on
p = surf(x * sphereRadius, y * sphereRadius, z * sphereRadius);
set(p, 'FaceColor', 'blue', 'FaceAlpha', 0.3, 'EdgeColor', 'none');

% Trajectory Generation Code from MATLAB Robotics Systems Toolbox.

{
time = 0:5;
tvec = time(1):1/10:time(end);
numSamples = length(tvec);
rng default
frankaWaypoints = [robot.homeConfiguration robot.homeConfiguration robot.homeConfiguration];
frankaTimepoints = linspace(tvec(1), tvec(end), 3);
[q, qd] = trapveltraj(frankaWaypoints, numSamples);
figure
set(gcf,"Visible","on");
rc = rateControl(10);
for i = 1:numSamples
   hold on
   show(robot, q(:, i), FastUpdate=true, PreservePlot=false);
   waitfor(rc);
end
}%