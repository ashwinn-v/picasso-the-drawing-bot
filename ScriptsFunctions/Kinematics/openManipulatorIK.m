%% Waypoint tracking demonstration using Robotics System Toolbox

% This demonstration performs inverse kinematics of a
% robot manipulator to follow a desired set of waypoints.

% Copyright 2017-2019 The MathWorks, Inc.

%% Load and display robot
clear
clc

addpath(genpath(strcat(pwd,'\Dependencies')))
robot = createRigidBodyTree;
axes = show(robot);
axes.CameraPositionMode = 'auto';
showdetails(robot)

%% Create a set of desired waypoints

% waypointType = 'simple'; % or 'complex'
% switch waypointType
%     case 'simple'
%         wayPoints = [0.2 -0.2 0.02;0.25 0 0.15; 0.2 0.2 0.02]; % Alternate set of wayPoints
%         wayPointVels = [0 0 0;0 0.1 0;0 0 0];
%     case 'complex'
%         wayPoints = [0.2 -0.2 0.02;0.15 0 0.28;0.15 0.05 0.2; 0.15 0.09 0.15;0.1 0.12 0.1; 0.04 0.1 0.2;0.25 0 0.15; 0.2 0.2 0.02];
%         wayPointVels = [0 0 0; 0 0.05 0; 0 0.025 -0.025; 0 0.025 -0.025;-0.1 0 0;0.05 0 0;0 -0.1 0;0 0 0];
% end
% 
% wayPoints
% exampleHelperPlotWaypoints(0.01,wayPoints);

Im = imread('/Users/ashwinv/Documents/SEM3/Robotics/proj/proj/snip1/pic2points/Images/plot.png');

T = graythresh(Im);
ImPlot = 'true';
maxNum = 75;
M = pic2point(Im,T,ImPlot,maxNum);
M = sortrows(M,'descend');

M = scale(M);
M = nearestSort(M);



wayPoints = [zeros(size(M,1),1), M];
wayPoints(wayPoints(:,1)==0) = .2;
wayPointVels = zeros(size(M,1),3);
exampleHelperPlotWaypoints(wayPoints);



%% Create a smooth trajectory from the waypoints
numTotalPoints = size(wayPoints,1)*10;
waypointTime = 0.001;
trajType = 'cubic'; % or 'trapezoidal'
switch trajType
    case 'trapezoidal'
        trajectory = trapveltraj(wayPoints',numTotalPoints,'EndTime',waypointTime);
    case 'cubic'
        wpTimes = (0:size(wayPoints,1)-1)*waypointTime;
        trajTimes = linspace(0,wpTimes(end),numTotalPoints);
        trajectory = cubicpolytraj(wayPoints',wpTimes,trajTimes, ...
                     'VelocityBoundaryCondition',wayPointVels');
end
% Plot trajectory spline and waypoints
hold on
plot3(trajectory(1,:),trajectory(2,:),trajectory(3,:),'r-','LineWidth',2);

%% Perform Inverse Kinematics
% Use desired weights for solution (First three are orientation, last three are translation)
% Since it is a 4-DOF robot with only one revolute joint in Z we do not
% put a weight on Z rotation; otherwise it limits the solution space

ik = robotics.InverseKinematics('RigidBodyTree',robot);
weights = [0.1 0.1 0 1 1 1];
initialguess = robot.homeConfiguration;

% Call inverse kinematics solver for every end-effector position using the
% previous configuration as initial guess
for idx = 1:size(trajectory,2)
    tform = trvec2tform(trajectory(:,idx)');
    configSoln(idx,:) = ik('end_effector',tform,weights,initialguess);
    initialguess = configSoln(idx,:);
end

%% Visualize robot configurations
title('Robot waypoint tracking visualization')
axis([-0.1 0.4 -0.35 0.35 0 0.35]);
for idx = 1:size(trajectory,2)
    show(robot,configSoln(idx,:), 'PreservePlot', false,'Frames','off');
    pause(0.1)
end
hold off


%% Converte Image to Coordinates Code

function [M] = scale(M)
x = M(:,1);
y = M(:,2);
x = x./(max(x)*4);
y = y./(max(y)*4);
M = [x y];
end

function [MResult] = pic2point(Im,TshV,ImPlot,maxNum)


switch nargin
    case 1
        Pmode = false; % Not a plot
        randomPick = false;
        TshV = graythresh(Im);
    case 2
        Pmode = false; % Not a plot
        randomPick = false;
    case 3
        if strcmpi(ImPlot,'plot')
            Pmode = true; % It is a plot
        else
            Pmode = false; % It is an image
        end
        randomPick = false;
    case 4
        if strcmpi(ImPlot,'plot')
            Pmode = true; % It is a plot
        else
            Pmode = false; % It is an image
        end
        randomPick = true;
end
ImBW=im2bw(Im,TshV);
ImBW=1-ImBW; % When the plot drew by black ink on the white background
if Pmode
    ImBW = bwmorph(ImBW,'thin',Inf);
end
% figure(); imshow(ImBW);
[r,c,v] = find(ImBW==1);
r = size(ImBW,1) - r; % Correct the coordinates from the image
TMresult = [c r];
if randomPick
    Sran = linspace(1,sum(v),maxNum);
    Sran = uint32(Sran);
    %Sran = randperm(sum(v),maxNum);
    MResult = TMresult(Sran',:);
else
    MResult = TMresult;
end 
end

%% Nearest Sort

function [Mnew] = nearestSort(M)

x = M(:,1);
y = M(:,2);
numPoints = size(M,1);

newx = [];
newy =[];

set(gcf, 'Units', 'Normalized', 'OuterPosition', [0, 0.08, 1, 0.92]);

% Make a list of which points have been visited
beenVisited = false(1, numPoints);
% Make an array to store the order in which we visit the points.
visitationOrder = ones(1, numPoints);
% Define a filasafe
maxIterations = numPoints + 1;
iterationCount = 1;
% Visit each point, finding which unvisited point is closest.
% Define a current index.  currentIndex will be 1 to start and then will vary.
currentIndex = 1;
while sum(beenVisited) < numPoints && iterationCount < maxIterations
  % Indicate current point has been visited.
  visitationOrder(iterationCount) = currentIndex; 
  beenVisited(currentIndex) = true; 
  % Get the x and y of the current point.
  thisX = x(currentIndex);
  newx = [newx thisX];
  
  thisY = y(currentIndex);
  newy = [newy thisY];
  
  % Compute distances to all other points
  distances = sqrt((thisX - x) .^ 2 + (thisY - y) .^ 2);
  % Don't consider visited points by setting their distance to infinity.
  distances(beenVisited) = inf;
  % Also don't want to consider the distance of a point to itself, which is 0 and would alsoways be the minimum distances of course.
  distances(currentIndex) = inf;
  % Find the closest point.  this will be our next point.
  [minDistance, indexOfClosest] = min(distances);
  % Save this index
  iterationCount = iterationCount + 1;
  % Set the current index equal to the index of the closest point.
  currentIndex = indexOfClosest;
end
Mnew(:,1) = newx;
Mnew(:,2) = newy;
end