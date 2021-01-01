
clc;    % Clear the command window.
close all;  % Close all figures (except those of imtool.)
clear;  % Erase all existing variables. Or clearvars if you want.
workspace;  % Make sure the workspace panel is showing.
format long g;
format compact;
fontSize = 20;
% Initial setup
numPoints = 10;
x = rand(1, numPoints);
y = rand(1, numPoints);

%%
plot(x, y, 'bo', 'LineWidth', 2, 'MarkerSize', 17);
grid on;
% Make axis square so we don't get deceived on distances in x and y directions.
axis square;
xlabel('X', 'FontSize', fontSize);
ylabel('Y', 'FontSize', fontSize);
% Enlarge figure to full screen.
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0, 0.08, 1, 0.92]);
% Label ths points
for k = 1 : numPoints
  text(x(k), y(k), num2str(k), 'FontSize', 15, 'Color', 'b');
end
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
  thisY = y(currentIndex);
  text(thisX + 0.01, thisY, num2str(currentIndex), 'FontSize', 35, 'Color', 'r');
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
visitationOrder
% Plot lines in that order.
hold on;
plot(x(visitationOrder), y(visitationOrder), 'm-', 'LineWidth', 2);
title('Start with 1 and visit next closest in order', 'FontSize', fontSize);