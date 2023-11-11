%set axis
 axis equal
 xlim([0 11])
 ylim([0 11])
 
for i=1:5
generate_random_line()
pause(0.1)
end

function generate_random_line()
clf
%create random two data points between 0 and 20
rand_mat = randi([0,20], 2);
line = polyfit(rand_mat(1, :),rand_mat(2, :),1);
x = linspace(rand_mat(1,1), rand_mat(1,2), 50);
y = polyval(line,x);
plot(x, y, 'k-')
angle = linspace(0, 2*pi, 50); %create different angle to itterate through so robot rotates 1 full circle
for i =1: length(x)  
q = [x(i), y(i), angle(i)];
A = [0.2852, 0.2619; 0.3354, 0.9133; 0.6797, 0.7962];
[ptch_A, hg] = plotRobot(q, A)
delete(hg)
end
end

% function [slope, intercept] = getEq(x,y)
% slope = (y(2) - y(1)) / (x(2) - x(1));
% intercept = -x(1)*slope + y(1);
% end

function array2d = H(angle, pos)
array2d = [cos(angle), -1*sin(angle), pos(1); sin(angle), cos(angle), pos(2); 
    0, 0, 1];

end

%creates tranformation matrix using H
function [trans] = T(H)
trans = zeros(4);
trans(1:2,1:2) = H(1:2, 1:2);
trans(1:2,4) = H(1:2, 3);
trans(3, 3) = 1;
trans(4, 4) = 1;
end

function [surf, hg] = plotRobot(q, A) 
%make robot
axis equal
xlim([0 21])
ylim([0 21])
x = A(:, 1);
y = A(:, 2);
%create convull and patch surface
[k,vol] = convhull(x, y);
surf = patch(x(k), y(k), 'r');

my_arrayh = H(q(3), [q(1), q(2)]); %degrees are in radians
trans = T(my_arrayh); %tranformation matrix

hg = triad;
set(surf, 'Parent', hg)
set(hg, 'Matrix', trans)
pause(0.01);
end

function PlotObstacle(obs)
poly = convhull(obs(:, 1), obs(:, 2));
x = obs(:, 1)
y = obs(:, 2)
patch(x(poly), y(poly), 'g')
end

% ploting matrix as boxes (takes values of 0 and 1)
% imagesc(inputBeacon);
% colormap(gray);

% function rotated_robot = rotateRobot(A, q)
% theta = q(3);
% 
% %this shouldn't have to be times by negative 1...something is wrong here
% R = rotMat(theta*-1); 
% rotated_robot = zeros(size(A));
%  numPoints = size(A, 1);
%  for i = 1:numPoints
% oldPoint = A(i, :);
% newPoint = oldPoint*R;
% rotated_robot(i, :) = newPoint;
% end
% end