%set axis
 axis equal
 xlim([0 11])
 ylim([0 11])
 
for i=1:5
generate_random_line()
pause(0.5)
end

function generate_random_line()
%make robot
clf
axis equal
xlim([0 21])
ylim([0 21])
x = [0.2852; 0.9133; 0.7962];
y = [0.2619; 0.3354; 0.6797];

%create random two data points between 0 and 20
rand_mat = randi([0,20], 2);

%create anvull and patch surface
[k,vol] = convhull(x, y);
surf = patch(x(k), y(k), 'r');

hg = triad;
set(surf, 'Parent', hg)
% num_points = 20;


% [slope, intercept] = getEq(rand_mat(1, :),rand_mat(2, :));
% x = linspace(rand_mat(1,1), rand_mat(1,2), 50);
% plot(x, slope*x+intercept, 'k-')

%create a line between the two random datat points
line = polyfit(rand_mat(1, :),rand_mat(2, :),1);
x = linspace(rand_mat(1,1), rand_mat(1,2), 50);
y = polyval(line,x);
plot(x, y, 'k-')

%create different angle to itterate through so robot rotates 1 full circle
angle = linspace(0, 2*pi, 50);

%create random obstacles
PlotObstacle(rand(5,2)*5+2)
PlotObstacle(rand(5,2)*5+6)
PlotObstacle(rand(5,2)*5+10)
PlotObstacle(rand(5,2)*5+15)

%now we are ready to transform our robot through the generated line
for i = 1:length(x)  
%  my_arrayh = H(angle(i), [x(i),slope*x(i)+intercept]); %degrees are in radians
 my_arrayh = H(angle(i), [x(i),polyval(line,x(i))]); %degrees are in radians
 trans_mat = T(my_arrayh); %tranformation matrix
 plotRobot(hg, trans_mat)    
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

function [hg] = plotRobot(hg, trans) 

pause(0.1);
set(hg, 'Matrix', trans)
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
