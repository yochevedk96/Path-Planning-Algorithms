x_robot = A(:, 1);
y_robot = A(:, 2);
robot = convhull(x_robot, y_robot);
patch(x_robot(robot), y_robot(robot), [1 0.5 0])

A2 = rotateRobot(A, [1, 1, pi/4]);
x2_robot = A2(:, 1);
y2_robot = A2(:, 2);
robot2 = convhull(x2_robot, y2_robot);
patch(x2_robot(robot2), y_robot2(robot2), 'b')


function rotated_robot = rotateRobot(A, q)
theta = q(3)
rotated_robot = zeros(size(A))
numPoints = size(A, 1);
for i = 1:numPoints
oldPoint = A(i, :)
phi = atan(oldPoint(2)/oldPoint(1))
syms x_new y_new
eqn1 = phi + theta == atan(y_new/ x_new)
eqn2 = oldPoint(2)^2 + oldPoint(1)^2 == abs(y_new^2 + x_new^2)
sol = solve([eqn1, eqn2], [x_new, y_new])
newPoint = [sol.x_new, sol.y_new]
rotated_robot(i, :) = newPoint;
end
end