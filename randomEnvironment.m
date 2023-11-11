function [B,bounds,q_init,q_goal,A] = randomEnvironment(k,n,m)
% RANDOMBs creates a set of k random, non-intersecting
% obstacles, each with a maximum of n-vertices
% robot is created with max of m-vertices
%   modified from M. Kutzer's randomCBs

%% Define environmental settings

% Define obstacle scaling and offset factors
obsScale = 20;
robotScale = 1;
obsOffset = 2*obsScale;

% Define bound limit scaling
% -> NOTE: This assumes a rectagular boundary (because most students make
%          this assumption even though it is not necessary).
bndOffset = obsScale/2;

%% Randomly create obstacles
B = {};
k_now = 1;
while true
    % Define obstacle vertices
    B_tmp = randi(10,2,n).*(rand(2,n) - 0.5*ones(2,n)) + ...
        repmat(randi(obsOffset,2,1).*(rand(2,1) - 0.5*ones(2,1)), 1, n);
    % Make sure obstacle is convex
    idx = convhull(B_tmp(1,:),B_tmp(2,:));
    idx(end) = [];
    B_tmp = B_tmp(:,idx);
    
    % Check if new obstacle intersects existing obstacles
    isIntersect = false;
    for i = 1:numel(B)
        for j = 1:size(B_tmp,2)
            if j < size(B_tmp,2)
                v1 = B_tmp(:,j);
                v2 = B_tmp(:,j+1);
            else
                v1 = B_tmp(:,j);
                v2 = B_tmp(:,1);
            end
            
            if intersectPolygon(v1,v2,B{i})
                isIntersect = true;
                break;
            end
        end
        
        if isIntersect
            break;
        end
    end
    
    % If no intersection, add obstacle
    if ~isIntersect
        % Add obstacle
        B{k_now} = B_tmp;
        % Check if number of obstacles is reached
        if k_now == k
            break
        end
        % Iterate obstacle number
        k_now = k_now+1;
    end
end

%% Randomly create Robot
A = [];
while true
    % Define obstacle vertices
    A_tmp = randi(10,2,m).*(rand(2,m) - 0.5*ones(2,m)) + ...
        repmat(randi(robotScale,2,1).*(rand(2,1) - 0.5*ones(2,1)), 1, m);
    % Make sure robot is convex
    idx = convhull(A_tmp(1,:),A_tmp(2,:));
    idx(end) = [];
    A_tmp = A_tmp(:,idx);
%     A = A_tmp(:,idx);    
%     %find centroid    
%     pIn = polyshape(A(1,:),A(2,:));
%     [xCnt,yCnt] = centroid(pIn);
%     Cntr = [xCnt; yCnt];
    %subtract centroid to move robot to (0,0)
%     for i=1:size(A)
%         A(:,i)= A(:,i)-Cntr;
%     end
        
% % TO DO     
%     % Check if robot intersects existing obstacles
    isIntersect = false;
    for i = 1:numel(B)
        for j = 1:size(A_tmp,2)
            if j < size(A_tmp,2)
                v1 = A_tmp(:,j);
                v2 = A_tmp(:,j+1);
            else
                v1 = A_tmp(:,j);
                v2 = A_tmp(:,1);
            end
            
            if intersectPolygon(v1,v2,B{i})
                isIntersect = true;
                break;
            end
        end
        
        if isIntersect
            break;
        end
    end
%     
    % If no intersection, add robot
    if ~isIntersect
        % Add obstacle
        A = A_tmp;
        break;
    end
end
%% Define bounds to environment

% Append all obstacles vertices
verts = [];
for i = 1:k
    verts = [verts, B{i}];
end

% Find limits
xLIM(1) = min(verts(1,:));
xLIM(2) = max(verts(1,:));
yLIM(1) = min(verts(2,:));
yLIM(2) = max(verts(2,:));

% Offset bound limits
xLIM = xLIM + [-bndOffset,bndOffset];
yLIM = yLIM + [-bndOffset,bndOffset];

% Define bounding polygon
bounds = [xLIM(1), xLIM(2), xLIM(2), xLIM(1);...
          yLIM(1), yLIM(1), yLIM(2), yLIM(2)];
      
%% Select random q_init and q_goal within the environmental bounds
q = [];

while true
    q_TMP(1,:) = diff(xLIM)*rand(1,1) + xLIM(1);
    q_TMP(2,:) = diff(yLIM)*rand(1,1) + yLIM(1);
    
    isIntersect = false;
    for i = 1:numel(B)
        [in,on] = inpolygon(q_TMP(1),q_TMP(2),B{i}(1,:),B{i}(2,:));
        
        if in || on
            isIntersect = true;
            break
        end
    end
    
    if ~isIntersect
        q(:,end+1) = q_TMP;
        
        if size(q,2) == 2
            break
        end
    end
end

q_init = q(:,1);
q_goal = q(:,2);
%Make q_init and q_goal 3x1
q_init(end+1) = pi/2;
q_goal(end+1) = pi/2;