%creating Cspace obstacles
clf
clear
% B{1} = [0, 4; 0, 0; 4, 0; 4, 4];
% A = [4, 8; 4, 4; 8, 4];
% q_goal = [-7; -7; pi/2];
% q_init = [2; 7; pi/2];
% makeCSpace(q_init, A, B, q_goal);
% plot(q_goal(1), q_goal(2), '.r',  'MarkerSize',14);

% A = [4,6; 4, 4; 6, 4];
% B{1} = [1, 4; 1, 1; 4, 1; 4, 4];
% B{2} = [11, 3; 8, 3; 6, 0; 8, -3; 11, -3; 13, 0];
% % 
% %subplot(3, 2, 1);
% title('Angle: 0 Rad');
% q_goal = [10; 5; 0];
% q_init = [0; 0; 0];
% CB = makeCSpace(q_init, A, B, q_goal);
% % hold on
% % plot(10, 5, '.r',  'MarkerSize',14);
% bounds = [-4, 15, 15, -4; -4, -4, 8, 8];
% xlim([-4, 15])
% ylim([-4, 8])

%horizontal cell decomposition
% [ADJ, coord, wADJ] = hCellGraph(q_init, q_goal, CB, bounds);

 %visibility graph method
% [ADJ, coord, wADJ] = visibilityGraph(q_init, q_goal, CB);

%plot shortest path using dijkstras
% [final_path, final_dist] = Dijkstra(wADJ);

% if final_path ~= 0 
% plotFinalPath(final_path, coord);
% end

% subplot(3, 2, 2);
% title('Angle: pi/2 Rad');
% q_goal = [4; 5; pi/2];
% q_init = [0; -4; pi/2];
% bounds = [-4, 15, 15, -4; -4, -4, 8, 8];
% CB = makeCSpace(q_init, A, B, q_goal);
% 
% %horizontal cell decomposition
% [ADJ, coord, wADJ] = hCellGraph(q_init, q_goal, CB, bounds);
% xlim([-4, 15])
% ylim([-6, 6])
% 
% subplot(3, 2, 3);
% title('Angle: pi Rad'); 
% q_goal = [6; 3; pi];
% q_init = [0; 0; pi];
% bounds = [-4, 16, 16, -4; -4, -4, 8, 8];
% CB = makeCSpace(q_init, A, B, q_goal)
% 
% %horizontal cell decomposition
% [ADJ, coord, wADJ] = hCellGraph(q_init, q_goal, CB, bounds);
% xlim([-4, 16])
% ylim([-6, 6])
%  
% subplot(3, 2, 4);
% title('Angle: 3*pi/2 Rad');
% q_goal = [6; 8; 3*pi/2];
% q_init = [0; 0; 3*pi/2];
% bounds = [-4, 15, 15, -4; -4, -4, 8, 8];
% CB = makeCSpace(q_init, A, B, q_goal);
% 
% %horizontal cell decomposition
% [ADJ, coord, wADJ] = hCellGraph(q_init, q_goal, CB, bounds);
% xlim([-4, 16])
% ylim([-4, 8])
% % 
% subplot(3, 2, 5);
% title('Random Environment #1');
% keep running until we get an enviroment with no CSpace-obstacle intersection
while true
[B,bounds,q_init,q_goal,A] = randomEnvironment(4,7,6);
isIntersect = CheckCBIntersection(A, B, q_init);
if ~isIntersect
            break
end
end

%the random environment function has the matrices transposed so we must
%transform to work with this code
for i =1: size(B, 2)
    B{i} = B{i}.';
end

%first graph finds path using horizontal cell decomposition
subplot(1, 3, 1);
title('Horizontal Cell Decomposition');
CB = makeCSpace(q_init, A.', B, q_goal);
plot(q_init(1), q_init(2), '.r',  'MarkerSize',10);   
plot(q_goal(1), q_goal(2), '.r',  'MarkerSize',10);

[ADJ,Nodes, wADJ] = hCellGraph(q_init, q_goal, CB, bounds);
[final_path, final_dist] = Dijkstra(wADJ);

%if a final path exist exist, plot it, otherwise print statement
if final_path ~= 0 
plotFinalPath(final_path, Nodes, A.', q_init);
disp('Distance Using Horizontal Cell Decomposition:')
disp(final_dist)
else
disp('No Path Found!')
end


%second graph finds path using visibility graph method
subplot(1, 3, 2);
title('Visibility Graph Method');
CB = makeCSpace(q_init, A.', B, q_goal);
plot(q_init(1), q_init(2), '.r',  'MarkerSize',10);   
plot(q_goal(1), q_goal(2), '.r',  'MarkerSize',10);

[ADJ, Nodes, wADJ] = visibilityGraph(q_init, q_goal, CB);
[final_path, final_dist] = Dijkstra(wADJ);

%if a final path exist exist, plot it, otherwise print statement
if final_path ~= 0 
plotFinalPath(final_path, Nodes, A.', q_init);
disp('Distance Using Visibility Graphs:');
disp(final_dist)
else
disp('No Path Found!')
end

%third graph finds path using approximate cell decomposition
subplot(1, 3, 3);
title('Approximate Cell Decomposition');
CB = makeCSpace(q_init, A.', B, q_goal);
plot(q_goal(1), q_goal(2), '.r',  'MarkerSize',10);
plot(q_init(1), q_init(2), '.r',  'MarkerSize',10);   

[ADJ, wADJ, Nodes] = approxCellGraph(q_init, q_goal, CB, bounds);
[final_path, final_dist] = Dijkstra(wADJ);

%if a final path exist exist, plot it, otherwise print statement
if final_path ~= 0 
plotFinalPath(final_path, Nodes, A.', q_init);
disp('Distance Using APX. Cell Decomposition:');
disp(final_dist)
else
disp('No Path Found!')
end

% subplot(3, 2, 6);
% title('Random Environment #2');
% %keep running until we get an enviroment with no CSpace-obstacle intersection
% while true
% [B,bounds,q_init,q_goal,A] = randomEnvironment(4,7,6);
% isIntersect = CheckCBIntersection(A, B, q_init);
% if ~isIntersect
%             break
% end
% end
% %the random environment function has the matrices transposed so we must
% %transform to work with this code
% for i =1: size(B, 2)
%     B{i} = B{i}.';
% end
% makeCSpace(q_init, A.', B, q_goal);
% plot(q_goal(1), q_goal(2), '.r',  'MarkerSize',14);

%% makeCSpace 
%makeCSpace plots a configuration space with a specific 
%set of obstacles, and any arbitrary robot.
%INPUTS: position vetor q which conatains [x, y, theta]
%OUTPUTS: none 
function CB = makeCSpace(q, A, B, q_goal)
axis equal
for i=1: size(B, 2)
CB{i} = cObstacle(q, A, B{i});
% plotCObstacle(CB{i});
plotObstacle(B{i});
end

%plot robot 
[surf, hg] = plotRobot(q, A);
delete(hg);
end

%% approxCellGraph
%This function makes nodes using approximate cell decomposition
%INPUTS: q_init, q_goal, CB, bounds - 2x4 matrix containing the data points
%of the 4 corners of the bounds
%OUTPUTS: 1- ADJ: an nxn adjacency matrix 2 - weight adjacency matrix which outputs 
%length between nodes 3 - Nodes: is a a 2xn set of xy coordinates coresponding to index 
%values of ADJ 
function [ADJ, wADJ, Nodes] = approxCellGraph(q_init, q_goal, CB, bounds)
Nodes = q_init(1:2);
k_idx = {};
k_verts = {};
%four corners: bottom left, bottom right, top right, top left
corners = bounds;

for i = 1:4
    k_idx{i} = [i];
    if i == 1
        mid1 = (corners(1,1) + corners(1,2))/2;
        mid2 = (corners(2,1) + corners(2,4))/2;
        div_corners = [corners(1,1), mid1, mid1, corners(1,1); corners(2,1),corners(2,1), mid2, mid2];
    elseif i == 2
        mid1 = (corners(1,1) + corners(1,2))/2;
        mid2 = (corners(2,2) + corners(2,3))/2;
        div_corners = [mid1, corners(1,2), corners(1,2), mid1; corners(2,2),corners(2,2), mid2, mid2];
    elseif i == 3
        mid1 = (corners(1,1) + corners(1,2))/2;
        mid2 = (corners(2,2) + corners(2,3))/2;
        div_corners = [mid1, corners(1,3), corners(1,3), mid1;mid2, mid2, corners(2,3), corners(2,3)];  
    elseif i == 4
        mid1 = (corners(1,1) + corners(1,2))/2;
        mid2 = (corners(2,1) + corners(2,4))/2;
        div_corners = [corners(1,4), mid1, mid1, corners(1,4);  mid2, mid2,corners(2,4), corners(2,4)];
        
    end
    k_verts{i} = div_corners;
    
%     plot cells for only for debugging
%     x = [div_corners(1, :), div_corners(1, 1)];
%     y = [div_corners(2, :), div_corners(2, 1)];
%     plot(x, y, 'b-', 'LineWidth', 0.5);
    hold on;
end

min_dist = 2;
while true
    for k=1:numel(k_idx)
       if isempty(k_verts{k}) %already subdivided
           continue;
       end
       these_corners = k_verts{k};
       idx = k_idx{k};
       mixedFlag = true;
       
       for obs =1: numel(CB)
           x = these_corners(1, :);
           y = these_corners(2, :);
           xv = CB{obs}(1, :);
           yv = CB{obs}(2,:);
           
           cell_in_obstacle = inpolygon(x, y, xv, yv);
           obstacle_in_cell = inpolygon(xv, yv, x, y);
           if all(obstacle_in_cell == 0) && all(cell_in_obstacle ==0)           
               mixedFlag = false;
               k_class{k} = 'Empty';
           elseif all(cell_in_obstacle ==1)
               mixedFlag = false;
               k_class{k} = 'Full';
               break;
           else
               mixedFlag = true;
               k_class{k} = 'Mixed';
               break;
           end
           
       end %at this point we have a cell and we know if its full/mixed/empty
       
       if mixedFlag
            for i = 1:4
                
                if i == 1
                    mid1 = (these_corners(1,1) + these_corners(1,2))/2;
                    mid2 = (these_corners(2,1) + these_corners(2,4))/2;
                    div_corners = [these_corners(1,1), mid1, mid1, these_corners(1,1); these_corners(2,1), these_corners(2,1), mid2, mid2];
                elseif i == 2
                    mid1 = (these_corners(1,1) + these_corners(1,2))/2;
                    mid2 = (these_corners(2,2) + these_corners(2,3))/2;
                    div_corners = [mid1, these_corners(1,2), these_corners(1,2), mid1; these_corners(2,2), these_corners(2,2), mid2, mid2];
                elseif i == 3
                    mid1 = (these_corners(1,1) + these_corners(1,2))/2;
                    mid2 = (these_corners(2,2) + these_corners(2,3))/2;
                    div_corners = [mid1, these_corners(1,3), these_corners(1,3), mid1;mid2, mid2, these_corners(2,3), these_corners(2,3)];  
                elseif i == 4
                    mid1 = (these_corners(1,1) + these_corners(1,2))/2;
                    mid2 = (these_corners(2,1) + these_corners(2,4))/2;
                    div_corners = [these_corners(1,4), mid1, mid1, these_corners(1,4);  mid2, mid2, these_corners(2,4), these_corners(2,4)];

                end
                
                k_idx{end+1} = [idx, i];
                k_verts{end+1} = div_corners;
%                 plot cells only for debugging
%                 x = [div_corners(1, :), div_corners(1, 1)];
%                 y = [div_corners(2, :), div_corners(2, 1)];
%                 plot(x, y, 'k-', 'LineWidth', 0.5);
                
                next_cell = k_verts{end};
                lastIDX = numel(k_idx);
              
                for obs =1: numel(CB)
                   x = next_cell(1, :);
                   y = next_cell(2, :);
                   xv = CB{obs}(1, :);
                   yv = CB{obs}(2,:);
                   
                   
                   cell_in_obstacle = inpolygon(x, y, xv, yv);
                   obstacle_in_cell = inpolygon(xv, yv, x, y);
                   if all(obstacle_in_cell == 0) && all(cell_in_obstacle ==0)           
                       k_class{lastIDX} = 'Empty';
                   elseif all(cell_in_obstacle ==1)
                       k_class{lastIDX} = 'Full';
                       break;
                   else
                       k_class{lastIDX} = 'Mixed';
                       break;
                   end

                end

            end
            k_verts{k} = [];
            k_idx{k} = [];
            
       end
       
    end
    %set stop condition
    if norm(next_cell(:, 1) -next_cell(:, 2)) < min_dist
        break;
    end
end
ix = 1;

%holds all of the indeces of the cells that are truly empty
empty_flag = [];
for z = 1:numel(k_class)
    if isequal(k_class{z}, 'Empty')
        empty_flag(ix) = z;
        ix = ix+1;
    end
end

Nodes = q_init(1:2);

for emptycell = 1: length(empty_flag)
    this_cell = k_verts{empty_flag(emptycell)};
    center = [(this_cell(1,1)+this_cell(1,2))/2; (this_cell(2,1)+this_cell(2,4))/2];
%     plot(center(1), center(2),  '.k');
    Nodes(:, end+1) = center;  
end


Nodes = [Nodes, q_goal(1:2)];

numNodes = size(Nodes, 2);
ADJ = zeros(numNodes);
wADJ = zeros(numNodes);

for i = 1:numNodes
    for j=1:numNodes
        
        if i<=j %matrix is a mirror so only need to calculate upper right (j, i)
            continue
        end
        
        %get the segment between node i and j
        seg1 = [Nodes(:, i), Nodes(:, j)];
        isIntersect = 0;

        %itterate through each obstacle 
        for l = 1:numel(CB)
            this_obstacle = CB{l};
           %iterate through each obstacle's edge
           for m = 1:size(this_obstacle, 2)  
            if m == 1
               seg2 = [this_obstacle(:, m), this_obstacle(:, size(this_obstacle, 2)  )];              
            else
               seg2 = [this_obstacle(:, m), this_obstacle(:, m-1)];
            end

            [intersect, point,ee] = SegSegIntersect(seg1, seg2);
            if ee %edge/edge intersection -- throw out path      
                    isIntersect = 1;
                    break
            end
           end
           if isIntersect == 1
               break 
           end
        end

        if isIntersect == 0
            ADJ(i, j) = 1;
            ADJ(j, i) = 1;
            len = norm(Nodes(:, i) -Nodes(:,j));
            wADJ(j, i) = len;
            wADJ(i, j) = len;

            %lets plot to check work
            hold on
%             x_vals = [Nodes(1, i),Nodes(1, j)];
%             y_vals = [Nodes(2, i),Nodes(2, j)];
%             
%             plot(x_vals, y_vals, 'k-')
        end

    end %for j =1
end 

end

%% hCellGraph
%This function makes nodes using horizontal cell decomposition
%INPUTS: q_init, q_goal, CB, bounds - 2x4 matrix containing the data points
%of the 4 corners of the bounds
%OUTPUTS: 1- ADJ: an nxn adjacency matrix 2 - Nodes: is a a 2xn set of xy 
%coordinates coresponding to index values of ADJ 3 - weight adjacency
%matrix which outputs length between nodes
function [ADJ, Nodes, wADJ] = hCellGraph(q_init, q_goal, CB, bounds)
right_bound = bounds(1, 2);
left_bound = bounds(1, 1);
Nodes = q_init(1:2);

%iterate through each obstacle
for i=1:numel(CB)
    this_obstacle = CB{i};
    %iterate through each node
    for j = 1:size(this_obstacle, 2)
        this_node = this_obstacle(:, j);
        boundary_node(:, 1) = [right_bound; this_node(2)];
        boundary_node(:, 2) = [left_bound; this_node(2)];
          
    seg1_right = [this_node, boundary_node(:, 1)];
    seg1_left = [this_node, boundary_node(:, 2)];
    
    %create boolean vector where the first index corresponds to right 
    %boundary and second index corresponds to left boundary
    intersectsItself(1:2) = false;
 
    %for the current node, check the boundary edges not connected to that node for intersection
    for vertex = 1: size(this_obstacle, 2)
        if vertex == j || vertex + 1 ==j
           continue 
        elseif vertex == size(this_obstacle, 2)      
           if j == 1
              continue 
           end
           segEdge = [this_obstacle(:, vertex), this_obstacle(:, 1)];
        else
           segEdge = [this_obstacle(:, vertex), this_obstacle(:, vertex+1)];
        end
    
        [intersect] = SegSegIntersect(seg1_right,segEdge);  
        if intersect
            intersectsItself(1) = true; %intersects itself on the right
        end
        [intersect] = SegSegIntersect(seg1_left,segEdge);
        if intersect
             intersectsItself(2)= true; %intersects intself on the left
        end
    end %end of checking for right and left intersetion
    
    %create holder which contains whether the line segment intersect other obstacles
    isIntersect(1:2) = false; 
    closestPoint(1:2, 1:2) = Inf; %create variable to hold closest point to shorten the segment

    for k =1:numel(CB)

        if k ==i %don't check for intersection of current obstacle - already did this
            continue
        else
            %check right segment
            [tf, pntALL] = segmentObstacleIntersect(seg1_right,CB{k});
            if tf  %shorten line
                isIntersect(1) = true;
                for point =1:size(pntALL,2)
                    if norm(pntALL(:, point) -this_node) < norm(closestPoint(:,1) -this_node)
                        closestPoint(:, 1) = pntALL(:, point);
                    end
                end
            end
            
            %check left segment
            [tf, pntALL] = segmentObstacleIntersect(seg1_left,CB{k});
            if tf %shorten line
                isIntersect(2) = true;
                for point =1:size(pntALL,2)
                    if norm(pntALL(:, point) -this_node) < norm(closestPoint(:,2) -this_node)
                        closestPoint(:, 2) = pntALL(:, point);
                    end
                end

            end
        end             
    end %only we checked all the obstacles for intersection can we continue
    
    for z = 1:2 %iterate through right and left side
        if isIntersect(z) && ~intersectsItself(z) 
%             x = linspace(this_node(1), closestPoint(1, z), 50); 
%             y = repmat(this_node(2), 1, 50);
%             plot(x, y, 'k-');
            Nodes(:, end+1) = (closestPoint(:, z) + this_node) / 2; 
%             plot((closestPoint(1, z) + this_node(1)) / 2, this_node(2),  '.k',  'MarkerSize',14);                     
        elseif ~intersectsItself(z)
            Nodes(:, end+1) = (this_node + boundary_node(:, z)) / 2;  
%             x = linspace(this_node(1), boundary_node(1, z), 50);
%             y = repmat(this_node(2), 1, 50);
%             plot(x, y, 'k-');
%             plot((boundary_node(1, z) + this_node(1)) / 2, this_node(2),  '.k',  'MarkerSize',14);
        end
    end
    end
end

%add in the last node
Nodes = [Nodes, q_goal(1:2)];

%initialize the adjacency matrix
n = size(Nodes,2);
ADJ = zeros(n);
wADJ = zeros(n);

%popuate connections in the adjacency
for i = 1:n
    for j=1:n
        
        if i<=j %matrix is a mirror so only need to calculate upper right (j, i)
            continue
        end
        
%         if i==19 && j == 16
%             disp('test')
%         end
            
        %get the segment between node i and j
        seg1 = [Nodes(:, i), Nodes(:, j)];
        isIntersect = 0;

        %itterate through each obstacle 
        for l = 1:numel(CB)
            this_obstacle = CB{l};
           %iterate through each obstacle's edge
           for m = 1:size(this_obstacle, 2)  
            if m == 1
               seg2 = [this_obstacle(:, m), this_obstacle(:, size(this_obstacle, 2)  )];              
            else
               seg2 = [this_obstacle(:, m), this_obstacle(:, m-1)];
            end

            [intersect, point,ee] = SegSegIntersect(seg1, seg2);
            if ee %edge/edge intersection -- throw out path      
                    isIntersect = 1;
                    break
            end
           end
           if isIntersect == 1
               break 
           end
        end

        if isIntersect == 0
            ADJ(i, j) = 1;
            ADJ(j, i) = 1;
            length = norm(Nodes(:, i) -Nodes(:,j));
            wADJ(j, i) = length;
            wADJ(i, j) = length;

            %lets plot to check work
            hold on
%             x_vals = [Nodes(1, i),Nodes(1, j)];
%             y_vals = [Nodes(2, i),Nodes(2, j)];
%             
%             plot(x_vals, y_vals, 'k-')
        end

    end %for j =1
end %for i = 1   
end


%% visibilityGraph
%This function finds the adjacency matrix which contains whether the
%connection between two data points are a viable path. Coord contains the
%data points with the proper indeces related to the adjacency matrix
%INPUTS: q_init, q_goal, CB
%OUTPUTS: 1- ADJ: an nxn adjacency matrix 2 - coord: is a a 2xn set of xy 
%coordinates coresponding to index values of ADJ 3 - weight adjacency
%matrix which outputs length between nodes
function [ADJ, coord, wADJ] = visibilityGraph(q_init, q_goal, CB)
%make a 2xn matrix coord which contains all the nodes n: number of nodes
%including the initial and qoal data points
obsFlag = 0;
Nodes = q_init(1:2);

for i=1:numel(CB)
    Nodes = [Nodes, CB{i}];
    obsFlag = [obsFlag, repmat(i, 1, size(CB{i}, 2))]; %hold indexes obstacle refrence
end
obsFlag = [obsFlag, 0];
coord = [Nodes, q_goal(1:2)];

%initialize the adjacency matrix
n = size(coord,2);
ADJ = zeros(n);
wADJ = zeros(n);

%popuate connections in the adjacency
for i = 1:n
    for j=1:n
        
        if i<=j %matrix is a mirror so only need to calculate upper right (j, i)
            continue
        end
        
        %check if data points are a part of the same obstacle
        if obsFlag(i) == obsFlag(j) && obsFlag(i) ~= 0
            
            %get first and last indeces of obstacle
            for k_first = i:-1:1
                if obsFlag(k_first)~= obsFlag(k_first-1)
                    break
                end
            end
            for k_last = i:n
                if obsFlag(k_last)~= obsFlag(k_last+1)
                    break
                end
            end
             
            %check if two indeces are adjacent and if not a good path so continue
            if i == k_first && (~all(coord(:, i +1) == coord(:, j)) && ~all(coord(:, k_last) == coord(:, j))) 
                continue %haha this never runs
            elseif i == k_last && (~all(coord(:, i -1) == coord(:, j)) && ~all(coord(:, k_first) == coord(:, j))) 
                continue
            elseif ~all(coord(:, i +1) == coord(:, j)) && ~all(coord(:, i-1) == coord(:, j))
                continue
            else %good path??? do we have to worry about intersection
                ADJ(i, j) = 1;
                ADJ(j, i) = 1; 
                len = norm(coord(:, i) - coord(:,j));
                wADJ(j, i) = len;
                wADJ(i, j) = len;
            end  
            
        else %not the same obstacle
            
            %get the segment between i and j
            seg1 = [coord(:, i), coord(:, j)];
            isIntersect = 0;
            
            %set desired tolerance for float comparing
            tol = 0.0005;
            
            %itterate through each obstacle 
            for l = 1:numel(CB)
                this_obstacle = CB{l};
               %iterate through each obstacle's edge
               for m = 1:size(this_obstacle, 2)  
                if m == 1
                   seg2 = [this_obstacle(:, m), this_obstacle(:, size(this_obstacle, 2)  )];              
                else
                   seg2 = [this_obstacle(:, m), this_obstacle(:, m-1)];
                end
                
                [intersect, point] = SegSegIntersect(seg1, seg2);
                if intersect      
                    if all(abs(point - coord(:, i))< tol) || all(abs(point - coord(:, j))< tol) 
                        if all(abs(point - coord(:, i))< tol) && obsFlag(i)~= l
                           %point of objects is different then the segment object
                           isIntersect = 1;
                           break
                        elseif all(abs(point - coord(:, j))< tol) && obsFlag(j)~= l
                           %point of objects is different then the segment object
                           isIntersect = 1;
                           break
                        end
                        %otherwise do nothing
  
                    else
                        isIntersect = 1;
                        break
                    end
                end
               end
               if isIntersect == 1
                   break 
               end
            end
            
            if isIntersect == 0
                ADJ(i, j) = 1;
                ADJ(j, i) = 1;
                len = norm(coord(:, i) -coord(:,j));
                wADJ(j, i) = len;
                wADJ(i, j) = len;
                
                %lets plot to check work
                hold on
%                 x_vals = [coord(1, i),coord(1, j)];
%                 y_vals = [coord(2, i),coord(2, j)];
%                 
%                 plot(x_vals, y_vals, 'k-')
                
            end
            
        end
        
    end %for j =1
end %for i = 1

end %function 

%% SegSegIntersect
%function checks if two lines intersect
%INPUTS: two segments - each segment contains two data points an the line in 
%between these two points is checked for intersection against the second segment   
%OUTPUTS: intersect - a boolean value containing whether and intersect
%occurs point - the point at which intersection occurs
function  varargout = SegSegIntersect(seg1, seg2)
%set initial conditions
intersect = false;
point = [];
ee = false;
%get coefficients
M1 = seg1*[0,1;1,1]^(-1);
M2 = seg2*[0,1;1,1]^(-1);
M = [M1(:,1), -M2(:,1)];
B = (M2(:,2)-M1(:,2));

%get s-values
s1s2 = M^(-1)*B;
s1 = s1s2(1);
s2 = s1s2(2);

if s1>0 && s1<1 && s2>0 && s2<1
   intersect = true; %edge/edge 
   ee = true;
end
if (s1==0 || s1 ==1) && (s2 ==0 || s2==1)
   intersect = true; %vertex/vertex
end
if (s1==0 || s1==1) && (s2>0 && s2<1)
   intersect = true; %vertex/edge 
end
if (s1>0 && s1<1) && (s2==0 || s2==1)
   intersect = true; %edge/vertex
end

%get intersection point
point = M1*[s1; 1];

varargout{1} = intersect;
varargout{2} = point;
varargout{3} = ee;

end

%% SegmentObstacleIntersect
%function checks if ...
%INPUTS:    
%OUTPUTS:
function varargout = segmentObstacleIntersect(segment01,CB)
%Check for an intersect between an obstacle and a line segment
% tf = segmentObstacleIntersect(segment01,CB)
% [tf, pntALL] = segmentObstacleIntersect(segment01,CB)
% [tf, pntALL, edge/edgeIntersection] = segmentObstacleIntersect(segment01,CB)
ee = false;
tf = false;
pntALL = [];
n = size(CB, 2);
for i = 1:n
    j = i+1;
    if j>n
        j = 1;
    end
    segment02 = [CB(:,i),CB(:,j)];
    
    [Int, pnt, edge] = SegSegIntersect(segment01,segment02);
    if edge 
        ee = true;
    end
    if Int
        tf = true;
        if nargout == 1
            varargout{1} = tf;
            return
        else
            pntALL(:,end+1) = pnt;
        end
                
    end
            
end

varargout{1} = tf;
varargout{2} = pntALL;
varargout{3} = ee;
end

%% Dijkstra
%Function Dijkstra uses dijkstras method to find the shortest path from the
%starting point to the goal of the robot.
%INPUTS: wADJ weighted adgacency matrix which contains nxn matrix
%containing the distance between node ni(rows) and node nj(columns)
%OUTPUTS: final_path - ordered set of nodes which produce the shortest path
%final_dist - the total distance of the shortest path found
function [final_path, final_dist] = Dijkstra(wADJ)
final_path = 0;
Q = [];
numVertices = size(wADJ, 2);
for v=1: numVertices
   dist(v) = Inf;
   prev(v) = 1/0; %matlab reads this as infinity too
   Q(v) = v;
end

%set distance from initial to initial as 0
dist(1) = 0;

while ~isempty(Q)
    this_vertex = find(dist == min(dist(Q)));
    
    %must check that vertex with smalest distance is still within Q
    if length(this_vertex) > 1
       while ~ismember(this_vertex(1),Q)
               this_vertex(1) = []; %as long as first the point is not in Q remove it
       end
       this_vertex = this_vertex(1); 
    end
    
    Q_var = find(Q == this_vertex);
    Q(Q_var) = [];

     
%check if we have reached the end 
if this_vertex == numVertices
   break; 
end
    
for i =1:length(Q)
    if wADJ(Q(i), this_vertex) ~= 0
    alt = dist(this_vertex) + wADJ(Q(i), this_vertex);
    
    if alt < dist(Q(i))
        dist(Q(i)) = alt;
        prev(Q(i)) = this_vertex;
    end
    
    end
    
end

end

backward_sequence = [];
idx = 1;
final_dist = 0;
u = numVertices;
if prev(u) ~= Inf || u == 1
   while u ~= Inf
       backward_sequence(idx) = u;
       idx = idx+1;
       final_dist = final_dist + dist(u);
       u = prev(u);
   end
   final_path = fliplr(backward_sequence);
end

end

%% rotMat
%INPUTS: angle
%OUTPUTS: rotation matrix of respective angle
function R = rotMat(angle)
R = [cos(angle), -sin(angle); sin(angle), cos(angle)];
end

%% APPL_A
%This function checks for type A contanct -- an obsticle corner is in contact 
%with the robot edge
%INPUTS: q, robot A (nA X 2), obstacle B
%OUTPUTS: appla - a matrix with nA(number of data points in A)X nB (number of 
%data points in B) which contains a 1 in it's respective position if their
%is type A contanct and 0 if there is not
function appla = APPL_A(q, A, B)
R = rotMat(q(3)); 
A = R*A.';

numA = size(A, 2); %number of data points of vertexes of robot
numB = size(B, 2); %number of data points of vertexes of obstacle
appla = zeros(numA, numB);

for i = 1: numA
   if i ~= numA
       edge = A(:, i:i+1);
   else
       edge = [A(:, i), A(:, 1)]; 
   end
   outNorm = outwardNormal(edge);
   for j =1:numB
      %check conditions using the dot product
      if j==1
          vectObs1 = B(:, j+1) - B(:, j);
          vectObs2 = B(:, numB) - B(:, j);
      elseif j==numB
          vectObs1 = B(:, 1) - B(:, j);
          vectObs2 = B(:, j-1) - B(:, j);    
      else
          vectObs1 = B(:, j+1) - B(:, j);
          vectObs2 = B(:, j-1) - B(:, j);  
      end
      if dot(outNorm, vectObs1) >= 0 && dot(outNorm, vectObs2) >=0
          appla(i,j) = 1;
      end
   end
   
end
end

%% APPL_B
%This function checks for type B contanct -- a robot's corner is in contact 
%with the obstacles edge
%INPUTS: q, robot A (nA X 2), obstacle B
%OUTPUTS: appla - a matrix with nA(number of data points in A)X nB (number of 
%data points in B) which contains a 1 in it's respective position if their
%is type B contanct and 0 if there is not
function applb = APPL_B(q, A, B)
R = rotMat(q(3)); 
A = R*A.';
numA = size(A, 2); %number of data points of vertexes of robot
numB = size(B, 2); %number of data points of vertexes of obstacle
applb = zeros(numA, numB);

for i = 1: numB
   if i ~= numB
       edge = B(:, i:i+1);
   else
       edge = [B(:, i), B(:, 1)]; 
   end
   outNorm = outwardNormal(edge);
   for j =1:numA
      %check conditions using the dot product
      if j==1
          vectObs1 = A(:, j+1) - A(:, j);
          vectObs2 = A(:, numA) - A(:, j);
      elseif j==numA
          vectObs1 = A(:, 1) - A(:, j);
          vectObs2 = A(:, j-1) - A(:, j);    
      else
          vectObs1 = A(:, j+1) - A(:, j);
          vectObs2 = A(:, j-1) - A(:, j);  
      end
      if dot(outNorm, vectObs1) >= 0 && dot(outNorm, vectObs2) >=0
          applb(j,i) = 1;
      end
   end
   
end

end

%% cObstacle
%This function uses the matrices from APPL_A and APPL_B to find the points
%of the configuration space
%INPUTS: q, robot A, obstacle B
%OUTPUTS: CB, a matrix which contains the points of the Configuration space
function CB = cObstacle(q, A, B)
R = rotMat(q(3)); 
Ar = R*A.';

B = B.';%now matrices are 2 X n_robot(A)/n_obst(B)
typeA = APPL_A(q, A, B); %gets matrices containing true and false for type A and B
typeB = APPL_B(q, A, B); %matrix is a n_robot X n_obst
n_robot = size(typeA, 1);
n_obst = size(typeA, 2);

%reset refrence frame of robot to origin
pt = Ar(:, 1);
    new_A(:, 1) = [0; 0];
    for p = 2:length(Ar(1,:))
        new_A(:, p) = Ar(:, p) - pt;
    end

%itterate for each true value of type A or B and add appropiate data points to CB
counter = 1;
for i = 1:n_robot
    for j = 1:n_obst
        if typeA(i, j) ==1 || typeB(i,j) == 1
        %put vertex bj-ai into CB
        CB(:, counter) = B(:, j) - new_A(:, i);
        counter = counter + 1; 
        end
    end
end
 CB = CB.';
 idx = convhull(CB, 'simplify', true);
 idx = idx(1: size(idx, 1)-1);
 CB = CB(idx, :).';
%returns CB which contains all new and old data points for the C-Space
end  

%% plotObstacle
%INPUTS: obstacle B
%OUTPUTS: none
function plotObstacle(B)
x_obs = B(:, 1);
y_obs = B(:, 2);
obs = convhull(x_obs, y_obs);
patch(x_obs(obs), y_obs(obs), 'b');
end

%% plotCObstacle
%INPUTS: C -obstacle CB
%OUTPUTS: none
function plotCObstacle(CB)
patch(CB(1, :), CB(2,:), 'g');
end

%% T
%INPUTS: H
%OUTPUTS: Transformation Matrix
function [trans] = T(H)
trans = zeros(4);
trans(1:2,1:2) = H(1:2, 1:2);
trans(1:2,4) = H(1:2, 3);
trans(3, 3) = 1;
trans(4, 4) = 1;
end

%% H
%This function rotates the points that make up of a robot one by one using
%the rotation matrix
%INPUTS: matrix A of the robot's data points, configuration vector q 
%OUTPUTS: The rotated robots matrix   
function array2d = H(angle, pos)
array2d = [cos(angle), -1*sin(angle), pos(1); sin(angle), cos(angle), pos(2); 
    0, 0, 1];

end

%% plotFinalPath
%Plots shortest path found using Dijkras method
%INPUTS: final_path - indices of verticis corresponding to the shortest path via 
%dijkstras methon, coord - list of vertices
%OUTPUTS: none
function plotFinalPath(final_path, coord, A, q_goal)
for i =1: length(final_path) -1
   
   x_vals = [coord(1, final_path(i)),coord(1, final_path(i+1))];
   y_vals = [coord(2, final_path(i)),coord(2, final_path(i+1))];
  
   if x_vals(1) == x_vals(2)
       y = linspace(y_vals(1), y_vals(2), 20);
       x = repmat(x_vals(1), 1, 50);
       plot(x, y, 'r-');
   else
       line = polyfit(x_vals, y_vals, 1);
       x = linspace(x_vals(1), x_vals(2), 20);
       y = polyval(line,x);
       plot(x, y, 'r-');
   end
   
%    plot(x_vals, y_vals,  'r-');

   for j =1: length(x)
        q = [x(j), y(j), q_goal(3)];
        [ptch_A, hg] = plotRobot(q, A);
        pause(0.1)
        delete(hg)
   end
   
end
end

%% plotRobot
%INPUTS: q and A
%OUTPUTS: patched object and parent handle
function [surf, hg] = plotRobot(q, A) 
%make robot
x_o = A(:, 1);
y_o = A(:, 2);

%shift robot so triad is at robots first point
x_shift = A(1,1);
y_shift = A(1,2);
x = x_o - x_shift;
y = y_o - y_shift;

%create convull and patch surface
[k,vol] = convhull(x, y);
surf = patch(x(k), y(k), [ 1 0.50 0]);

my_arrayh = H(q(3), [q(1), q(2)]); %degrees are in radians
trans = T(my_arrayh); %tranformation matrix

hg = triad;
set(surf, 'Parent', hg)
set(hg, 'Matrix', trans)

%plot black point to represent robot in the configuration space
R = rotMat(-1*q(3)); 
Ar = A*R;
hold on

end

%% isIntersect
%checks to see if the environment has colliding CB obstacles
%INPUTS:A, B, q_init
%OUTPUTS: boolean isIntersect which is true if 2 or more CB ostacles collide
function isIntersect = CheckCBIntersection(A, B, q_init)
A = A.';
for i =1: size(B, 2)
    B{i} = B{i}.';
end
for i=1: size(B, 2)
CB_copy{i} = cObstacle(q_init, A, B{i});
end
%check for CSpace obstacles overlapping
isIntersect = false;
while ~isempty(CB_copy)
    CB_tmp = CB_copy{1};
    %remove from CB_copy
    CB_copy(1) = [];
    for i = 1:numel(CB_copy)
        for j = 1:size(CB_tmp,2)
            if j < size(CB_tmp,2)
                v1 = CB_tmp(:,j);
                v2 = CB_tmp(:,j+1);
            else
                v1 = CB_tmp(:,j);
                v2 = CB_tmp(:,1);
            end
            
            if intersectPolygon(v1,v2,CB_copy{i})
                isIntersect = true;
                break;
            end
        end
        
        if isIntersect
            break;
        end
    end
    if isIntersect
            break;
    end
end

end

%% Extra Pieces of code that may be helpful later
%    if x_vals(1) == x_vals(2)
%        y = linspace(y_vals(1), y_vals(2), 50);
%        x = repmat(x_vals(1), 1, 50);
%        plot(x, y, 'r-', 'LineWidth',2);
%    else
%        line = polyfit(x_vals, y_vals, 1);
%        x = linspace(x_vals(1), x_vals(2), 50);
%        y = polyval(line,x);
%        plot(x, y, 'r-', 'LineWidth',2);
%    end