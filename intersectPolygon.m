function [tf,vIntersect,onVertex] = intersectPolygon(v1,v2,PG)
% INTERSECTPOLYGON determines whether the segment connecting two points
% intersects a convex polygon
%   [tf,vIntersect,onVertex] = INTERSECTPOLYGON(v1,v2,PG) finds if the  
%   segment defined by the pair of vertices v1 and v2 intersects a polygon 
%   defined as an ordered set of x/y vertices.
%
%       tf - binary value that is true if an intersection occurs (unless
%            that intersection is solely with a vertex of the obstacle)
%       vIntersect - 2xN array containing all points of intersect between
%            the obstacle and polygon including intersections with polygon
%            vertices.
%       onVertex - 1xN binary array describing whether an intersection
%            occurs on a vertex or not.
%
%   M. Kutzer

%% Set debug flag
debugON = false;

%% Initialize tf
tf = false;

%% Check if either of the points are inside of the polygon
[in,on] = inpolygon(v1(1),v1(2),PG(1,:),PG(2,:));
if in && ~on
    tf = true;
    return;
end

[in,on] = inpolygon(v2(1),v2(2),PG(1,:),PG(2,:));
if in && ~on
    tf = true;
    return;
end

%% Check each edge individually
s0 = 0;
sf = 1;
S = [s0,sf;1,1];
invS = S^(-1);

% Fit line to vertex pair
AB = [v1,v2]*invS;
A = AB(:,1);
B = AB(:,2);

% Fit line to each edge of the polygon
vIntersect = [];    % Initialize intersection array
onVertex = [];

ZERO = 1e-6;
n = size(PG,2);
for i = 1:n
    if i < n
        idx1 = i;
        idx2 = i+1;
    else
        idx1 = i;
        idx2 = 1;
    end
    CD = [PG(:,idx1),PG(:,idx2)]*invS;
    C = CD(:,1);
    D = CD(:,2);
    
    % Find possible intersection
    ss = [A, -C]^(-1) * (D-B);
    
    if debugON
        fprintf('\t%2.0d: %10.4f, %10.4f\n',i,ss(1),ss(2));
    end
    
    % Check if intersection occurs within the bounds of each segment
    if ss(1) >= 0 && ss(1) <= 1
        if ss(2) >= 0 && ss(2) <= 1
            
            vIntersect(:,end+1) = AB*[ss(1); 1];  % Calculate intersection point
            if norm(vIntersect(:,end)-v1) < ZERO || norm(vIntersect(:,end)-v2) < ZERO
                % Start and/or end-points of the segments are shared
                onVertex(:,end+1) = true;
                if debugON
                    fprintf('One vertex is the same as the polygon vertex... IGNORING\n');
                end
            else
                % Check if intersection occurs 
                if abs(ss(2)) < ZERO || abs(ss(2) - 1) < ZERO
                    % -> On polygon vertex
                    onVertex(:,end+1) = true;
                    
                    % Check for and remove possible vertex redundancy
                    % -> Ignore current value when checking
                    v_tmp = vIntersect;
                    v_tmp(:,end) = inf;
                    % -> Get current value
                    v_now = vIntersect(:,end);
                    % -> Calculate Euclidean norm
                    deltaV = v_tmp - repmat(v_now,1,size(v_tmp,2));
                    norm_dV = sqrt( sum( deltaV.^2, 1) );
                    
                    % If the vertex is redundant, remove it
                    if any( norm_dV < ZERO )
                        vIntersect(:,end) = [];
                        onVertex(:,end) = [];
                    end
                else
                    % -> On polygon edge
                    onVertex(:,end+1) = false;
                end
                
                tf = true;          % Return true
                %return
            end
            
        end
    end
end

if debugON
    fprintf('\n');
end

%% Check for redundant intersections
% % Define Euclidean norm between adjacent vertices
% d_vInt = diff(vIntersect,1,2);
% norm_d_vInt = sqrt( sum(d_vInt.^2,1) );
% % Find all values that are larger than our ZERO tolerance
% bin = norm_d_vInt > ZERO;
% 
% % Keep only non-redundant values
% vIntersect = vIntersect(:,bin);
% onVertex = onVertex(:,bin);
