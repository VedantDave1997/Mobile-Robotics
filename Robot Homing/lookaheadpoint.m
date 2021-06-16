function [xp, yp, phip, nextPathPointIdx, previousPathPointCleared] = lookaheadpoint(x, y, waypoints, lookaheaddistance)
%·lookaheadpoint - Calculates the next lookaheadpoint
%·========================================================================
%
%  [xp, yp] = lookaheadpoint(x, y, waypoints, lookaheaddistance)
%
%  Description:
%    Calculates the next goalpoint based on the current position [x,y] and
%    the path waypoints defined in waypoints with the distance defined in
%    lookaheaddistance.
%
%  Input:
%    x: Current X-Position
%    y: Current Y-Position
%    waypoints: n x 2 array of waypoints
%    lookaheaddistance: Distance of the lookaheadpoint to the robot
%
%  Output:
%    xp: X-Position of the lookaheadpoint
%	 yp: Y-Position of the lookaheadpoint
%    phip: direction of the line segment on which lookaheadpoint is located
%
%  Known Bugs:
%	  - 
%
%
%  Authors:
%	 Christian Wissing
%
%  See also
%
%  Copyright (c) 2014 RST, Technische Universität Dortmund, Germany
%                     www.rst.e-technik.tu-dortmund.de
%
%%·=======================================================================
%#codegen

if size(waypoints,2) ~= 2
    error('Dimension of waypoints should be n by 2!');
end

% Number of waypoints
nWaypoints = size(waypoints,1);

% Actual position
pAct = [x, y];

% Find closest point on the path
projectedPoint = [];
normDirectionVec = [];
minDist = 99999;
nextPathPointIdx = 1;
for i_point = 1:nWaypoints-1
    
    % Project point on the segment
    p1 = waypoints(i_point,:);
    p2 = waypoints(i_point + 1,:);
    
    % Direction Vector
    dir = p2 - p1;
    l = norm(dir);
    
    if l == 0
        % Waypoints p1 and p2 are equal
        continue;
    end
    
    % Projection
    t = max([0, min([1, (pAct - p1)*dir'/(l^2)])]);
    pp = p1 + t * dir;
    
    dist = norm(pp-pAct);
    
    if dist < minDist
        minDist = dist;
        projectedPoint = pp;
        normDirectionVec = dir./l;
        nextPathPointIdx = i_point + 1;
    end    
end




% Check distance to next waypoint
dist2WP = norm((projectedPoint - waypoints(nextPathPointIdx,:)));
if dist2WP < lookaheaddistance
    % The lookahead point is on the consecutive segment
    previousPathPointCleared=0;
    % Check for end of path
    if nextPathPointIdx == nWaypoints
        xp = waypoints(nWaypoints, 1);
        yp = waypoints(nWaypoints, 2);
        phip = 0;
        return;
    end

    % Substract distance to next wp
    ldist = lookaheaddistance - dist2WP;
    % New projected point is the consecutive wp
    projectedPoint = waypoints(nextPathPointIdx,:);
    % New direction vector
    normDirectionVec = waypoints(nextPathPointIdx + 1,:) - waypoints(nextPathPointIdx,:);
    normDirectionVec = normDirectionVec./norm(normDirectionVec);
else
    % The lookahead point is on the actual segment
    previousPathPointCleared=1;
    ldist = lookaheaddistance;
end

% Calculate lookahead point
lp = projectedPoint + normDirectionVec * ldist;

xp = lp(1);
yp = lp(2);
[rho, phip]=cart2pol(normDirectionVec(1),normDirectionVec(2));

end