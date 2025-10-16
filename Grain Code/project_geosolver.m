function xb = project_geosolver(w,D,theta, r1)
% Solves the geometric problem of the second stage of 6-pointed star grain
% propogation.
%
% Inputs:
%   w     - Current web burn distance (in)
%   D     - Outer diameter of the grain (in)
%   theta - Half-angle of the star point (deg)
%   r1    - Initial inner radius of the star point (in)
%
% Outputs:
%   xb    - Distance from the chunk centerline to the casing intersection (in)

    num_points = 10000; % Number of points for numerical solution
    x = linspace(0, D/2, num_points);
    
    % Equation of the line representing the burning grain face
    y1 = tand(theta) * x + (r1/2 + w);
    
    % Equation of the circle representing the outer casing
    y2 = sqrt((D/2)^2 - x.^2);
    
    % Find the intersection
    [~, idx] = min(abs(y1 - y2));
    
    % Return the x-coordinate of the intersection
    xb = x(idx);
end