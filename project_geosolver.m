function xb = project_geosolver(w,D,theta, r1)
% Solves the geometric problem of the second stage of 6-pointed star grain
% propogation.

% The target is a constant value that allows the function to calculate the
% intersection of the circular casing and the receding grain-line at all
% time steps, and therefore the length of the grain chunk perpendicular to
% its centerline.
target = D^2 / 4;
checker = 0;
size = 100;

while checker == 0
    % Defines possible values that x could be for the iteration to check
    x = linspace(0,0.5,size);
    j = 1;
    for j = 1:1:length(x)
        % This is the equation of intersection between the casing and
        % grain-line equations. 
        val = x(j)^2 + (-tand(theta)*x(j) - w / sind(2*theta) - r1/(2*cosd(theta)))^2;
        % If the intersection is true, val will be equal to the target
        % (within tolerance). 
        if abs(val - target) <= 0.0001
            % breaks loop and returns perpendicular grain dimension
            checker = 1;
            xb = x(j);
        end
    end
    % If a length has not been found, makes the x-values to be searched
    % smaller and repeats the loop.
    if checker == 0
        size = size * 10;
    end       
end

return