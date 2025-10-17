function [t, Pc] = project_grain(r1, options)
% AUTHOR: CHARLIE OLSON
% This function tracks the propagation of the grain in a 6-sided star bore
% geometry, and provides a Chamber Pressure History.
arguments
    r1
    options.Dt
    options.c_star
end

% Rocket Dimensions of tube, throat, and exit
D = 1.212;  % [in]
Dt = 0.25;  % [in]
De = 1.5;  % [in]
z(1) = 4.6;  % [in]
% Defines second initial geometric conditions of 6-pointed star geometry
theta = 30;  % [deg]
% Defines initial propulsion conditions
rho_p = 0.06068;  % [lb/in^3]
a = 0.0318;  % [in/(s*Psi^n)]
n = 0.28;  % [unitless]
At = pi * Dt^2/4;  % [in^2]
g = 32.174 * 12;  % [in/s^2]
c_star = 5088 * 12;  % [in/s]

% t=0 Grain conditions:
% Length of a star face initially. The star points are a distance r1 from
% the centerline, while the star corners are a distance r1/2. The length 
% of a star face is the hypotenuse of this triangle.
L(1) = r1/2 / cosd(theta);  % [in]
% Initial Burn area. Side area is A = L(w) * z(w). The code divides the grain into 12
% identical arcs, so this calculates the burn area of one face and then
% multiplies by a factor of 12. The top area is determined by dividing the
% star into 12 equal triangles, and subtracting their areas from the
% casing circle.
Ab_top(1) = pi * D^2 / 2 - 12 * r1 * L(1) * sind(theta);
Ab(1) = 12 * z(1) * L(1) + Ab_top(1);  % [in^2]
Pc(1) = (a * rho_p * Ab(1) * c_star / (g * At))^(1/(1-n));
% Calculates initial burning rate and sets up the time iteration
rb(1) = a * Pc(1)^n;
w(1) = 0;
t = 0;
tstep = 0.0001;
% Defines the limiting web distance of the first phase of the burn, the
% web distance where the points of the stars contact the outer casing.
w_lim = D / 4 - r1 / 2;
% Sets counter and checker variables for integration
j = 1;
Ab_checker = 0;
% Ab_checker breaks the loop when Ab ~ 0, i.e. when the fuel has been
% expended.
while Ab_checker == 0
    Pc(j) = (a * rho_p * Ab(j) * c_star / (g * At))^(1/(1-n));
    rb(j) = a * Pc(j)^n;
    % This first conditional is the first stage of the burn, where the star
    % points are propogating rapidly to the case boundary.
    if w(j) <= w_lim
        % Here is the iterative process, where the web distance at every
        % time t is determined by the time interval, the prior time step,
        % and the burning rate at the prior time step.
        j = j + 1;
        t(j) = t(j-1) + tstep;
        w(j) = w(j-1) + tstep * rb(j-1);
        % Calculates the new length and heights of the star faces, and
        % then determines the new burn area.The points of the stars 
        % propogate twice as fast as the rest of the star surface due to
        % our assumptions of sharp corners.
        L(j) = (r1 / 2 + w(j)) / cosd(theta);
        z(j) = z(1) - 2 * w(j);
        Ab_top(j) = pi * D^2 / 2 - 12 * (r1+2*w(j)) * L(j) * sind(theta);
        Ab(j) = 12 * z(j) * L(j) + Ab_top(j);
    % The next area of the code is the second phase of the burn, when the
    % grain splits into 6 different triangles burning down to a point. Like
    % before, this solution zooms into one of the twelce sector arcs that
    % is representative of the problem.
    else
        j = j + 1;
        t(j) = t(j-1) + tstep;
        w(j) = w(j-1) + tstep * rb(j-1);
        z(j) = z(1) - 2 * w(j);
        % Solves for the distance xb between the centerline of the grain
        % chunk and where the grain intersects the circle casing.
        xb(j) = project_geosolver(w(j),D,theta,r1);
        % Solves for the length of the chunk face.
        L(j) = xb(j) / cosd(theta);
        Ab_triangle(j) = xb(j) * L(j) * sind(theta) / 2;
        beta(j) = asind(xb(j) / (D/2));
        Ab_arc(j) = 0.25 * (pi * D^2 * beta(j)/360 - xb(j) * D * cosd(beta(j)));
        Ab_top(j) = 24 * (Ab_triangle(j) + Ab_arc(j));
        Ab(j) = 12 * z(j) * L(j) + Ab_top(j);
        % Condition where Ab ~ 0 and the grain burns out
        if Ab(j) <= 0.00001
            Ab_checker = 1;
            Pc(j) = (a * rho_p * Ab(j) * c_star / (g * At))^(1/(1-n));
            rb(j) = a * Pc(j)^n;
        end
    end
end
end