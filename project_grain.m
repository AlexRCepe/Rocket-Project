function [t, Pc, m_p] = project_grain(r1, options)
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
z(1) = 4.6;  % [in]
% Defines second initial geometric conditions of 6-pointed star geometry
theta = 30;  % [deg]
% Defines initial propulsion conditions
rho_p = 0.06068;  % [lb/in^3]
a = 0.0318;  % [in/(s*Psi^n)]
n = 0.28;  % [unitless]
At = pi * options.Dt^2/4;  % [in^2]
g = 32.174 * 12;  % [in/s^2]
c_star = options.c_star * 12;  % [in/s]

% Calculate initial propellant mass
A_cylinder = pi * (D/2)^2;
% The port area is 6 triangles with base 2*(r1/2 * tand(theta)) and height r1/2, plus a central hexagon.
% A simpler way is to calculate the area of the 12 right triangles forming the star.
A_port = 12 * (0.5 * (r1/2) * (r1/2 * tand(theta)));
V_p = (A_cylinder - A_port) * z(1);
m_p = V_p * rho_p;

% t=0 Grain conditions:
% Length of a star face initially. The star points are a distance r1 from
% the centerline, while the star corners are a distance r1/2. The length 
% of a star face is the hypotenuse of this triangle.
L(1) = r1/2 / cosd(theta);  % [in]
% Initial Burn area. A = L(w) * z(w). The code divides the grain into 12
% identical arcs, so this calculates the burn area of one face and then
% multiplies by a factor of 12.
Ab(1) = 12 * z(1) * L(1);  % [in^2]
Pc(1) = (a * rho_p * Ab(1) * c_star / (g * At))^(1/(1-n));
% Calculates initial burning rate and sets up the time iteration
rb(1) = a * Pc(1)^n;
w(1) = 0;
t = 0;
tstep = 0.001;
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

        A_port_inst = 12 * (0.5 * (r1/2 + w(j)) * ((r1/2 + w(j)) * tand(theta)));
        A_cylinder = pi * (D/2)^2;

        Ab_ends = 2 * (A_cylinder - A_port_inst);

        Ab(j) = 12 * z(j) * L(j);
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
        
        A_propellant_inst = 12 * (0.5 * (L(j) * sind(theta)) * (L(j) * cosd(theta)));
        Ab_end_faces = 2 * A_propellant_inst;

        
        Ab(j) = 12 * z(j) * L(j);
        % Condition where Ab ~ 0 and the grain burns out
        if Ab(j) <= 0.00001
            Ab_checker = 1;
            Pc(j) = (a * rho_p * Ab(j) * c_star / (g * At))^(1/(1-n));
            rb(j) = a * Pc(j)^n;
        end
    end
end

mdot = rho_p .* Ab .* rb; % [lb/s]

m_p_integrated = cumtrapz(t, mdot); % [lb]

fprintf("Total propellant mass from integration: %.2f lb\n", m_p_integrated(end));
fprintf("Total propellant mass from geometry: %.2f lb\n", m_p);

end