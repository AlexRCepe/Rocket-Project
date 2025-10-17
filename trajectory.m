clc
clear
close all

%% ROCKET AND ENVIRONMENTAL PARAMETERS

g = 9.81;            % Gravity                    [m/s^2]
m_dry = 0.625;       % Dry mass                   [kg]
m_prop = 0.086;      % Propellant mass            [kg]
m0 = m_dry + m_prop; % Initial mass               [kg]
A = 0.01081;         % Cross-sectional area       [m^2]
Cd = 0.75;           % Drag coefficient
rho0 = 1.225;        % Sea-level air density      [kg/m^3]
h_scale = 8400;      % Atmospheric scale height   [m]
v_w = 40;            % Wind speed in +x direction [km/h]

v_w = v_w / 3.6;     % From km/h to m/s

gamma = 1.2;         % Specific heat ratio (air)

De = 1.5;            % Exit diameter               [in]
Dt = 0.25;           % Throat diameter             [in]
c_star = 5088;       % Characteristic velocity     [ft/s]

epsilon = De.^2 ./ Dt.^2;
Me = get_Me(epsilon, gamma);

%% GRAIN PARAMETERS AND THRUST CURVE CALCULATION

r1 = 0.5;             % Internal radius             [in]

% Thrust history [time (s), thrust (N)]
[thrust, mass_flow] = get_curves(r1, Me, epsilon, gamma, "Dt", Dt, "c_star", c_star);
thrust_fn = @(t) interp1(thrust(:,1), thrust(:,2), t, 'linear', 0);

mdot_fn = @(t) interp1(mass_flow(:,1), mass_flow(:,2), t, 'linear', 0);

% TODO : Change to ISA
% Air Density Model 
rho = @(z) rho0 * exp(-z / h_scale); % Exponential atmosphere model (z is altitude)

drag_fn = @(t, z, vz, vx) compute_drag_and_alpha(t, z, vz, vx, v_w, rho, A, Cd);

%%  INTEGRATION

% State vector = [x; z; vx; vz; m; v_loss_gravity]

x0 = [0; 0; 0; 0; m0; 0];
tspan = [0, 1000];

options = odeset('Events', @ground_hit_event); % Event to stop at ground hit

[t, x] = ode45(@(t, x) rocket_dynamics(t, x, thrust_fn, mdot_fn, drag_fn, g, m_dry), tspan, x0, options);

% Extract Maximum Altitude and Gravity Losses
z = x(:,2); % Altitude trajectory
[max_altitude, idx] = max(z);
max_time = t(idx);
v_loss = x(:,6); % Gravity loss trajectory
total_gravity_loss = v_loss(end); % Total gravity loss at end

% Compute Angle of Attack for Plotting
alpha = zeros(size(t));
for i = 1:length(t)
    vx = x(i,3); % Horizontal velocity
    vz = x(i,4); % Vertical velocity

    v_rocket_mag = sqrt(vx^2 + vz^2);
    v_r = [vx - v_w; vz];
    v_rel_mag = norm(v_r);

    if v_rocket_mag == 0
        if v_w ~= 0
            alpha(i) = pi/2;
        else
            alpha(i) = 0;
        end
    elseif v_rel_mag == 0
        alpha(i) = 0;
    else
        dot_product = vx * (vx - v_w) + vz * vz;
        alpha(i) = acos(dot_product / (v_rocket_mag * v_rel_mag));
    end
end

alpha_deg = rad2deg(alpha); % Convert to degrees for plotting

%% OUTPUT AND PLOTTING

%  Output Results 
fprintf('Maximum Altitude: %.2f meters at t = %.2f seconds\n', max_altitude, max_time);
fprintf('Total Gravity Loss: %.2f m/s\n', total_gravity_loss);
fprintf('Downrange Distance at Max Altitude: %.2f meters\n', x(idx,1));

% Plot Results 
figure()
subplot(2,3,1)
plot(x(:,1), x(:,2))
xlabel('Downrange Distance (m)')
ylabel('Altitude (m)')
title('Rocket Trajectory')
grid on

subplot(2,3,2)
plot(t, x(:,2))
xlabel('Time (s)')
ylabel('Altitude (m)')
title('Altitude vs Time')
grid on

subplot(2,3,3)
plot(t, x(:,4))
xlabel('Time (s)')
ylabel('Vertical Velocity (m/s)')
title('Vertical Velocity vs Time')
grid on

subplot(2,3,4)
plot(t, x(:,3))
xlabel('Time (s)')
ylabel('Horizontal Velocity (m/s)')
title('Horizontal Velocity vs Time')
grid on

subplot(2,3,5)
plot(t, x(:,5))
xlabel('Time (s)') 
ylabel('Mass (kg)')
title('Mass vs Time')
grid on

subplot(2,3,6)
plot(t, v_loss)
xlabel('Time (s)')
ylabel('Gravity Loss (m/s)')
title('Cumulative Gravity Loss vs Time')
grid on

figure()
plot(t, alpha_deg)
xlabel('Time (s)')
ylabel('Angle of Attack (deg)')
title('Angle of Attack vs Time')
grid on

%% FUNCTION DEFINITIONS

%  Wind Effect and Drag Function
function [D, alpha] = compute_drag_and_alpha(t, z, vz, vx, v_w, rho, A, Cd)
    % Compute relative velocity and angle of attack

    v_r = [vx - v_w; vz]; % Relative velocity vector (rocket velocity - wind)
    v_rel_mag = norm(v_r);

    v_rocket_mag = sqrt(vx^2 + vz^2);

    if v_rocket_mag == 0
        if v_w ~= 0
            alpha = pi/2;
        else
            alpha = 0;
        end
    elseif v_rel_mag == 0
        alpha = 0;
    else
        % Angle between rocket velocity and relative wind
        dot_product = vx * (vx - v_w) + vz * vz;
        alpha = acos(dot_product / (v_rocket_mag * v_rel_mag));
    end
    
    D_mag = 0.5 * rho(z) * v_rel_mag^2 * A * Cd; % Drag magnitude
    if v_rel_mag == 0
        D = [0; 0]; % No drag if no relative velocity
    else
        D = -D_mag * v_r / v_rel_mag; % Drag opposes relative velocity
    end
end

% Equations of Motion
function dxdt = rocket_dynamics(t, x, thrust_fn, mdot_fn, drag_fn, g, m_dry)

    x_pos = x(1);  % Horizontal position [m]
    z = x(2);      % Altitude [m]
    vx = x(3);     % Horizontal velocity [m/s]
    vz = x(4);     % Vertical velocity [m/s]
    m = x(5);      % Mass [kg]
    v_loss = x(6); % Cumulative gravity loss [m/s]
    
    % Thrust
    T_mag = thrust_fn(t);
    v_mag = sqrt(vz^2 + vx^2); % Rocket velocity magnitude
    if v_mag == 0
        T = [0; T_mag]; % Thrust vertical at launch
    else
        T = T_mag * [vx; vz] / v_mag; % Thrust along velocity vector
    end
    
    % Drag and angle of attack
    [D, ~] = drag_fn(t, z, vz, vx);
    
    % Mass flow rate
    mdot = mdot_fn(t);
    
    % Prevent mass from going below dry mass
    if m <= m_dry
        mdot = 0;
        T = [0; 0];
        m = m_dry;
    end
    
    % Gravity loss rate (m/s per second, only during thrust)
    if T_mag > 0
        theta = atan2(vx, vz); % Trajectory angle from vertical
        g_loss = g * cos(theta); % Gravity loss along thrust direction
    else
        g_loss = 0; % No gravity loss after burnout
    end

    dxdt = [vx; vz; (T(1) + D(1)) / m; (T(2) + D(2) - m * g) / m; -mdot; g_loss];
end

function [value, isterminal, direction] = ground_hit_event(t, x)
    % Function to detect when the rocket hits the ground (z = 0)
    %
    % Inputs:
    %   t - current time (not used)
    %   x - current state vector
    % Outputs:
    %   value      - value to be zero (altitude)
    %   isterminal - 1 to stop the integration
    %   direction  - -1 to detect only decreasing altitude

    value = x(2);      % Detect when altitude = 0
    isterminal = 1;    % Stop the integration
    direction = -1;    % Trigger only when altitude is decreasing
end

function Me = get_Me(epsilon, gamma)
% Calculates the exit Mach number for a given expansion ratio.
%
% Inputs:
%   epsilon - Nozzle expansion ratio (Ae/At)
%   gamma   - Ratio of specific heats
%
% Outputs:
%   Me      - Mach number at the exit of the nozzle
    
    options = optimoptions("fsolve", "Display", "none");

    func = @(M) -epsilon + (1 ./ M) .* ((2 + (gamma - 1) .* M.^2) ./ (gamma + 1)) .^ ((gamma + 1) ./ (2.*(gamma-1)));

    Me = fsolve(func, 2, options);

end