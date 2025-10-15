clc
clear
close all

%% Create a new rocket object
myRocket = Rocket("Rocket");

noseCone = NoseCone('MyNoseCone', 'ogive', 0.5, 0.1, 1.0, []);
myRocket = myRocket.add_part(noseCone);

bodyTube1 = BodyTube('MainBodyTube', 0.5, 0.1, 1.2, 0.003);
myRocket = myRocket.add_part(bodyTube1);

parachute = Parachute('MainParachute', 1.5, 0.8, 300, 0.1, 0.5, bodyTube1);
parachute = parachute.update_position([0, 0, 0.1]); % Place it 10cm from the top of the tube
myRocket = myRocket.add_part(parachute);

avionics = Mass('AvionicsBay', 0.4, 0.15, 0.09, bodyTube1);
avionics = avionics.update_position([0, 0, 0.3]); % Place it 30cm from the top of the tube
myRocket = myRocket.add_part(avionics);

motorTube = BodyTube('MotorTube', 0.5, 0.05, 0.7, 0.002);
myRocket = myRocket.add_part(motorTube);


thrust_curve = [0.0, 0.0; 0.003, 281.69; 0.05, 1436.62; 0.121, 1363.38; 0.366, 1263.85; 0.59, 1230.05; 1.745, 1322.07; 2.835, 1166.2; 4.0, 974.648; 4.158, 839.437; 4.668, 82.629; 4.736, 0.0];
motor = Motor('MyMotor', 5.0, 2.8, 0.03, 0.6, thrust_curve, motorTube);
motor = motor.update_position([0, 0, 0.05]); % Place it 5cm from the top of the motor tube
myRocket = myRocket.add_part(motor);

finSet = FinSet('MyFins', 4, 0.15, 0.2, 0.1, 0.05, 0.004, 0.1, motorTube);
finSet = finSet.update_position([0, 0, 0.4]); % Attach fins starting 40cm down the motor tube
myRocket = myRocket.add_part(finSet);

% Debug calculations
time = 0;
total_mass = myRocket.get_total_mass(time);
cg = myRocket.get_cg(time);
inertia = myRocket.get_inertia(time);

fprintf('Rocket: %s\n', myRocket.rocketName);
fprintf('Total Mass at t=%.1f s: %.2f kg\n', time, total_mass);
fprintf('Center of Gravity (from nose tip) at t=%.1f s: %.2f m\n', time, cg);
fprintf('Inertia Tensor at t=%.1f s (kg*m^2):\n', time);
disp(inertia);

%% Simulation Parameters

% Wind Profile
wind_model = PowerLawWindProfile();

altitudes = linspace(0, 1000, 200);
wind_vectors = zeros(length(altitudes), 2);

for i = 1:length(altitudes)
    wind_vectors(i, :) = wind_model.wind_at_h(altitudes(i));
end

% Plot the wind vector components in 3D
figure('Name', '3D Wind Profile');
plot3(wind_vectors(:,1), wind_vectors(:,2), altitudes, "b", 'LineWidth', 2, 'DisplayName', 'Wind Vector Tip');
hold on; % Keep the line plot

% Add arrows to the plot to show vector direction
arrow_indices = 1:20:length(altitudes); % Show an arrow every 20 points
quiver3(zeros(1, length(arrow_indices)), ... % Start arrows from X=0 (as row vector)
    zeros(1, length(arrow_indices)), ... % Start arrows from Y=0 (as row vector)
    altitudes(arrow_indices), ...        % Start arrows at different altitudes (already a row vector)
    wind_vectors(arrow_indices, 1)', ... % X-component of wind (transposed to row vector)
    wind_vectors(arrow_indices, 2)', ... % Y-component of wind (transposed to row vector)
    zeros(1, length(arrow_indices)), ... % Z-component is zero (as row vector)
    'b', 'LineWidth', 2, 'DisplayName', 'Wind Vectors', 'AutoScale', 'off', 'MaxHeadSize', 0.001);

set(gca, 'TickLabelInterpreter', 'latex');
xlabel('Wind $V_x$ (m/s)', 'Interpreter', 'latex', 'FontSize', 15);
ylabel('Wind $V_y$ (m/s)', 'Interpreter', 'latex', 'FontSize', 15);
zlabel('Altitude (m)', 'Interpreter', 'latex', 'FontSize', 15);
title('3D Wind Profile (0 to 1 km)', 'Interpreter', 'latex', 'FontSize', 20);
grid on;
view(45, 30); % Adjust view for better 3D perspective
legend('Interpreter', 'latex', 'Location', 'best');
hold off;

theta = deg2rad(45); % Launch rail elevation angle from horizontal
phi = deg2rad(85);   % Launch rail azimuth angle from North

[time, state] = compute_trajectory(myRocket, ...
                                   "rail_length", 5.0, ...
                                   "theta", theta, ...
                                   "phi", phi, ...
                                   "wind_model", wind_model, ...
                                   "max_time", 600);

% Plot Trajectory
figure('Name', 'Rocket Trajectory');
plot3(state(:,1), state(:,2), state(:,3), 'LineWidth', 2);
grid on;
hold on;

% Mark apogee
[max_alt, apogee_idx] = max(state(:,3));
plot3(state(apogee_idx, 1), state(apogee_idx, 2), max_alt, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
text(state(apogee_idx, 1), state(apogee_idx, 2), max_alt, sprintf(' Apogee: %.1f m', max_alt), 'Interpreter', 'latex', 'VerticalAlignment', 'bottom');

% Mark landing point
landing_pos = state(end, 1:3);
plot3(landing_pos(1), landing_pos(2), landing_pos(3), 'kx', 'MarkerSize', 10, 'LineWidth', 2);
text(landing_pos(1), landing_pos(2), landing_pos(3), ' Landing', 'Interpreter', 'latex', 'VerticalAlignment', 'bottom');

% Mark the launch point
plot3(state(1,1), state(1,2), state(1,3), 'g^', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
text(state(1,1), state(1,2), state(1,3), ' Launch', 'Interpreter', 'latex', 'VerticalAlignment', 'bottom');

% Draw the launch rail
rail_length = 5.0; % meters
rail_end = [rail_length * cos(phi) * cos(theta), ...
            rail_length * cos(phi) * sin(theta), ...
            rail_length * sin(phi)];
plot3([0, rail_end(1)], [0, rail_end(2)], [0, rail_end(3)], 'm--', 'LineWidth', 2, 'DisplayName', 'Launch Rail');
legend('Trajectory', 'Apogee', 'Landing', 'Launch', 'Launch Rail', 'Interpreter', 'latex', 'Location', 'best');

axis equal;
xlabel('X (m)', 'Interpreter', 'latex', 'FontSize', 15);
ylabel('Y (m)', 'Interpreter', 'latex', 'FontSize', 15);
zlabel('Altitude (m)', 'Interpreter', 'latex', 'FontSize', 15);
title('Rocket Trajectory', 'Interpreter', 'latex', 'FontSize', 20);
set(gca, 'TickLabelInterpreter', 'latex');
view(45, 25);

% Plot Velocities
figure('Name', 'Velocities vs. Time');
plot(time, state(:,4), 'r', 'LineWidth', 2, 'DisplayName', '$V_x$');
hold on;
plot(time, state(:,5), 'g', 'LineWidth', 2, 'DisplayName', '$V_y$');
plot(time, state(:,6), 'b', 'LineWidth', 2, 'DisplayName', '$V_z$');
grid on;
hold off;
xlabel('Time (s)', 'Interpreter', 'latex', 'FontSize', 15);
ylabel('Velocity (m/s)', 'Interpreter', 'latex', 'FontSize', 15);
title('Inertial Velocities vs. Time', 'Interpreter', 'latex', 'FontSize', 20);
legend('Interpreter', 'latex', 'Location', 'best');
set(gca, 'TickLabelInterpreter', 'latex');

% Plot Angular Velocities
figure('Name', 'Angular Velocities vs. Time');
plot(time, rad2deg(state(:,11)), 'r', 'LineWidth', 2, 'DisplayName', '$p$ (roll)');
hold on;
plot(time, rad2deg(state(:,12)), 'g', 'LineWidth', 2, 'DisplayName', '$q$ (pitch)');
plot(time, rad2deg(state(:,13)), 'b', 'LineWidth', 2, 'DisplayName', '$r$ (yaw)');
grid on;
hold off;
xlabel('Time (s)', 'Interpreter', 'latex', 'FontSize', 15);
ylabel('Angular Velocity (deg/s)', 'Interpreter', 'latex', 'FontSize', 15);
title('Body Angular Velocities vs. Time', 'Interpreter', 'latex', 'FontSize', 20);
legend('Interpreter', 'latex', 'Location', 'best');
set(gca, 'TickLabelInterpreter', 'latex');

% Rocket and Environment Parameters
g = 9.81; % Gravity (m/s^2)
Isp = 200; % Specific impulse (s, user-provided)
m0 = 10; % Initial mass (kg)
m_dry = 2; % Dry mass (kg)
A = 0.01; % Cross-sectional area (m^2)
Cd = 0.75; % Drag coefficient
rho0 = 1.225; % Sea-level air density (kg/m^3)
h_scale = 8400; % Atmospheric scale height (m)
v_w = 40 / 3.6; % Wind speed (30 km/h converted to m/s, user-provided, in +x direction)
%max Thrust is 54 lbf at liftoff

% Thrust history [time (s), thrust (N)]
thrust_history = [0, 1000; 1, 1000; 2, 800; 3, 600; 4, 400; 5, 200; 6, 0];

% Thrust Interpolation Function 
thrust_fn = @(t) interp1(thrust_history(:,1), thrust_history(:,2), t, 'linear', 0);

% Mass Flow Rate (from Isp and thrust)
ve = Isp * g; % Exhaust velocity (m/s)
mdot_fn = @(t) thrust_fn(t) / ve; % Mass flow rate (kg/s)

% Air Density Model 
rho = @(z) rho0 * exp(-z / h_scale); % Exponential atmosphere model (z is altitude)



%  Integration 
% Initial conditions: [altitude, x-position, vz, vx, mass, gravity loss]
x0 = [0; 0; 0; 0; m0; 0];

% Time span (based on thrust history duration)
tspan = [thrust_history(1,1), thrust_history(end,1) + 20]; % Extend past burn for coast

% Create drag function with fixed parameters
drag_fn = @(t, z, vz, vx) compute_drag_and_alpha(t, z, vz, vx, v_w, rho, A, Cd);

% Solve ODE
[t, x] = ode45(@(t, x) rocket_dynamics(t, x, thrust_fn, mdot_fn, drag_fn, g, m_dry), tspan, x0);

%  Extract Maximum Altitude and Gravity Losses 
z = x(:,1); % Altitude trajectory
[max_altitude, idx] = max(z);
max_time = t(idx);
v_loss = x(:,6); % Gravity loss trajectory
total_gravity_loss = v_loss(end); % Total gravity loss at end

% Compute Angle of Attack for Plotting 
alpha = zeros(size(t));
for i = 1:length(t)
    vz = x(i,3); % Vertical velocity
    vx = x(i,4); % Horizontal velocity
    
    if vz == 0 && vx == 0
        alpha(i) = pi/2; % 90 deg when stationary
    else
        alpha(i) = atan2(abs(v_w), vz); % Angle of attack (radians)
    end
end
alpha_deg = rad2deg(alpha); % Convert to degrees for plotting

%  Output Results 
fprintf('Maximum Altitude: %.2f meters at t = %.2f seconds\n', max_altitude, max_time);
fprintf('Total Gravity Loss: %.2f m/s\n', total_gravity_loss);
fprintf('Downrange Distance at Max Altitude: %.2f meters\n', x(idx,2));

% Plot Results 
figure(1)
subplot(2,3,1)
plot(x(:,2), x(:,1))
xlabel('Downrange Distance (m)')
ylabel('Altitude (m)')
title('Rocket Trajectory')
grid on

subplot(2,3,2)
plot(t, x(:,1))
xlabel('Time (s)')
ylabel('Altitude (m)')
title('Altitude vs Time')
grid on

subplot(2,3,3)
plot(t, x(:,3))
xlabel('Time (s)')
ylabel('Vertical Velocity (m/s)')
title('Vertical Velocity vs Time')
grid on

subplot(2,3,4)
plot(t, x(:,4))
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

figure(2)
plot(t, alpha_deg)
xlabel('Time (s)')
ylabel('Angle of Attack (deg)')
title('Angle of Attack vs Time')
grid on



%  Wind Effect and Drag Function
function [D, alpha] = compute_drag_and_alpha(t, z, vz, vx, v_w, rho, A, Cd)
    % Compute relative velocity and angle of attack
    v_r = [vx - v_w; vz]; % Relative velocity vector (rocket velocity - wind)
    v_rel = sqrt(v_r(1)^2 + v_r(2)^2); % Magnitude of relative velocity
    if v_rel == 0
        alpha = pi/2; % 90 deg when rocket is stationary
    else
        alpha = atan2(abs(v_w), vz); % Angle of attack (wind relative to vertical)
    end
    D_mag = 0.5 * rho(z) * v_rel^2 * A * Cd; % Drag magnitude
    if v_rel == 0
        D = [0; 0]; % No drag if no relative velocity
    else
        D = -D_mag * v_r / v_rel; % Drag opposes relative velocity
    end
end

% Equations of Motion
function dxdt = rocket_dynamics(t, x, thrust_fn, mdot_fn, drag_fn, g, m_dry)
    z = x(1); % Altitude (m)
    x_pos = x(2); % Horizontal position (m)
    vz = x(3); % Vertical velocity (m/s)
    vx = x(4); % Horizontal velocity (m/s)
    m = x(5); % Mass (kg)
    v_loss = x(6); % Cumulative gravity loss (m/s)
    
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
    
    % State derivatives: dz/dt = vz, dx/dt = vx, dvz/dt, dvx/dt, dm/dt, dv_loss/dt
    dxdt = [vz; vx; (T(2) + D(2) - m * g) / m; (T(1) + D(1)) / m; -mdot; g_loss];
end