clc
clear
close all

%% Create a new rocket object
myRocket = Rocket("Rocket");

noseCone = NoseCone('MyNoseCone', 'ogive', 0.28, 0.104, 0.107, []);
myRocket = myRocket.add_part(noseCone);

bodyTube1 = BodyTube('MainBodyTube', 6.104, 0.104, 0.676, 0.002);
myRocket = myRocket.add_part(bodyTube1);

% parachute = Parachute('MainParachute', 1.5, 0.8, 300, 0.1, 0.5, bodyTube1);
% parachute = parachute.update_position([0, 0, 0.1]); % Place it 10cm from the top of the tube
% myRocket = myRocket.add_part(parachute);

% avionics = Mass('AvionicsBay', 0.4, 0.15, 0.09, bodyTube1);
% avionics = avionics.update_position([0, 0, 0.3]); % Place it 30cm from the top of the tube
% myRocket = myRocket.add_part(avionics);

% motorTube = BodyTube('MotorTube', 0.5, 0.05, 0.7, 0.002);
% myRocket = myRocket.add_part(motorTube);

thrust_curve = [0.0, 0.0; 0.003, 281.69; 0.05, 1436.62; 0.121, 1363.38; 0.366, 1263.85; 0.59, 1230.05; 1.745, 1322.07; 2.835, 1166.2; 4.0, 974.648; 4.158, 839.437; 4.668, 82.629; 4.736, 0.0];
motor = Motor('MyMotor', 4.938, 3.096, 0.065, 0.621, thrust_curve, bodyTube1);
motor = motor.update_position([0, 0, bodyTube1.length - motor.length]); % Place it 5cm from the top of the motor tube
myRocket = myRocket.add_part(motor);

finSet = FinSet('MyFins', 4, 0.0865, 0.17, 0.0425, 0.0978, 0.0007, 0.443/4, bodyTube1);
finSet = finSet.update_position([0, 0, 0.506]); % Attach fins starting 40cm down the motor tube
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