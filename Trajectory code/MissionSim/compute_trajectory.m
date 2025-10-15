function [time, state] = compute_trajectory(rocket, options)
% Simulates the trajectory of a rocket through rail, free-flight, and recovery phases.
%
% Inputs:
%   rocket  - An instance of the Rocket class.
%   options - A struct with simulation parameters (rail_length, theta, phi, wind_model, etc.).
%
% Outputs:
%   time    - A column vector of time points (s).
%   state   - An N-by-13 matrix where each row corresponds to a time point and
%             columns represent the rocket's state (position, velocity, quaternion, angular velocity).
%

    arguments
        rocket
        options.rail_length
        options.theta % Rotation of the rail about the z-axis
        options.phi   % Inclination of the rail
        options.wind_model
        options.t_start = 0
        options.max_time = 300
    end

    %% PRE-CHECKS AND INITIALIZATION

    if isempty(options.wind_model)
        error("ERROR.\nA wind model must be defined in order to perform the simulations") 
    end
    
    time = [];
    state = [];

    %% INITIAL CONDITIONS 

    % Position, Velocity, Angular Velocity
    r0 = [0; 0; 0];
    v0 = [0; 0; 0];
    omega0 = [0; 0; 0];

    % Orientation from rail angles
    phi = options.phi;
    theta = options.theta;
    
    % Rail direction vector in inertial frame (this is the rocket's z-axis)
    xb_i = [cos(phi) * cos(theta); cos(phi) * sin(theta); sin(phi)];
    % Body y-axis (perpendicular to rocket axis and horizontal)
    yb_i = [-sin(theta); cos(theta); 0];
    % Body x-axis
    zb_i = cross(yb_i, xb_i);
    
    
    C_b2i = [xb_i, yb_i, zb_i]; % Rotation from body to inertial
    C_i2b = C_b2i';             % Rotation from inertial to body
    
    % Initial quaternion [q0, q1, q2, q3] (w, x, y, z)
    q0 = dcm2quat(C_i2b);
    
    state0 = [r0; v0; q0'; omega0];

    %% RAIL
    
    fprintf('Simulating rail phase...\n');

    tspan_rail = [options.t_start, options.max_time];
    ode_options_rail = odeset('Events', @(t, y) railExitEvent(t, y, options.rail_length));

    
    [t1, s1, te1, ye1, ie1] = ode45(@(t,y) rail(t, y, rocket, xb_i), tspan_rail, state0, ode_options_rail);
    
    time = [time; t1];
    state = [state; s1];
    
    if isempty(ie1)
        fprintf('Warning: Rocket did not leave the rail.\n');
        return;
    end
    
    %% FREE-FLIGHT
    
    fprintf('Simulating free flight phase...\n');
    
    t_start_ff = te1(end);
    state0_ff = ye1(end, :)';

    tspan_ff = [t_start_ff, options.max_time];
    ode_options_ff = odeset('Events', @apogeeEvent);
    
    [t2, s2, te2, ye2, ie2] = ode45(@(t,y) freeflight(t, y, rocket), tspan_ff, state0_ff, ode_options_ff);
    
    time = [time; t2(2:end)];
    state = [state; s2(2:end, :)];
    
    if isempty(ie2)
        fprintf('Warning: Rocket did not reach apogee.\n');
        return;
    end
    
    %% RECOVERY
    
    fprintf('Simulating recovery phase...\n');

    t_start_rec = te2(end);
    state0_rec = ye2(end, :)';

    tspan_rec = [t_start_rec, options.max_time];
    ode_options_rec = odeset('Events', @groundHitEvent);

    if ~isempty(rocket.parachute) % There is a parachute for recovery

        odefun_recovery = @(t, y) recovery(t, y, rocket, rocket.parachute.drag_coefficient, pi*(rocket.parachute.diameter/2)^2, options.wind_model);

    else % No parachute -> continue in freeflight

        odefun_recovery = @(t, y) freeflight(t, y, rocket);

    end
    
    [t3, s3, te3, ye3, ~] = ode45(odefun_recovery, tspan_rec, state0_rec, ode_options_rec);
    
    % Append the recovery phase trajectory up to the event
    time = [time; t3(2:end)];
    state = [state; s3(2:end, :)];

    fprintf('Simulation finished.\n');

end

% Event functions

function [value, isterminal, direction] = railExitEvent(t, y, rail_length)
    % Event triggers when the rocket has traveled the length of the rail

    value = norm(y(1:3)) - rail_length;
    isterminal = 1; % Stop integration
    direction = 1;  % Trigger when value is increasing

end

function [value, isterminal, direction] = apogeeEvent(~, y)
    % Event triggers at apogee (vertical velocity is zero)

    value = y(6);
    isterminal = 1; % Stop integration
    direction = -1; % Trigger when value is decreasing (i.e., at a peak)
end

function [value, isterminal, direction] = groundHitEvent(~, y)
    % Event triggers when the rocket hits the ground (altitude is zero)

    value = y(3);
    isterminal = 1; % Stop integration
    direction = -1; % Trigger when approaching from above
end