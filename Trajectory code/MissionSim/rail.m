function dstate_dt = rail(t, state, rocket, rail_direction)

    %% Extract Rocket and State parameters

    g = 9.81;       % [m/s^2]
    rho_SL = 1.225; % [kg/m^3]

    I = rocket.get_inertia(t);
    mass = rocket.get_total_mass(t);

    thrust = rocket.get_thrust(t);
    F_g = [0; 0; -mass * g];

    % Aerodynamic Properties (placeholders, as in freeflight.m)
    ref_area = 0.0182; % Reference area [m^2]
    Ca = 0.3;          % Axial force coefficient
    % Note: Normal and side forces are generated but counteracted by the rail.
    % They are still needed to calculate the total force on the system.
    Cn = 2.0;          % Normal force coefficient
    Cy = 0;            % Side force coefficient

    % Unpack state vector
    r_vec = state(1:3);        % Position (x, y, z)
    v_vec = state(4:6);        % Velocity (vx, vy, vz)
    q = state(7:10);           % Quaternion (q0, q1, q2, q3)
    omega_body = state(11:13); % Angular velocity (p, q ,r)

    %% Intermediate variables

    C_i2b = quat2dcm(q'); % Rotation Matrix for inertial to body
    vel_body = C_i2b * v_vec;

    % Angle of attack (alpha) and sideslip (beta)
    if norm(vel_body) < 1e-6
        alpha = 0;
        beta = 0;
    else
        alpha = atan2(vel_body(3), vel_body(1));
        beta = asin(vel_body(2) / norm(vel_body));
    end

    % Dynamic pressure
    q_dyn = 0.5 * rho_SL * norm(vel_body)^2;

    %% Forces (Body RF)

    % Aerodynamic forces 
    F_axial = -q_dyn * ref_area * Ca;
    F_normal = q_dyn * ref_area * Cn * alpha;
    F_side = q_dyn * ref_area * Cy * beta;
    F_aero_body = [F_axial; F_side; F_normal];

    F_total_body = F_aero_body + thrust;

    %% Accelerations

    C_b2i = C_i2b'; % From body to inertial
    F_total_inertial = C_b2i * F_total_body;
    
    F_net_inertial = F_total_inertial + F_g;
    
    % Project the forces on the rail direction
    d_hat = rail_direction / norm(rail_direction);
    F_proj = dot(F_net_inertial, d_hat);

    % Acceleration along the rail
    accel_mag = F_proj / mass;

    % Position and velocity derivatives
    pos_dot = vel_inertial;
    v_dot = accel_mag * d_hat;

    % Rotational motion constrained by the rail
    ang_accel_body = [0; 0; 0];
    q_dot = [0; 0; 0; 0];      

    dstate_dt = [pos_dot; v_dot; q_dot; ang_accel_body];

end