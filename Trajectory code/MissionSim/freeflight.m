function dstate_dt = freeflight(t, state, rocket)

    %% Extract Rocket and State parameters

    g = 9.81; % [m/s^2]
    rho_SL = 1.225; %! Might want to change to ISA

    I = rocket.get_inertia(t);
    mass = rocket.get_total_mass(t);

    thrust = rocket.get_thrust(t); % 3x1 vector in the body frame. z axis is the rocket axis of symmetry
    F_g = [0; 0; -mass * g];

    % TODO: Change aero with aero model once it is finished
    % Aerodynamic Properties - Assumed for the moment
    ref_area = 0.0182; % Reference area for aerodynamic coefficients (m^2)
    ref_length = 3.0;  % Reference length for aerodynamic moments (m)
    % Simple aerodynamic model: Coefficients are assumed constant for this example.
    % In a real simulation, these would be functions of Mach number, AoA, etc.
    Ca = 0.3;   % Axial force coefficient (along body x-axis)
    Cn = 2.0;   % Normal force coefficient (along body z-axis)
    Cy = 0;     % Side force coefficient (along body y-axis)
    Cl = 0;     % Roll moment coefficient
    Cm = -5.0;  % Pitch moment coefficient
    Cn_yaw = 0; % Yaw moment coefficient
    xcp = 2.0;  % Distance from nose cone tip to center of pressure (m)
    xcg = 1.8;  % Distance from nose cone tip to center of gravity (m)

    % Unpack state vector
    r_vec = state(1:3);        % Position (x, y, z)
    v_vec = state(4:6);        % Velocity (vx, vy, vz)
    q = state(7:10);           % Quaternion (q0, q1, q2, q3)
    omega_body = state(11:13); % Angular velocity (p, q ,r)

    q = q / norm(q);

    p = omega_body(1);
    q_rate = omega_body(2);
    r = omega_body(3);

    %% Intermediate variables

    C_i2b = quat2dcm(q'); % Rotation Matrix for inertial to body
    vel_body = C_i2b * v_vec;

    % Calculate angle of attack (alpha) and sideslip angle (beta)
    % Note: Assumes no wind. For wind, use v_aero = vel_body - v_wind_body
    v_aero_body = vel_body;
    if norm(v_aero_body) < 1e-6
        alpha = 0;
        beta = 0;
    else
        alpha = atan2(v_aero_body(3), v_aero_body(1));
        beta = asin(v_aero_body(2) / norm(v_aero_body));
    end

    % Dynamic pressure
    q_dyn = 0.5 * rho_SL * norm(v_aero_body)^2;

    %% Forces and Moments (Body RF)

    % Aerodynamic forces
    F_axial = -q_dyn * ref_area * Ca;         % Along -x body axis
    F_normal = q_dyn * ref_area * Cn * alpha; % Along +z body axis
    F_side = q_dyn * ref_area * Cy * beta;    % Along +y body axis
    F_aero = [F_axial; F_side; F_normal];

    F_total_body = F_aero + thrust;

    % Aerodynamic moments
    roll_mom = q_dyn * ref_area * ref_length * Cl;
    pitch_mom = q_dyn * ref_area * ref_length * Cm * alpha;
    yaw_mom = q_dyn * ref_area * ref_length * Cn_yaw * beta;

    Moments_body = [roll_mom; pitch_mom; yaw_mom];

    %% Accelerations & Quaternion derivative

    C_b2i = C_i2b'; % From body to inertial
    F_total_inertial = C_b2i * F_total_body;

    pos_dot = v_vec;
    v_dot = (F_total_inertial + F_g) / mass;

    omega_cross_I_omega = cross(omega_body, I * omega_body);
    omega_dot = I \ (Moments_body - omega_cross_I_omega);

    % Quaternion derivative
    Omega = [ 0, -p, -q_rate, -r;
              p,  0,  r, -q_rate;
              q_rate, -r,  0,  p;
              r,  q_rate, -p,  0];

    q_dot = 0.5 * Omega * q;
    
    dstate_dt = [pos_dot; v_dot; q_dot; omega_dot];

end
