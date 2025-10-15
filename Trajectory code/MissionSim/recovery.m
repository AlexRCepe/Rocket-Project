function dstate_dt = recovery(t, state, rocket, Cd_p, Ap, wind_model)

    %% Extract Rocket and State parameters

    g = 9.81;       % [m/s^2]
    rho_SL = 1.225; % [kg/m^3]

    mass = rocket.get_total_mass(t);

    F_g = [0; 0; -mass * g];

    % Unpack state vector
    pos_inertial = state(1:3);
    vel_inertial = state(4:6);      % Velocity (vx, vy, vz) in inertial frame
    q = state(7:10);                % Quaternion (q0, q1, q2, q3)
    omega_body = state(11:13);      % Angular velocity (p, q, r) in body frame

    p = omega_body(1);
    q_rate = omega_body(2);
    r = omega_body(3);

    %% --- 2. Calculate Forces in Inertial Frame ---

    [vx_wind, vy_wind] = wind_model.wind_at_h(pos_inertial(3));

    vel_relative = vel_inertial - [vx_wind; vy_wind; 0];

    vel_rel_mag = norm(vel_relative);
    F_drag_inertial = -0.5 * rho_SL * Cd_p * Ap * vel_rel_mag * vel_relative;

    F_net_inertial = F_g + F_drag_inertial;

    %% Accelerations & Quaternion derivative

    pos_dot = vel_inertial;
    v_dot = F_net_inertial / mass;

    % Rotational motion is unforced
    omega_dot = [0; 0; 0];

    Omega = [ 0, -p, -q_rate, -r;
              p,  0,  r, -q_rate;
              q_rate, -r,  0,  p;
              r,  q_rate, -p,  0];

    q_dot = 0.5 * Omega * q;

    dstate_dt = [vel_inertial; v_dot; q_dot; omega_dot];

end
