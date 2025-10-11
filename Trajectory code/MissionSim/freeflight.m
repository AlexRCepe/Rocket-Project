function dstate_dt = freeflight(t, state)
% freeflight Computes the derivative of the state vector for a rocket in 6-DOF free flight.
%
% REQUIRES: MATLAB Aerospace Toolbox
%
% This function models the translational and rotational dynamics of a rocket.
% It is designed to be used with a MATLAB ODE solver (e.g., ode45).
%
% INPUTS:
%   t     - Current simulation time (not used directly but required by ODE solvers).
%   state - 13x1 state vector at the current time t:
%           state(1:3)   - Position [x; y; z] in inertial frame (m)
%           state(4:6)   - Velocity [vx; vy; vz] in inertial frame (m/s)
%           state(7:10)  - Orientation quaternion [q0; q1; q2; q3] (scalar-first)
%                          representing rotation from body to inertial frame.
%           state(11:13) - Angular velocity [p; q; r] in body frame (rad/s)
%
% OUTPUT:
%   dstate_dt - 13x1 vector of the derivatives of the state vector.

%% --- 1. Placeholder Rocket & Environment Parameters ---
% !!! REPLACE THESE VALUES WITH YOUR ROCKET'S ACTUAL DATA !!!

% Mass Properties
m = 50; % Mass (kg) - This should be updated if mass changes over time
Ixx = 0.05; % Moment of inertia about body x-axis (kg*m^2)
Iyy = 10;   % Moment of inertia about body y-axis (kg*m^2)
Izz = 10;   % Moment of inertia about body z-axis (kg*m^2)
Izx = 0.01; % Product of inertia (kg*m^2)
I = [Ixx, 0, -Izx; 0, Iyy, 0; -Izx, 0, Izz]; % Inertia Tensor

% Thrust Properties
thrust_mag = 1000; % Thrust magnitude (N). Set to 0 if motor is off.
% For simplicity, thrust is aligned with the body x-axis.
% l_0 is the distance from CG to nozzle exit, needed for thrust moments.
l_0 = 1.5; % (m)
delta1 = 0; % Thrust angle in x-z plane (rad)
delta2 = 0; % Thrust angle in x-y plane (rad)

% Aerodynamic Properties
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

% Environment
g = 9.81;   % Gravitational acceleration (m/s^2)
rho = 1.225;% Air density (kg/m^3) - assuming constant for simplicity

%% --- 2. Deconstruct State Vector ---
pos_inertial = state(1:3);
vel_inertial = state(4:6);
quat = state(7:10);
omega_body = state(11:13);

% Normalize the quaternion to prevent drift
quat = quat / norm(quat);

p = omega_body(1);
q = omega_body(2);
r = omega_body(3);

%% --- 3. Calculate Intermediate Variables ---

% Rotate inertial velocity into body frame to find angles of attack
C_i2b = quat2dcm(quat'); % DCM for inertial to body
vel_body = C_i2b * vel_inertial;

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
q_dyn = 0.5 * rho * norm(v_aero_body)^2;

%% --- 4. Calculate Forces and Moments in Body Frame ---

% Aerodynamic forces
axial_force = -q_dyn * ref_area * Ca;  % Along -x body axis
normal_force = q_dyn * ref_area * Cn * alpha; % Along +z body axis
side_force = q_dyn * ref_area * Cy * beta;   % Along +y body axis
F_aero_body = [axial_force; side_force; normal_force];

% Thrust force (aligned with body x-axis)
F_thrust_body = [thrust_mag * cos(delta1)*cos(delta2); 
                 thrust_mag * sin(delta2); 
                 thrust_mag * sin(delta1)*cos(delta2)];

% Total force in body frame
F_total_body = F_aero_body + F_thrust_body;

% Aerodynamic moments
l = xcg - xcp; % Static margin
roll_mom = q_dyn * ref_area * ref_length * Cl;
pitch_mom = q_dyn * ref_area * ref_length * Cm * alpha;
yaw_mom = q_dyn * ref_area * ref_length * Cn_yaw * beta;

% Moments from thrust (assuming small angles)
M_thrust = -thrust_mag * l_0 * sin(delta1);
N_thrust = thrust_mag * l_0 * sin(delta2);

% Total moments in body frame
Moments_body = [roll_mom; pitch_mom + M_thrust; yaw_mom + N_thrust];

%% --- 5. Calculate Accelerations ---

% Rotate body forces to inertial frame
C_b2i = C_i2b'; % DCM for body to inertial
F_total_inertial = C_b2i * F_total_body;

% Gravitational force in inertial frame
F_gravity_inertial = [0; 0; -m * g];

% Linear acceleration in inertial frame (Newton's 2nd Law)
accel_inertial = (F_total_inertial + F_gravity_inertial) / m;

% Angular acceleration in body frame (Euler's Equations)
omega_cross_I_omega = cross(omega_body, I * omega_body);
ang_accel_body = I \ (Moments_body - omega_cross_I_omega);

%% --- 6. Calculate State Derivatives ---

d_pos = vel_inertial;
d_vel = accel_inertial;
d_omega = ang_accel_body;

% Quaternion derivative
% d(quat)/dt = 0.5 * quat * omega_pure_quat
omega_pure_quat = [0; omega_body];
d_quat = 0.5 * quatmultiply(quat', omega_pure_quat')';

%% --- 7. Assemble Output Vector ---
dstate_dt = [d_pos; d_vel; d_quat; d_omega];

end
