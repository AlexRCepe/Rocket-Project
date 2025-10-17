clc
clear
close all

%% CONSTANTS AND MOTOR PARAMETERS

gamma = 1.2;

De = 1.5;      % [in]
Dt = 0.25;     % [in]
c_star = 5088; % [ft/s]

epsilon = De.^2 ./ Dt.^2;

r1 = 0.4; % [in]

Me = get_me(epsilon, gamma);
Isp = get_Isp(r1, Me, epsilon, gamma, "Dt", Dt, "c_star", c_star);
fprintf("Initial Isp for r1=%.2f: %.2f s\n", r1, Isp);


%% FUNCTION DEFINITIONS

function thrust_curve = get_thrust_curve(r, Me, epsilon, gamma, options)
% Calculates the thrust curve for a solid rocket motor with a star grain.
%
% Inputs:
%   r                - Initial radius parameter for the star points [in]
%   Me               - Mach number at the exit of the nozzle
%   epsilon          - Nozzle expansion ratio (Ae/At)
%   gamma            - Ratio of specific heats
%   options.Dt       - Throat diameter [in]
%   options.c_star   - Characteristic velocity [ft/s]
%
% Outputs:
%   thrust_curve     - A [Nx2] matrix of [time, thrust] data

    arguments
        r
        Me
        epsilon
        gamma
        options.Dt
        options.c_star
    end

    At = pi * options.Dt^2 / 4; % [in^2]

    [t, Pc, m_p] = project_grain(r, "Dt", options.Dt, "c_star", options.c_star);
    
    % Pe should be a vector, calculated for each value of Pc
    Pe = Pc ./ (1 + (gamma - 1)./2 .* Me.^2).^(gamma./(gamma-1));

    % Pressure ratio Pe/Pc for each time step
    pressure_ratio = Pe ./ Pc;

    cf_v = sqrt((2 .* gamma.^2 ./ (gamma - 1)) .* ((2 ./ (gamma + 1)).^((gamma + 1)./(gamma - 1))) .* ...
         (1 - pressure_ratio.^((gamma - 1)./gamma))) ...
         + (pressure_ratio).*epsilon;

    % Remove the last value from vectors to prevent NaN value in impulse calculation
    cf_v = cf_v(1:end-1);
    Pc = Pc(1:end-1);
    t = t(1:end-1);

    F_v = cf_v .* Pc .* At;
    
    thrust_curve = [t', F_v'];

    % Plot thrust curve
    figure()
    plot(t, F_v)
    title('Thrust vs. Time')
    xlabel('Time (s)')
    ylabel('Thrust (lbf)')
    grid on;

    % Plot chamber pressure curve
    figure()
    plot(t, Pc)
    title('Chamber Pressure vs. Time')
    xlabel('Time (s)')
    ylabel('Chamber Pressure (psi)')
    grid on;
end

function Isp = get_Isp(r, Me, epsilon, gamma, options)
% Calculates the specific impulse of a solid rocket motor with a star grain.
%
% Inputs:
%   r                - Initial radius parameter for the star points [in]
%   Me               - Mach number at the exit of the nozzle
%   epsilon          - Nozzle expansion ratio (Ae/At)
%   gamma            - Ratio of specific heats
%   options.Dt       - Throat diameter [in]
%   options.c_star   - Characteristic velocity [ft/s]
%   options.DisplayForce - Boolean flag to display the thrust curve plot
%
% Outputs:
%   Isp              - Specific Impulse [s]

    arguments
        r
        Me
        epsilon
        gamma
        options.Dt
        options.c_star
        options.DisplayForce = false
    end

    [~, ~, m_p] = project_grain(r, "Dt", options.Dt, "c_star", options.c_star);

    [thrust_curve, ~] = get_curves(r, Me, epsilon, gamma, "Dt", options.Dt, "c_star", options.c_star);
    t = thrust_curve(:, 1);
    F_v = thrust_curve(:, 2);

    I = trapz(t, F_v);    % [lb*s]
    Isp = I ./ (m_p .* 9.81);

    if options.DisplayForce

        figure()
        plot(t, F_v)
        title('Thrust vs. Time')
        xlabel('Time (s)')
        ylabel('Thrust (lbf)')
        grid on;
        
    end

end

function Me = get_me(epsilon, gamma)
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