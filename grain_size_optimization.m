clc
clear
close all

%% CONSTANTS AND MOTOR PARAMETERS

gamma = 1.2;

De = 0.63;      % [in]
Dt = 0.25;     % [in]
c_star = 5088; % [ft/s]

epsilon = De.^2 ./ Dt.^2;

r1 = 0.4; % [in]

Me = get_me(epsilon, gamma);
Isp = get_Isp(r1, Me, epsilon, gamma, "Dt", Dt, "c_star", c_star);
fprintf("Initial Isp for r1=%.2f: %.2f s\n", r1, Isp);


%% FUNCTION DEFINITIONS

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
        options.DisplayForce = true;
    end

    [thrust_curve, mass_flow_curve] = get_curves(r, Me, epsilon, gamma, "Dt", options.Dt, "c_star", options.c_star);
    t = thrust_curve(:, 1);
    F_v = thrust_curve(:, 2);

    % Remove NaN values from F_v and corresponding time values
    validIdx = ~isnan(F_v);
    F_v = F_v(validIdx);
    t = t(validIdx);
    t_m_p = mass_flow_curve(validIdx, 1);
    mass_flow_curve = mass_flow_curve(validIdx, 2);

    m_p = trapz(t_m_p, mass_flow_curve);

    I = trapz(t, F_v);    % [lb*s]
    fprintf("Total Impulse for r=%.2f in: %.2f N*s\n", r, I);
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