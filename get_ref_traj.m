function [p_ref, v_ref, a_ref] = get_ref_traj(t)
% [p_ref, v_ref, a_ref] = get_ref_traj(t)
%% Reference trajectory at time t:
%   Inputs
%       t: query time
%   Outputs
%       p_ref: reference position of the ball
%       v_ref: reference velocity of the ball
%       a_ref: reference acceleration of the ball
    coder.allowpcode('plain');
    % amplitude = 0.1; % m
    % period = 6; % sec
    % 
    % omega = 2 * pi / period;    
    
    %% Sine wave.
    % p_ref = amplitude * sin(omega * t);
    % v_ref = amplitude * omega * cos(omega * t);
    % a_ref = - amplitude * omega^2 * sin(omega * t);
    %% Square wave.
    % p_ref = amplitude * sign(sin(omega * t));
    % v_ref = 0;
    % a_ref = 0;


    %% Tester
    n_changes = 10;
    amp_ratio = 1 - floor(t/90 * n_changes)/n_changes;
    period_ratio = floor(t/90 * n_changes)/n_changes;

    period = 6 + 4*period_ratio;

    omega = 2 * pi / period;

    if mod(round(amp_ratio,1) * n_changes,2) == 1
        p_ref = amp_ratio * 0.15 * sin(omega * t);
        v_ref = amp_ratio * 0.15 * omega * cos(omega * t);
        a_ref = - amp_ratio * 0.15 * omega^2 * sin(omega * t);
    else
        p_ref = amp_ratio * 0.1 * sign(sin(omega * t));
        v_ref = 0;
        a_ref = 0;
    end
end