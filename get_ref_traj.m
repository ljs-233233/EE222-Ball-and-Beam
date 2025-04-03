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
    % amplitude = 0.04; % m
    % period = 10; % sec
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
    % n_changes = 10;
    % amp_ratio = ceil(t/90 * n_changes)/n_changes;
    % period_ratio = floor(t/90 * n_changes)/n_changes;
    % 
    % period = 10 - 4*period_ratio;
    % 
    % omega = 2 * pi / period;
    % 
    % if mod(amp_ratio * n_changes,2) == 1
    %     p_ref = amp_ratio * 0.15 * sin(omega * t);
    %     v_ref = amp_ratio * 0.15 * omega * cos(omega * t);
    %     a_ref = - amp_ratio * 0.15 * omega^2 * sin(omega * t);
    % else
    %     p_ref = amp_ratio * 0.1 * sign(sin(omega * t));
    %     v_ref = 0;
    %     a_ref = 0;
    % end

    %% Random Tester
    % Persistent variables to keep track of the current segment.
    persistent t_segment_start segment_duration segment_type amplitude omega
    
    % On the very first call, initialize the segment parameters.
    if isempty(t_segment_start)
        t_segment_start = 0;
        [segment_type, amplitude, segment_duration] = generateSegment();
        omega = 2*pi/segment_duration;
    end
    
    % If the query time t exceeds the current segment duration, update segments.
    while t > t_segment_start + segment_duration
        t_segment_start = t_segment_start + segment_duration;
        [segment_type, amplitude, segment_duration] = generateSegment();
        omega = 2*pi/segment_duration;
    end
    
    % Local time within the current segment.
    tau = t - t_segment_start;
    
    % Compute the reference trajectory based on the segment type.
    switch segment_type
        case 'sine'
            p_ref = amplitude * sin(omega * tau);
            v_ref = amplitude * omega * cos(omega * tau);
            a_ref = -amplitude * omega^2 * sin(omega * tau);
        case 'square'
            p_ref = amplitude * sign(sin(omega * tau));
            % For the square wave, assume zero velocity and acceleration (except at discontinuities)
            v_ref = 0;
            a_ref = 0;
        otherwise
            error('Unknown segment type.');
    end
    
    end
    
    function [segment_type, amplitude, segment_duration] = generateSegment()
    % GENERATESEGMENT Randomly generates the waveform type and its parameters.
    %
    %   For a sine wave:
    %       - amplitude is in [0, 0.15] m (15 cm)
    %   For a square wave:
    %       - amplitude is in [0, 0.10] m (10 cm)
    %
    %   The period (segment_duration) is uniformly chosen between 6 and 10 seconds.
    
    if rand < 0.5
        segment_type = 'sine';
        amplitude = 0.15 * rand;  % amplitude between 0 and 0.15 m
    else
        segment_type = 'square';
        amplitude = 0.10 * rand;  % amplitude between 0 and 0.10 m
    end
    
    segment_duration = 6 + 4 * rand;  % period between 6 and 10 seconds
end