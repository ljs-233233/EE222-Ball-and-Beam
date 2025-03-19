classdef studentControllerInterfaceIO < matlab.System
    properties (Access = private)
        %% You can add values that you want to store and updae while running your controller.
        % For more information of the supported data type, see
        % https://www.mathworks.com/help/simulink/ug/data-types-supported-by-simulink.html
        t_prev = -1;
        theta_d = 0;
        p_prev      = 0; % Ball position at last step
        theta_prev  = 0; % Servo angle at last step
        
        %% Gains for feedback
        k1 = 2.0;  
        k2 = 16.0;
        
        %% System parameters
        rg   = 0.0254;  % [m] Servo arm length
        L    = 0.4255;  % [m] Beam length
        g    = 9.81;    % [m/s^2] Gravity
        K    = 1.5;     % [rad/s/V] Motor constant
        tau  = 0.025;   % [s] Motor time constant
    end
    methods(Access = protected)
        % function setupImpl(obj)
        %    disp("You can use this function for initializaition.");
        % end

        function V_servo = stepImpl(obj, t, p_ball, theta)
        % This is the main function called every iteration. You have to implement
        % the controller in this function, bu you are not allowed to
        % change the signature of this function. 
        % Input arguments:
        %   t: current time
        %   p_ball: position of the ball provided by the ball position sensor (m)
        %
        %   theta: servo motor angle provided by the encoder of the motor (rad)
        % Output:
        %   V_servo: voltage to the servo input.        
            dt = t - obj.t_prev;
            if (obj.t_prev < 0) || (dt <= 0)
                % default
                dt = 0.01;  
            end
            
            p_dot_est     = (p_ball   - obj.p_prev)     / dt;  % ~ ż
            theta_dot_est = (theta    - obj.theta_prev) / dt;  % ~ θ̇
            
            [z_ref, zdot_ref, zddot_ref] = get_ref_traj(t);
            
            %% desired ball acceleration with PD feedback 
            %    on (z - z_ref) and (ż - ż_ref).
            e1 = (p_ball    - z_ref);
            e2 = (p_dot_est - zdot_ref);

            zddot_des = zddot_ref - obj.k1*e1 - obj.k2*e2;
            
            %% Approximate the ball dynamics to solve for motor voltage
            % simplified
            % ignoring cross terms with θ̇² or friction, etc.
            
            rg  = obj.rg;
            L   = obj.L;
            g   = obj.g;
            tau = obj.tau; 
            K   = obj.K;
            
            f_ball = (5*g/7)*sin(theta);
            g_ball = (5*g/7)*(rg/L)*cos(theta)*(K/tau);
            
            if abs(g_ball) < 1e-6
                V_servo = 0;
            else
                V_servo = (zddot_des - f_ball) / g_ball;
            end
            
            val = zddot_des * (7/(5*g));
            val = max(min(val, 1), -1);
            obj.theta_d = asin(val);
            
            obj.t_prev      = t;
            obj.p_prev      = p_ball;
            obj.theta_prev  = theta;
        end
    end
    
    methods(Access = public)
        % Used this for matlab simulation script. fill free to modify it as
        % however you want.
        function [V_servo, theta_d] = stepController(obj, t, p_ball, theta)        
            V_servo = stepImpl(obj, t, p_ball, theta);
            theta_d = obj.theta_d;
        end
    end
    
end
