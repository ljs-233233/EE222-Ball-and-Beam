classdef pidControllerInterface < matlab.System
    properties (Access = private)
        %% You can add values that you want to store and updae while running your controller.
        % For more information of the supported data type, see
        % https://www.mathworks.com/help/simulink/ug/data-types-supported-by-simulink.html
        t_prev = -1;
        theta_d = 0;
        p_ball_prev = 0;
        integral_p = 0;
        theta_ball_prev = 0;
        integral_theta = 0;
    end
    methods(Access = protected)
        function setupImpl(obj, t, p_ball, theta)
        %    disp("You can use this function for initializaition.");
            obj.p_ball_prev = p_ball;
        end

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
            %% Sample Controller: Simple Proportional Controller
            % Extract reference trajectory at the current timestep.
            % Decide desired servo angle based on simple proportional feedback.

            % Make sure that the desired servo angle does not exceed the physical
            % limit. This part of code is not necessary but highly recommended
            % because it addresses the actual physical limit of the servo motor.

            % Simple position control to control servo angle to the desired
            % position.
            
            % Update class properties if necessary.
        
        [p_ball_ref, v_ball_ref, a_ball_ref] = get_ref_traj(t);
        dt = t - obj.t_prev;
        v_ball_est = (p_ball - obj.p_ball_prev) / dt;
        
        k_p_ball = 5;
        k_d_ball = 3;
        k_i_ball = 0;
        
        pos_error = p_ball_ref - p_ball;
        vel_error = v_ball_ref - v_ball_est;

        obj.integral_p = obj.integral_p + pos_error * dt;
        
        a_des = a_ball_ref + k_p_ball * pos_error + k_d_ball * vel_error + k_i_ball * obj.integral_p;
        
        g = 9.81;
        r_arm = 0.0254;
        L = 0.4255;
        a_param = 5 * g * r_arm / (7 * L);

        theta_d = a_des / a_param;
        
        theta_saturation = 56 * pi / 180;
        theta_d = min(max(theta_d, -theta_saturation), theta_saturation);
        
        k_p_theta = 5;
        k_d_theta = 0.1;
        theta_error = theta_d - theta;
        omega = (theta - obj.theta_ball_prev) / dt;

        V_servo = k_p_theta * theta_error + k_d_theta * omega;
        
        obj.t_prev = t;
        obj.p_ball_prev = p_ball;
        obj.theta_ball_prev = theta;
        obj.theta_d = theta_d;
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
