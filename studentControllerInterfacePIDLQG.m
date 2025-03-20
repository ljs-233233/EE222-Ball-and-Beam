classdef studentControllerInterfacePIDLQG < matlab.System
    properties (Access = private)
        %% You can add values that you want to store and updae while running your controller.
        % For more information of the supported data type, see
        % https://www.mathworks.com/help/simulink/ug/data-types-supported-by-simulink.html
        
        % General Properties
        t_prev = -0.01;
        r_g = 0.0254;
        len = 0.4255;
        g = 9.81;
        K_motor = 1.5;
        tau = 0.025;

        % EKF Properties
        x_hat = [-0.19;0;0;0];
        P_m = 0.1*eye(4);
        L = eye(4);
        H = [1,0,0,0;
             0,0,1,0];
        M = eye(2);
        u_prev = 0;
        Sigma_vv = 0.01*eye(4);
        Sigma_ww = 0.01*eye(2);

        % PID Properties
        theta_d = 0;
        pos_error_prev = 0;
        integral_p = 0;
        vel_error_prev = 0;
        integral_v = 0;
        theta_ball_prev = 0;
        integral_theta = 0;

        % LQR Properties    
        
        
    end
    methods(Access = protected)
        function setupImpl(obj, t, p_ball, theta)
            obj.x_hat(1) = p_ball;
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
    
            % p_err = obj.p_err;
            % % Extract reference trajectory at the current timestep.
            % [p_ball_ref, v_ball_ref, a_ball_ref] = get_ref_traj(t);
            % 
            % % Decide desired servo angle based on simple proportional feedback.
            % k_p = 1;
            % theta_d = - k_p * (p_ball - p_ball_ref);
            % 
            % % Make sure that the desired servo angle does not exceed the physical
            % % limit. This part of code is not necessary but highly recommended
            % % because it addresses the actual physical limit of the servo motor.
            % theta_saturation = 56 * pi / 180;    
            % theta_d = min(theta_d, theta_saturation);
            % theta_d = max(theta_d, -theta_saturation);
            % 
            % % Simple position control to control servo angle to the desired
            % % position.
            % k_servo = 10;
            % V_servo = k_servo * (theta_d - theta);

            t_prev = obj.t_prev;
            dt = t - t_prev;
            x = obj.x_hat;
            P_m = obj.P_m;
            r_g = obj.r_g;
            len = obj.len;
            g = obj.g;
            u_prev = obj.u_prev;
            K_motor = obj.K_motor;
            tau = obj.tau;
            L = obj.L;
            H = obj.H;
            M = obj.M;
            Sigma_vv = obj.Sigma_vv;
            Sigma_ww = obj.Sigma_ww;
            pos_error_prev = obj.pos_error_prev;
            vel_error_prev = obj.vel_error_prev;
            integral_p = obj.integral_p;
            integral_v = obj.integral_v;

            %% EKF
            A = [1, dt, 0, 0;
                 5/7*(r_g/len)^2*x(4)^2*(cos(x(3)))^2*dt, 1, (5*g/7*r_g/len*cos(x(3)) + 10/7*(len/2-x(1))*(r_g/len)^2*x(4)^2*cos(x(3))*sin(x(3)))*dt, -10/7*(len/2-x(1))*(r_g/len)^2*x(4)*(cos(x(3)))^2*dt;
                 0, 0, 1, dt;
                 0, 0, 0, 1-dt/tau];

            x_p = [x(1) + x(2)*dt;
                   x(2) + (5*g/7*r_g/len*sin(x(3)) - 5/7*(len/2-x(1))*(r_g/len)^2*x(4)^2*(cos(x(3)))^2)*dt;
                   x(3) + x(4)*dt;
                   x(4) + (-x(4)/tau + K_motor/tau*u_prev)*dt];
            P_p = A*P_m*A' + L*Sigma_vv*L';

            K = P_p*H'/(H*P_p*H' + M*Sigma_ww*M');
            x_hat = x_p + K*([p_ball;theta] - [x_p(1);x_p(3)]);
            P_m = (eye(4) - K*H)*P_p*(eye(4) - K*H)' + K*M*Sigma_ww*M'*K';


            %% PID

            [p_ball_ref, v_ball_ref, a_ball_ref] = get_ref_traj(t);

            % Position control to get a desired velocity
            pos_error = x_hat(1) - p_ball_ref;
            integral_p = integral_p + pos_error * dt;
            max_int_p = 0.03;
            integral_p = min(max(-max_int_p, integral_p),max_int_p);
            k_p_ball = 1.2;  % --\
            k_d_ball = 0.00; % --- Tune these values
            k_i_ball = 0.02; % --/
            v_des = v_ball_ref - k_p_ball*pos_error - k_d_ball*(pos_error - pos_error_prev)/dt - k_i_ball*integral_p;

            % Velocity control to get a desired acceleration (angle)
            vel_error = x_hat(2) - v_des;
            integral_v = integral_v + vel_error * dt;
            max_int_v = 1;
            integral_v = min(max(-max_int_v, integral_v),max_int_v);
            k_p_vel = 3;   % --\
            k_d_vel = 0;   % --- Tune these values
            k_i_vel = 0.0; % --/
            a_des = a_ball_ref - k_p_vel*vel_error - k_d_vel*(vel_error - vel_error_prev)/dt + k_i_vel*integral_v;

            % Angle Computation
            a_param = 5 * g * r_g / (7 * len);
            theta_d = a_des / a_param;
            % theta_d = asin(a_des*len/(r_g*g));
            theta_saturation = 56 * pi / 180;
            theta_d = min(max(theta_d, -theta_saturation), theta_saturation);

            % Proportional control to get a desired omega
            k_p_theta = 1; % Tune this value
            omega_d = -k_p_theta * (x_hat(3) - theta_d);



            %% LQR

            A_lqr = [1, dt;
                     0, 1-dt/tau];
            B_lqr = [0;K_motor/tau*dt];
            Q = diag([1000, 100]); % for MATLAB sim   --\
            R = 1;                                  % --/ Tune these values
            thresh = 0.1;

            % DARE solver since dlqr not supported in Simulink
            A_cur = A_lqr;
            G_cur = B_lqr*B_lqr'/R;
            H_prev = zeros(2);
            H_cur = Q;
            while norm(H_cur - H_prev)/norm(H_cur) >= thresh
                A_prev = A_cur;
                G_prev = G_cur;
                H_prev = H_cur;
                temp = (eye(2) + G_cur*H_cur)\eye(2);
                A_cur = A_prev*temp*A_prev;
                G_cur = G_prev + A_prev*temp*G_cur*A_prev';
                H_cur = H_prev + A_prev'*H_cur*temp*A_prev;
            end

            F = (R + B_lqr'*H_cur*B_lqr)\(B_lqr'*H_cur*A_lqr);

            % F_debug = dlqr(A_lqr,B_lqr,Q,R);
            
            theta_tilde = [x_hat(3) - theta_d; x_hat(4) - omega_d];

            % Apply voltage for previous timestep if current one is zero
            % (for some reason there is a Simulink error if you don't do
            % this)
            if dt > 0
                V_servo = -F*theta_tilde;
            else
                V_servo = u_prev;
            end

            %% Safety check, ensure motor does not exceed 56 degrees
            % future_theta = x_hat(3) + (x_hat(4) + (-x_hat(4) + K_motor)*dt/tau)*dt;
            % if future_theta > 56*pi/180 || future_theta < -56*pi/180
            %     V_servo = 0;
            % end

            %% Update class properties if necessary.
            obj.t_prev = t;
            obj.x_hat = x_hat;
            obj.P_m = P_m;
            obj.u_prev = V_servo;
            obj.t_prev = t;
            obj.theta_d = theta_d;
            obj.pos_error_prev = pos_error;
            obj.vel_error_prev = vel_error;
            obj.integral_p = integral_p;
            obj.integral_v = integral_v;
        end
    end
    
    methods(Access = public)
        % Used this for matlab simulation script. fill free to modify it as
        % however you want.
        function [V_servo, theta_d] = stepController(obj, t, p_ball, theta)        
            V_servo = stepImpl(obj, t, p_ball, theta);
            theta_d = obj.theta_d;
        end
        function initializeController(obj, t, p_ball, theta)
            setupImpl(obj,t,p_ball,theta);
        end
    end
    
end
