classdef studentControllerInterfaceLQG < matlab.System
    properties (Access = private)
        %% You can add values that you want to store and updae while running your controller.
        % For more information of the supported data type, see
        % https://www.mathworks.com/help/simulink/ug/data-types-supported-by-simulink.html
        t_prev = -0.01;
        theta_d = 0;
        x_hat = [-0.19;0;0;0];
        P_m = 0.1*eye(4);
        L = eye(4);
        H = [1,0,0,0;
             0,0,1,0];
        M = eye(2);
        r_g = 0.0254;
        len = 0.4255;
        g = 9.81;
        K_motor = 1.5;
        tau = 0.025;
        u_prev = 0;
        Sigma_vv = 0.01*eye(4);
        Sigma_ww = 0.01*eye(2);
    end
    methods(Access = protected)
        % function setupImpl(obj)
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


            %% LQR

            [p_ball_ref, v_ball_ref, a_ball_ref] = get_ref_traj(t);
            
            theta_ref = asin(a_ball_ref*len/(r_g*g));

            A_lqr = [1, dt, 0, 0;
                     5/7*(r_g/len)^2*x_hat(4)^2*(cos(x_hat(3)))^2*dt, 1, (5*g/7*r_g/len*cos(x_hat(3)) + 10/7*(len/2-x_hat(1))*(r_g/len)^2*x_hat(4)^2*cos(x_hat(3))*sin(x_hat(3)))*dt, -10/7*(len/2-x_hat(1))*(r_g/len)^2*x_hat(4)*(cos(x_hat(3)))^2*dt;
                     0, 0, 1, dt;
                     0, 0, 0, 1-dt/tau];
            B_lqr = [0;0;0;K_motor/tau*dt];
            Q = diag([min(1000,1 + t*999), min(1000,1 + t*999), 1, 1]); % for MATLAB sim
            % Q = diag([200,200,1,1]); % for Simulink
            R = 1;
            thresh = 0.1;

            % DARE solver since dlqr not supported in Simulink
            A_cur = A_lqr;
            G_cur = B_lqr*B_lqr'/R;
            H_prev = zeros(4);
            H_cur = Q;
            while norm(H_cur - H_prev)/norm(H_cur) >= thresh
                A_prev = A_cur;
                G_prev = G_cur;
                H_prev = H_cur;
                temp = (eye(4) + G_cur*H_cur)\eye(4);
                A_cur = A_prev*temp*A_prev;
                G_cur = G_prev + A_prev*temp*G_cur*A_prev';
                H_cur = H_prev + A_prev'*H_cur*temp*A_prev;
            end

            F = (R + B_lqr'*H_cur*B_lqr)\(B_lqr'*H_cur*A_lqr);

            % F_debug = dlqr(A_lqr,[0;0;0;K_motor/tau*dt],diag([1000, 1000, 1, 1]),1);
            
            x_tilde = x_hat - [p_ball_ref; v_ball_ref; theta_ref; 0];


            % Apply voltage for previous timestep if current one is zero
            % (for some reason there is a Simulink error if you don't do
            % this)
            if dt > 0
                V_servo = -F*x_tilde;
            else
                V_servo = u_prev;
            end

            %% Safety check, ensure motor does not exceed 56 degrees
            future_theta = x_hat(3) + (x_hat(4) + (-x_hat(4) + K_motor)*dt/tau)*dt;
            if future_theta > 56*pi/180 || future_theta < -56*pi/180
                V_servo = 0;
            end

            % Update class properties if necessary.
            obj.t_prev = t;
            obj.x_hat = x_hat;
            obj.P_m = P_m;
            obj.u_prev = V_servo;
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
