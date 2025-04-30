classdef studentControllerInterface < matlab.System
    properties (Access = private)
        %% You can add values that you want to store and updae while running your controller.
        % For more information of the supported data type, see
        % https://www.mathworks.com/help/simulink/ug/data-types-supported-by-simulink.html
        
        % General Properties
        r_g = 0.0254;
        len = 0.4255;
        g = 9.81;
        K_motor = 1.5;
        tau = 0.025;

        % Memory properties
        t_prev = -0.01;
        u_prev = 0;
        a_ref_prev = 0;
        j_ref_prev = 0;
        s_ref_prev = 0;

        % EKF Properties
        x_hat = [0.0;0;-55*pi/180;0];
        P_m = 0.1*eye(4);
        L = eye(4);
        H = [1,0,0,0;
             0,0,1,0];
        M = eye(2);
        Sigma_vv = 0.01*eye(4);
        Sigma_ww = 0.01*eye(2); 

        % PID Properties
        theta_d = 0;
        sum_p_err = 0;
        sum_v_err = 0;
        v_err_prev = 0;

        % Safety properties
        lambda_cbf = 10;

        % ADJUST THIS TO SWITCH BETWEEN CONTROLLERS
        ctr_type = 0; % FEEDBACK LINEARIZATION
        % ctr_type = 1; % PID-LQR

        % Properties used when testing switching between controllers, not
        % currently used
        prev_ctr_type = 0;
        switch_time = 0;
    end
    methods(Access = protected)
        % function setupImpl(obj)
        %    disp("You can use this function for initializaition.");
        % end

        function [V_servo,p_est,v_est,th_est,om_est] = stepImpl(obj, t, p_ball, theta)
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

        ctr_type = obj.ctr_type;
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
        a_ref_prev = obj.a_ref_prev;
        j_ref_prev = obj.j_ref_prev;
        s_ref_prev = obj.s_ref_prev;
        sum_p_err = obj.sum_p_err;
        sum_v_err = obj.sum_v_err;
        v_err_prev = obj.v_err_prev;
        switch_time = obj.switch_time;
        prev_ctr_type = obj.prev_ctr_type;
        
        %% EKF (For both controllers)
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

        p_est = x_hat(1);
        v_est = x_hat(2);
        th_est = x_hat(3);
        om_est = x_hat(4);

        noise_coef = 0.04/(1 + exp(30*(p_est+0.1))) + 0.01;
        Sigma_ww = noise_coef*eye(2);

        [p_ball_ref, v_ball_ref, a_ball_ref] = get_ref_traj(t);

        % Controller type switching logic, not used in final controller
        %
        % if v_ball_ref == 0 && a_ball_ref == 0
        %     ctr_type = 1;
        %     if prev_ctr_type == 0
        %         switch_time = t;
        %     else
        %         switch_time = switch_time;
        %     end
        % else
        %     ctr_type = 0;
        %     if prev_ctr_type == 1
        %         switch_time = t;
        %     else
        %         switch_time = switch_time;
        %     end
        % end


        if ctr_type == 0 % FEEDBACK LINEARIZATION CONTROLLER
            t_ramp = 0.5;

            % Desired pole locations with "worse" approximation: MATLAB
            % p1 = max(-0.55 - 0.57*t/t_ramp, -1.12);
            % p2 = max(-2.7 - 0*t/t_ramp, -2.7);
            % p3 = max(-6 - 0*t/t_ramp, -6);
            % p4 = max(-32.5 - 0*t/t_ramp, -32.5);

            % Desired pole locations with "better" approximation: MATLAB
            % p1 = max(-0.55 - 0.6*t/t_ramp, -1.15);
            % p2 = max(-2.7 - 0*t/t_ramp, -2.7);
            % p3 = max(-4.6 - 0*t/t_ramp, -4.6);
            % p4 = max(-52.5 - 0*t/t_ramp, -52.5);

            % Desired pole locations with "worse" approximation: Simulink
            % p1 = max(-0.55 - 0.57*t/t_ramp, -1.12);
            % p2 = max(-2.7 - 0*t/t_ramp, -2.7);
            % p3 = max(-3.0 - 0*t/t_ramp, -3.0);
            % p4 = max(-40.0 - 0*t/t_ramp, -40.0);

            % NOTE: "BETTER" APPROXIMATION RUNS INTO NAN ERROR IN SIMULINK
            % Desired pole locations with "better" approximation: Simulink
            % p1 = max(-0.55 - 0.57*t/t_ramp, -1.12);
            % p2 = max(-1.2 - 0*t/t_ramp, -1.2);
            % p3 = max(-1.8 - 0*t/t_ramp, -1.8);
            % p4 = max(-40.0 - 0*t/t_ramp, -40.0);

            % Desired pole locations Hardware
            p1 = -1.25;
            p2 = max(-2.7 - 0*t/t_ramp, -2.7);
            p3 = max(-6 - 0*t/t_ramp, -6);
            p4 = max(-32.5 - 0*t/t_ramp, -32.5);

            % k values (in A_k) based on desired poles
            k1 = p1*p2*p3*p4;
            k2 = -(p2*p3*p4 + p1*p3*p4 + p1*p2*p4 + p1*p2*p3);
            k3 = p1*p2 + p1*p3 + p1*p4 + p2*p3 + p2*p4 + p3*p4;
            k4 = -(p1 + p2 + p3 + p4);


            % Get or calculate reference trajectory and derivatives
            if dt > 0
                j_ball_ref = (a_ball_ref - a_ref_prev)/dt;
                s_ball_ref = (j_ball_ref - j_ref_prev)/dt;
            else
                j_ball_ref = j_ref_prev;
                s_ball_ref = s_ref_prev;
            end

            % Calculate reference Lie derivatives

            % "Worse" approximation

            % without friction
            % LgLf3 = (7*len*tau) / (5*g*r_g*K_motor*cos(x_hat(3))); 

            % with friction:
            if v_ball_ref == 0 && a_ball_ref == 0
                mu = 0.15;
            else
                mu = 0.1;
            end

            if x_hat(2) > 0
                LgLf3 = (7*len*tau) / (5*g*r_g*K_motor) * 1 / (cos(x_hat(3)) + mu*sin(x_hat(3)));
            else
                LgLf3 = (7*len*tau) / (5*g*r_g*K_motor) * 1 / (cos(x_hat(3)) - mu*sin(x_hat(3)));
            end

            % without friction
            % Lf4 = -(5*g*r_g) / (7*len) * (x_hat(4)*cos(x_hat(3))/tau + x_hat(4)^2*sin(x_hat(3))); 

            % with friction
            if x_hat(2) > 0 
                Lf4 = (5*g*r_g) / (7*len) * (x_hat(4)^2*(mu*cos(x_hat(3)) - sin(x_hat(3))) - x_hat(4)/tau*(cos(x_hat(3)) + mu*sin(x_hat(3))));
            else
                Lf4 = (5*g*r_g) / (7*len) * (x_hat(4)^2*(-mu*cos(x_hat(3)) - sin(x_hat(3))) - x_hat(4)/tau*(cos(x_hat(3)) - mu*sin(x_hat(3))));
            end

            % "Better" approximation
            % xdot2 = (5*g*r_g) / (7*len) * sin(x_hat(3)) - (5*r_g^2) / (7*len^2) * (len/2 - x_hat(1)) * x_hat(4)^2*(cos(x_hat(3))^2);
            % Lf4 = -(5*g*r_g*x_hat(4)*cos(x_hat(3))) / (7*len*tau) - (5*g*r_g*x_hat(4)^2*sin(x_hat(3))) / (7*len^2) + (5*r_g^2*xdot2*x_hat(4)^3*(cos(x_hat(3)))^2) / (7*len^2) ...
            %     - (10*r_g^2*x_hat(4)^2*(cos(x_hat(3)))^2) / (7*len^2*tau) - (5*r_g^2*x_hat(2)*x_hat(4)^2*sin(2*x_hat(3))) / (7*len^2) - (10*r_g^2*x_hat(2)*x_hat(4)^2*(cos(x_hat(3)))^2) / (7*tau*len^2) ...
            %     - (20*r_g^2*x_hat(4)^2*(cos(x_hat(3)))^2)*(len/2 - x_hat(1)) / (7*tau*len^2) - (10*r_g^2*x_hat(4)^2*sin(2*x_hat(3)))*(len/2 - x_hat(1)) / (7*tau*len^2) - (5*r_g^2*x_hat(2)*x_hat(4)^2*sin(2*x_hat(3))) / (7*len^2) ...
            %     + (10*r_g^2*x_hat(4)^2*(len/2 - x_hat(1))*sin(2*x_hat(3))) / (7*len^2) + (10*r_g^2*x_hat(4)^2*(len/2 - x_hat(1))*cos(2*x_hat(3))) / (7*len^2);

            % Define xi state without friction (output and derivatives)
            % xi1 = x_hat(1);
            % xi2 = x_hat(2);
            % xi3 = (5*g*r_g) / (7*len) * sin(x_hat(3));
            % xi4 = (5*g*r_g) / (7*len) * x_hat(4) * cos(x_hat(3));

            % Define xi state with friction (output and derivatives)
            xi1 = x_hat(1);
            xi2 = x_hat(2);
            if x_hat(2) > 0
                xi3 = (5*g*r_g) / (7*len) * (sin(x_hat(3)) - mu*cos(x_hat(3)));
                xi4 = (5*g*r_g) / (7*len) * x_hat(4) * (cos(x_hat(3)) + mu*sin(x_hat(3)));
            else
                xi3 = (5*g*r_g) / (7*len) * (sin(x_hat(3)) + mu*cos(x_hat(3)));
                xi4 = (5*g*r_g) / (7*len) * x_hat(4) * (cos(x_hat(3)) - mu*sin(x_hat(3)));
            end

            % Apply I/O Linerazation
            V_servo = LgLf3*(-Lf4 - k1*(xi1 - p_ball_ref) - k2*(xi2 - v_ball_ref) - k3*(xi3 - a_ball_ref) - k4*(xi4 - j_ball_ref) + s_ball_ref);

            % Saturate output to help with energy cost
            V_sat = min(0.5 + 2.5*(t-switch_time)/1.0, 3);
            if V_servo > V_sat
                V_servo = V_sat;
            elseif V_servo < -V_sat
                V_servo = -V_sat;
            end
            
            theta_d = 0;
            sum_p_err = 0;
            sum_v_err = 0;
            v_err_prev = 0;
            vel_error = 0;
            prev_ctr_type = 0;

        else % PID-LQR CONTROLLER

            [p_ball_ref, v_ball_ref, a_ball_ref] = get_ref_traj(t);  

            % --- MATLAB tuning ---
            % k_p_ball = 1.2;
            % k_i_ball = 0.6;
            % k_p_vel = 3.0;
            % k_i_vel = 1.2;
            % k_p_theta = 1.0;
            % Q = diag([1000, 100]);
            % R = 16;

            % --- Simulink tuning ---
            % k_p_ball = 1.2;
            % k_p_vel = 3.0;
            % k_p_theta = 1.0;
            % Q = diag([1000,100]);
            % R = 1;

            % --- Hardware tuning ---
            k_p_ball = 1.0;
            k_i_ball = 0.2;
            k_d_ball = 0.1;
            k_p_vel = 3.0;
            k_i_vel = 1.0;
            k_p_theta = 1.0;
            Q = diag([1000, 100]);
            R = 81;

            %% PID

            % Position control to get a desired velocity
            pos_error = x_hat(1) - p_ball_ref;
            sum_p_err = sum_p_err + pos_error*dt;
            if sum_p_err > 0.01
                sum_p_err = 0.01;
            elseif sum_p_err < -0.01
                sum_p_err = -0.01;
            end
            v_des = v_ball_ref - k_p_ball*pos_error - k_i_ball*sum_p_err - k_d_ball*v_err_prev;

            % Velocity control to get a desired acceleration (angle)
            vel_error = x_hat(2) - v_des;
            sum_v_err = sum_v_err + vel_error*dt;
            if sum_v_err > 0.1
                sum_v_err = 0.1;
            elseif sum_v_err < -0.1
                sum_v_err = -0.1;
            end
            a_des = a_ball_ref - k_p_vel*vel_error - k_i_vel*sum_v_err;

            % Angle Computation
            a_param = 5 * g * r_g / (7 * len);
            theta_d = a_des / a_param;
            theta_saturation = 56 * pi / 180;
            theta_d = min(max(theta_d, -theta_saturation), theta_saturation);

            % Proportional control to get a desired omega
            omega_d = -k_p_theta * (x_hat(3) - theta_d);


            %% LQR
            A_lqr = [1, dt;
                     0, 1-dt/tau];
            B_lqr = [0;K_motor/tau*dt];
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

            % Saturate output to help with energy cost
            V_sat = min(0.5 + 2.5*(t-switch_time)/0.25, 3);
            if V_servo > V_sat
                V_servo = V_sat;
            elseif V_servo < -V_sat
                V_servo = -V_sat;
            end

            j_ball_ref = 0;
            s_ball_ref = 0;
            prev_ctr_type = 1;
        end

        %% Safety (for both controllers)
        h = (len/2 - 0.025)^2 - p_ball^2;

        p_dot_est = x_hat(2);
        h_dot = -2 * x_hat(1) * p_dot_est;

        f_ball = (5*g/7)* sin(x_hat(3));
        g_ball = (5*g/7)*(r_g/len)*cos(x_hat(3))*(K_motor/tau);

        Lf2_h = -2*(p_dot_est^2 + x_hat(1) * f_ball);
        Lg2_h = -2 * x_hat(1) * g_ball;

        lambda_val = obj.lambda_cbf;
        
        % CBF condition: Lf2_h + Lg2_h*u + 2*lambda*h_dot + lambda^2*h >= 0
        A_cbf = Lg2_h;
        b_cbf = Lf2_h + 2*lambda_val * h_dot + lambda_val^2 * h;

        if (A_cbf * V_servo + b_cbf) >= 0
            safe = V_servo;
        else
            if abs(A_cbf) > 1e-6
                safe = -b_cbf / A_cbf;
            else
                safe = V_servo;
            end
        end

        voltage_limit = 10;
        V_servo = max(min(safe, voltage_limit), -voltage_limit);

        %% Update class properties
        if ctr_type == 0
            obj.t_prev = t;
            obj.x_hat = x_hat;
            obj.P_m = P_m;
            obj.u_prev = V_servo;
            obj.a_ref_prev = a_ball_ref;
            obj.j_ref_prev = j_ball_ref;
            obj.s_ref_prev = s_ball_ref;
            obj.theta_d = asin(a_ball_ref/(5 * g * r_g / (7 * len)));
            obj.Sigma_ww = Sigma_ww;
        else
            obj.t_prev = t;
            obj.x_hat = x_hat;
            obj.P_m = P_m;
            obj.u_prev = V_servo;
            obj.t_prev = t;
            obj.theta_d = theta_d;
            obj.sum_p_err = sum_p_err;
            obj.sum_v_err = sum_v_err;
            obj.v_err_prev = vel_error;
            obj.Sigma_ww = Sigma_ww;
        end
        obj.switch_time = switch_time;
        obj.prev_ctr_type = prev_ctr_type;


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
