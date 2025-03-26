classdef studentControllerInterfaceAIO < matlab.System
    properties (Access = private)
        %% You can add values that you want to store and updae while running your controller.
        % For more information of the supported data type, see
        % https://www.mathworks.com/help/simulink/ug/data-types-supported-by-simulink.html
        
        % General properties
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

        % EKF properties
        x_hat = [-0.19;0;0;0];
        P_m = 0.1*eye(4);
        L = eye(4);
        H = [1,0,0,0;
             0,0,1,0];
        M = eye(2);
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
            a_ref_prev = obj.a_ref_prev;
            j_ref_prev = obj.j_ref_prev;
            s_ref_prev = obj.s_ref_prev;

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

            %% Approximate I/O Linearization
            t_ramp = 0.5;

            % Desired pole locations with "worse" approximation: MATLAB
            p1 = max(-0.55 - 0.57*t/t_ramp, -1.12);
            p2 = max(-2.7 - 0*t/t_ramp, -2.7);
            p3 = max(-6 - 0*t/t_ramp, -6);
            p4 = max(-32.5 - 0*t/t_ramp, -32.5);

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

            % k values (in A_k) based on desired poles
            k1 = p1*p2*p3*p4;
            k2 = -(p2*p3*p4 + p1*p3*p4 + p1*p2*p4 + p1*p2*p3);
            k3 = p1*p2 + p1*p3 + p1*p4 + p2*p3 + p2*p4 + p3*p4;
            k4 = -(p1 + p2 + p3 + p4);


            % Get or calculate reference trajectory and derivatives
            [p_ball_ref, v_ball_ref, a_ball_ref] = get_ref_traj(t);
            if dt > 0
                j_ball_ref = (a_ball_ref - a_ref_prev)/dt;
                s_ball_ref = (j_ball_ref - j_ref_prev)/dt;
            else
                j_ball_ref = j_ref_prev;
                s_ball_ref = s_ref_prev;
            end

            % Calculate reference Lie derivatives
            LgLf3 = (7*len*tau) / (5*g*r_g*K_motor*cos(x_hat(3)));
            % "Worse" approximation
            Lf4 = -(5*g*r_g) / (7*len) * (x_hat(4)*cos(x_hat(3))/tau + x_hat(4)^2*sin(x_hat(3)));
            % "Better" approximation
            % xdot2 = (5*g*r_g) / (7*len) * sin(x_hat(3)) - (5*r_g^2) / (7*len^2) * (len/2 - x_hat(1)) * x_hat(4)^2*(cos(x_hat(3))^2);
            % Lf4 = -(5*g*r_g*x_hat(4)*cos(x_hat(3))) / (7*len*tau) - (5*g*r_g*x_hat(4)^2*sin(x_hat(3))) / (7*len^2) + (5*r_g^2*xdot2*x_hat(4)^3*(cos(x_hat(3)))^2) / (7*len^2) ...
            %     - (10*r_g^2*x_hat(4)^2*(cos(x_hat(3)))^2) / (7*len^2*tau) - (5*r_g^2*x_hat(2)*x_hat(4)^2*sin(2*x_hat(3))) / (7*len^2) - (10*r_g^2*x_hat(2)*x_hat(4)^2*(cos(x_hat(3)))^2) / (7*tau*len^2) ...
            %     - (20*r_g^2*x_hat(4)^2*(cos(x_hat(3)))^2)*(len/2 - x_hat(1)) / (7*tau*len^2) - (10*r_g^2*x_hat(4)^2*sin(2*x_hat(3)))*(len/2 - x_hat(1)) / (7*tau*len^2) - (5*r_g^2*x_hat(2)*x_hat(4)^2*sin(2*x_hat(3))) / (7*len^2) ...
            %     + (10*r_g^2*x_hat(4)^2*(len/2 - x_hat(1))*sin(2*x_hat(3))) / (7*len^2) + (10*r_g^2*x_hat(4)^2*(len/2 - x_hat(1))*cos(2*x_hat(3))) / (7*len^2);

            % Define xi state (output and derivatives)
            xi1 = x_hat(1);
            xi2 = x_hat(2);
            xi3 = (5*g*r_g) / (7*len) * sin(x_hat(3));
            xi4 = (5*g*r_g) / (7*len) * x_hat(4) * cos(x_hat(3));

            % Apply I/O Linerazation
            V_servo = LgLf3*(-Lf4 - k1*(xi1 - p_ball_ref) - k2*(xi2 - v_ball_ref) - k3*(xi3 - a_ball_ref) - k4*(xi4 - j_ball_ref) + s_ball_ref);

            %% Update class properties if necessary.
            obj.t_prev = t;
            obj.x_hat = x_hat;
            obj.P_m = P_m;
            obj.u_prev = V_servo;
            obj.a_ref_prev = a_ball_ref;
            obj.j_ref_prev = j_ball_ref;
            obj.s_ref_prev = s_ball_ref;
        end
    end
    
    methods(Access = public)
        % Used this for matlab simulation script. fill free to modify it as
        % however you want.
        function V_servo = stepController(obj, t, p_ball, theta)        
            V_servo = stepImpl(obj, t, p_ball, theta);
        end
    end
    
end
