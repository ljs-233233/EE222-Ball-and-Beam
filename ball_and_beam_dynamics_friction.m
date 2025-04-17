function dx = ball_and_beam_dynamics_friction(t, x, u)
p_ball = x(1);
v_ball = x(2);
theta = x(3);
dtheta = x(4);

g = 9.81;
r_arm = 0.0254;
L = 0.4255;
mu = 0.4;

a = 5 * g * r_arm / (7 * L);
b = (5 * L / 14) * (r_arm / L)^2;
c = (5 / 7) * (r_arm / L)^2;

dx = zeros(4, 1);

% dynamics
dx(1) = v_ball;
a2 = a * sin(theta) - b * dtheta^2 * cos(theta)^2 + c * p_ball * dtheta^2 * cos(theta)^2; % Acceleration with no friction
if abs(v_ball) < 0.01
    af_2 = - sign(v_ball) * mu * a * cos(theta); % Static friction
else
    af_2 = 0;
end

if a2 >= 0 % Friction cannot change the direction of acceleration
    dx(2) = max(a2 + af_2, 0); 
else
    dx(2) = min(a2 + af_2, 0); 
end

dx(3) = dtheta;
K = 1.5;
tau = 0.025;
dx(4) = (- dtheta + K * u) / tau; 
end