function [F, M, trpy, drpy] = controller(qd, t, qn, params)
% CONTROLLER quadrotor controller
% The current states are:
% qd{qn}.pos, qd{qn}.vel, qd{qn}.euler = [roll;pitch;yaw], qd{qn}.omega
% The desired states are:
% qd{qn}.pos_des, qd{qn}.vel_des, qd{qn}.acc_des, qd{qn}.yaw_des, qd{qn}.yawdot_des
% Using these current and desired states, you have to compute the desired controls

% =================== Your code goes here ===================

% Gains
kp = 10;
kd = 7;
kp_ang = 6*[1 1 0]';
kd_ang = .2*[1 1 0]';

% Pull out params
m = params.mass;
I = params.I;
g = params.grav;
maxangle = params.maxangle;
maxF = params.maxF;
minF = params.minF;

% Pull out state data
pos = qd{qn}.pos;
vel = qd{qn}.vel;
euler = qd{qn}.euler;
omega = qd{qn}.omega;
pos_des = qd{qn}.pos_des;
vel_des = qd{qn}.vel_des;
yaw_des = qd{qn}.yaw_des;
acc_des = qd{qn}.acc_des;
yawdot_des = qd{qn}.yawdot_des;

% Euler angles and rotations
phi = euler(1); theta = euler(2); psi = euler(3);
% R_world_to_body = RPYtoRot_ZXY(phi,theta,psi);

% Thrust
pos_error = pos_des - pos;
vel_error = vel_des - vel;
thrust_input = acc_des*m + [0 0 m*g]' + m * (kp * pos_error + kd * vel_error);
% F_des = R_world_to_body' * thrust_input;
F_des = thrust_input;
F = F_des(3);

% Moments
phi_des = 1/g * (thrust_input(1)/m * sin(yaw_des) - thrust_input(2)/m * cos(yaw_des));
theta_des = 1/g * (thrust_input(1)/m * cos(yaw_des) + thrust_input(2)/m * sin(yaw_des));
psi_des = yaw_des;
ang_error = [phi_des theta_des psi_des]' - [phi theta psi]';

p_des = 0;
q_des = 0;
r_des = yawdot_des; 
ang_vel_error = [p_des q_des r_des]' - omega;

M = kp_ang .* ang_error + kd_ang .* ang_vel_error;

% Warn if too much input
if F > params.maxF
    error('Exceeding maximum thrust.');
elseif F < 0
    error('Commanding negative thrust.');
end
% =================== Your code ends here ===================

% Output trpy and drpy as in hardware
trpy = [F, phi_des, theta_des, psi_des];
drpy = [0, 0,       0,         0];

end
