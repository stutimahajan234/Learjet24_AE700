%----------------------------------------------------------
% AUTOPILOT GAINS
%----------------------------------------------------------

addpath('../chap5')
load transfer_function_coef
addpath('../parameters')
simulation_parameters

%----------------------------------------------------------
% BASIC PARAMETERS
%----------------------------------------------------------

AP.gravity = MAV.gravity;
AP.sigma   = 0.05;     % derivative filter
AP.Va0     = Va_trim;
AP.Ts      = 0.01;     % sampling time

%----------------------------------------------------------
% ROLL LOOP (PD)
%----------------------------------------------------------

zeta_phi = 0.707;
wn_phi   = 7;

AP.roll_kp = wn_phi^2 / a_phi2;
AP.roll_kd = (2*zeta_phi*wn_phi - a_phi1) / a_phi2;

%----------------------------------------------------------
% COURSE LOOP (PI)
%----------------------------------------------------------

zeta_chi = 1;
wn_chi   = 1;

AP.course_kp = 2*zeta_chi*wn_chi * Va_trim / MAV.gravity;
AP.course_ki = wn_chi^2 * Va_trim / MAV.gravity;

%----------------------------------------------------------
% SIDESLIP LOOP (PI)
%----------------------------------------------------------

zeta_beta = 0.707;
wn_beta   = 0.5;

AP.sideslip_kp = (2*zeta_beta*wn_beta - a_beta1) / a_beta2;
AP.sideslip_ki = wn_beta^2 / a_beta2;

%----------------------------------------------------------
% YAW DAMPER
%----------------------------------------------------------

AP.yaw_damper_tau_r = 0.5;
AP.yaw_damper_kp    = 0.5;

%----------------------------------------------------------
% PITCH LOOP (PD)
%----------------------------------------------------------

zeta_theta = 0.707;
wn_theta   = 5;

AP.pitch_kp = (wn_theta^2 - a_theta2) / a_theta3;
AP.pitch_kd = (2*zeta_theta*wn_theta - a_theta1) / a_theta3;

K_theta_DC = (AP.pitch_kp * a_theta3) / (a_theta2 + AP.pitch_kp * a_theta3);

%----------------------------------------------------------
% ALTITUDE LOOP (PI)
%----------------------------------------------------------

zeta_h = 0.707;
wn_h   = 0.5;

AP.altitude_kp = 2*zeta_h*wn_h / (K_theta_DC * Va_trim);
AP.altitude_ki = wn_h^2 / (K_theta_DC * Va_trim);

AP.altitude_zone = 10;   % meters

%----------------------------------------------------------
% AIRSPEED HOLD (THROTTLE)
%----------------------------------------------------------

zeta_V = 0.707;
wn_V   = 1;

AP.airspeed_throttle_kp = (2*zeta_V*wn_V - a_V1) / a_V2;
AP.airspeed_throttle_ki = wn_V^2 / a_V2;

disp('Autopilot gains computed successfully')