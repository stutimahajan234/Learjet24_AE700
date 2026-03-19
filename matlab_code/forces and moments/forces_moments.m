% forces_moments.m
%   Computes the forces and moments acting on the airframe. 
%
%   Output is
%       F     - forces
%       M     - moments
%       Va    - airspeed
%       alpha - angle of attack
%       beta  - sideslip angle
%       wind  - wind vector in the inertial frame
%

function out = forces_moments(x, delta, wind, P)

    % relabel the inputs
    pn      = x(1);
    pe      = x(2);
    pd      = x(3);
    u       = x(4);
    v       = x(5);
    w       = x(6);
    phi     = x(7);
    theta   = x(8);
    psi     = x(9);
    p       = x(10);
    q       = x(11);
    r       = x(12);
    delta_e = delta(1);
    delta_a = delta(2);
    delta_r = delta(3);
    delta_t = delta(4);
    w_ns    = wind(1); % steady wind - North
    w_es    = wind(2); % steady wind - East
    w_ds    = wind(3); % steady wind - Down
    u_wg    = wind(4); % gust along body x-axis
    v_wg    = wind(5); % gust along body y-axis    
    w_wg    = wind(6); % gust along body z-axis
    
    % wind in inertial frame
    w_n = w_ns;
    w_e = w_es;
    w_d = w_ds;
    
    %Rotation matrix (inertial to body frame)
    R_ib = [cos(theta)*cos(psi), cos(theta)*sin(psi), -sin(theta);
        sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), ...
        sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), ...
        sin(phi)*cos(theta);
        cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi), ...
        cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi), ...
        cos(phi)*cos(theta)];


   % steady wind in body frame
   wind_body_steady = R_ib * [w_n; w_e; w_d];

   % total wind
   wind_body = wind_body_steady + [u_wg; v_wg; w_wg];

   %relative airvelocity
   u_r = u - wind_body(1);
   v_r = v - wind_body(2);
   w_r = w - wind_body(3);

   %airspeed 
   Va = sqrt(u_r^2 + v_r^2 + w_r^2);
   Va = max(Va, 0.01);

   %angles
   alpha = atan2(w_r,u_r);
   beta = asin( max(min(v_r/Va,1),-1) );

    %computing forces
    %gravity force in body frame
    F_g = P.mass * P.gravity * ...
         [-sin(theta);
         sin(phi)*cos(theta);
         cos(phi)*cos(theta)];

   % aerodynamic forces 
   q_bar = 0.5 * P.rho * Va^2;

   CL = P.C_L_0 ...
   + P.C_L_alpha * alpha ...
   + P.C_L_q * (P.c/(2*Va)) * q ...
   + P.C_L_delta_e * delta_e;

   CD = P.C_D_0 ...
   + P.C_D_alpha * alpha ...
   + P.C_D_q * (P.c/(2*Va)) * q ...
   + P.C_D_delta_e * delta_e;

   F_lift = q_bar * P.S_wing * CL;
   F_drag = q_bar * P.S_wing * CD;

   Fx_aero = -F_drag*cos(alpha) - F_lift*sin(alpha);
   Fz_aero = -F_drag*sin(alpha) + F_lift*cos(alpha);

   CY = P.C_Y_0 + P.C_Y_beta*beta ...
        + P.C_Y_p*(P.b/(2*Va))*p + P.C_Y_r*(P.b/(2*Va))*r ...
        + P.C_Y_delta_a*delta_a + P.C_Y_delta_r*delta_r;

   Fy_aero = q_bar * P.S_wing * CY;

   %Propulsion Forces
   V_in = P.V_max * delta_t;

   a = P.C_Q0 * P.rho * P.D_prop^5 / (4*pi^2);
   b = (P.C_Q1 * P.rho * P.D_prop^4 * Va)/(2*pi) + (P.KQ^2)/P.R_motor;
   c = P.C_Q2 * P.rho * P.D_prop^3 * Va^2 ...
        - (P.KQ/P.R_motor)*V_in + P.KQ*P.i0;

   disc = b^2 - 4*a*c;
   disc = max(disc, 0);

   Omega = (-b + sqrt(disc)) / (2*a);
   Omega = max(Omega, 1);

   J = 2*pi*Va / (Omega * P.D_prop);
   J = max(min(J, 1), 0);

   C_T = P.C_T2*J^2 + P.C_T1*J + P.C_T0;

   T_prop = P.rho * Omega^2 * P.D_prop^4 * C_T / (4*pi^2);

   Fx_prop = T_prop;

   %Total forces 
   Force = [Fx_aero + Fx_prop; 
            Fy_aero; 
            Fz_aero] + F_g;

   % Compute Moments
   ell = q_bar * P.S_wing * P.b * ...
    (P.C_ell_0 ...
    + P.C_ell_beta * beta ...
    + P.C_ell_p * (P.b/(2*Va)) * p ...
    + P.C_ell_r * (P.b/(2*Va)) * r ...
    + P.C_ell_delta_a * delta_a ...
    + P.C_ell_delta_r * delta_r);

   m = q_bar * P.S_wing * P.c * ...
    (P.C_m_0 ...
    + P.C_m_alpha * alpha ...
    + P.C_m_q * (P.c/(2*Va)) * q ...
    + P.C_m_delta_e * delta_e);

   n = q_bar * P.S_wing * P.b * ...
    (P.C_n_0 ...
    + P.C_n_beta * beta ...
    + P.C_n_p * (P.b/(2*Va)) * p ...
    + P.C_n_r * (P.b/(2*Va)) * r ...
    + P.C_n_delta_a * delta_a ...
    + P.C_n_delta_r * delta_r);

   Torque = [ell; m; n];


    out = [Force; Torque; Va; alpha; beta; w_n; w_e; w_d];
end



