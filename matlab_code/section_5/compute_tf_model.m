%----------------------------------------------------------
% compute transfer function models
%----------------------------------------------------------

% extract trim values
Va_trim    = sqrt(x_trim(4)^2 + x_trim(5)^2 + x_trim(6)^2);
alpha_trim = atan2(x_trim(6), x_trim(4));
theta_trim = x_trim(8);

%----------------------------------------------------------
% LATERAL DYNAMICS
%----------------------------------------------------------

a_phi1 = -0.5 * MAV.rho * Va_trim^2 * MAV.S_wing * MAV.b ...
         * MAV.C_ell_p * MAV.b / (2*Va_trim);

a_phi2 =  0.5 * MAV.rho * Va_trim^2 * MAV.S_wing * MAV.b ...
         * MAV.C_ell_delta_a;

%----------------------------------------------------------
% LONGITUDINAL DYNAMICS (PITCH)
%----------------------------------------------------------

a_theta1 = -MAV.rho * Va_trim^2 * MAV.c * MAV.S_wing ...
           * MAV.C_m_q * MAV.c / (2*MAV.Jy*2*Va_trim);

a_theta2 = -MAV.rho * Va_trim^2 * MAV.c * MAV.S_wing ...
           * MAV.C_m_alpha / (2*MAV.Jy);

a_theta3 =  MAV.rho * Va_trim^2 * MAV.c * MAV.S_wing ...
           * MAV.C_m_delta_e / (2*MAV.Jy);

%----------------------------------------------------------
% AIRSPEED DYNAMICS
%----------------------------------------------------------

% safe defaults (since your params don't include these)
C_prop  = 1;
k_motor = 1;

a_V1 = MAV.rho * Va_trim * MAV.S_wing / MAV.mass * ...
       (MAV.C_D_0 + MAV.C_D_alpha * alpha_trim + MAV.C_D_delta_e * u_trim(1)) ...
       + MAV.rho * MAV.S_prop / MAV.mass * C_prop * Va_trim;

a_V2 = MAV.rho * MAV.S_prop / MAV.mass * C_prop * k_motor^2 * u_trim(4);

a_V3 = MAV.gravity * cos(theta_trim - alpha_trim);

%----------------------------------------------------------
% SIDESLIP DYNAMICS
%----------------------------------------------------------

a_beta1 = -MAV.rho * Va_trim * MAV.S_wing * MAV.C_Y_beta ...
          / (2*MAV.mass);

a_beta2 =  MAV.rho * Va_trim * MAV.S_wing * MAV.C_Y_delta_r ...
          / (2*MAV.mass);

%----------------------------------------------------------
% TRANSFER FUNCTIONS
%----------------------------------------------------------

T_phi_delta_a   = tf([a_phi2],[1,a_phi1,0]);
T_chi_phi       = tf([MAV.gravity/Va_trim],[1,0]);

T_theta_delta_e = tf([a_theta3],[1,a_theta1,a_theta2]);

T_h_theta       = tf([Va_trim],[1,0]);
T_h_Va          = tf([theta_trim],[1,0]);

T_Va_delta_t    = tf([a_V2],[1,a_V1]);
T_Va_theta      = tf([-a_V3],[1,a_V1]);

T_v_delta_r     = tf([a_beta2],[1,a_beta1]);

