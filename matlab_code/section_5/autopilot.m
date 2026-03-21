function y = autopilot(uu, AP)


%-------------------- INPUTS --------------------
NN = 0;
pn       = uu(1+NN);
pe       = uu(2+NN);
h        = uu(3+NN);
Va       = uu(4+NN);
alpha    = uu(5+NN);
beta     = uu(6+NN);
phi      = uu(7+NN);
theta    = uu(8+NN);
chi      = uu(9+NN);
p        = uu(10+NN);
q        = uu(11+NN);
r        = uu(12+NN);
Vg       = uu(13+NN);
wn       = uu(14+NN);
we       = uu(15+NN);
psi      = uu(16+NN);
bx       = uu(17+NN);
by       = uu(18+NN);
bz       = uu(19+NN);
NN = NN+19;

Va_c     = uu(1+NN);
h_c      = uu(2+NN);
chi_c    = uu(3+NN);
NN = NN+3;

phi_c_ff = 0;
t        = uu(1+NN);

%-------------------- LATERAL --------------------
chi_ref = wrap(chi_c, chi);

if t==0
    phi_c   = 0;
    delta_r = 0;
else
    phi_c   = course_with_roll(chi_ref, chi, AP) + phi_c_ff;
    delta_r = yaw_damper(r, AP);
end

delta_a = roll_with_aileron(phi_c, phi, p, AP);

%-------------------- LONGITUDINAL --------------------
h_ref = sat(h_c, h+AP.altitude_zone, h-AP.altitude_zone);

if t==0
    delta_t = 0;
    theta_c = 0;
else
    delta_t = airspeed_with_throttle(Va_c, Va, AP);
    theta_c = altitude_with_pitch(h_ref, h, AP);
end

delta_e = pitch_with_elevator(theta_c, theta, q, AP);

delta_t = sat(delta_t, 1, 0);

%-------------------- OUTPUT --------------------
delta = [delta_e; delta_a; delta_r; delta_t];

x_command = [...
    0;
    0;
    h_c;
    Va_c;
    0;
    0;
    phi_c;
    theta_c;
    chi_c;
    0;
    0;
    0;
];

y = [delta; x_command];

end

%================ CONTROLLERS ======================

function phi_c = course_with_roll(chi_c, chi, AP)
persistent integrator
if isempty(integrator), integrator = 0; end

error = chi_c - chi;
integrator = integrator + error * AP.Ts;

phi_c = AP.course_kp*error + AP.course_ki*integrator;
phi_c = sat(phi_c, 30*pi/180, -30*pi/180);

end

function delta_a = roll_with_aileron(phi_c, phi, p, AP)
error = phi_c - phi;
delta_a = AP.roll_kp*error - AP.roll_kd*p;
end

function delta_e = pitch_with_elevator(theta_c, theta, q, AP)
error = theta_c - theta;
delta_e = AP.pitch_kp*error - AP.pitch_kd*q;
end

function delta_t = airspeed_with_throttle(Va_c, Va, AP)
persistent integrator
if isempty(integrator), integrator = 0; end

error = Va_c - Va;
integrator = integrator + error * AP.Ts;

delta_t = AP.airspeed_throttle_kp*error + AP.airspeed_throttle_ki*integrator;

end

function theta_c = altitude_with_pitch(h_c, h, AP)
persistent integrator
if isempty(integrator), integrator = 0; end

error = h_c - h;
integrator = integrator + error * AP.Ts;

theta_c = AP.altitude_kp*error + AP.altitude_ki*integrator;
theta_c = sat(theta_c, 30*pi/180, -30*pi/180);

end

function delta_r = yaw_damper(r, AP)
delta_r = -AP.yaw_damper_kp * r;
end

function out = sat(in, up, low)
out = min(max(in, low), up);
end

function chi_c = wrap(chi_c, chi)
while chi_c - chi > pi
chi_c = chi_c - 2*pi;
end
while chi_c - chi < -pi
chi_c = chi_c + 2*pi;
end
end
