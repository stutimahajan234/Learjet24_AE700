%----------------------------------------------------------
% compute state-space model
%----------------------------------------------------------

[A,B,C,D] = linmod('mavsim_trim', x_trim, u_trim);

%----------------------------------------------------------
% LONGITUDINAL STATES
% [u w q theta pd]
%----------------------------------------------------------

lon_states = [4 6 11 8 3];   % u, w, q, theta, pd
lon_inputs = [1 4];          % delta_e, delta_t

A_lon = A(lon_states, lon_states);
B_lon = B(lon_states, lon_inputs);

%----------------------------------------------------------
% LATERAL STATES
% [v p r phi psi]
%----------------------------------------------------------

lat_states = [5 10 12 7 9];  % v, p, r, phi, psi
lat_inputs = [2 3];          % delta_a, delta_r

A_lat = A(lat_states, lat_states);
B_lat = B(lat_states, lat_inputs);

%----------------------------------------------------------
% DISPLAY
%----------------------------------------------------------
