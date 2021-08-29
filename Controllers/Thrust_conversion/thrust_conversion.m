function tau = thrust_conversion(thrust)
% WORKS!! Kindof. When used with large values from control outputs, this
% code produced weird results.
% Errores numericos. El input esta saturado a [100,-100], y estas salidas
% dan valores muchisimo mayores a 35 N c/u
% Ideal to tau --> used in controller
theta = pi/4;
rx_v = 0.0395;
ry_v = 0.2384;
rx_h = 0.1867;
ry_h = 0.2347;
rz_h = 0.0175;

tau = [0 0 0 0 0 0].';

% For surge
s_val = thrust(1)/cos(theta)/4;
tau(1) = tau(1)+s_val;
tau(2) = tau(2)+s_val;
tau(3) = tau(3)-s_val;
tau(4) = tau(4)-s_val;

% For sway
sw_val = thrust(2)/sin(theta)/4;
tau(1) = tau(1)-sw_val;
tau(2) = tau(2)+sw_val;
tau(3) = tau(3)-sw_val;
tau(4) = tau(4)+sw_val;

% For heave
tau(5) = tau(5)+(-thrust(3)/2);
tau(6) = tau(6)+(-thrust(3)/2);

% For yaw
y_val = thrust(6)/(rx_h*sin(theta)+ry_h*cos(theta))/4;
tau(1) = tau(1)-y_val;
tau(2) = tau(2)+y_val;
tau(3) = tau(3)+y_val;
tau(4) = tau(4)-y_val;

% K and M come as zero
% For K
if  thrust(4)~= 0
    %disp("K")
    k_val = (thrust(4)-(-tau(1)+tau(2)-tau(3)+tau(4))*sin(theta)*rz_h)/(2*ry_v);
    tau(5) = tau(5)-k_val;
    tau(6) = tau(6)+k_val;
end

% For M
if thrust(5) ~= 0
    %disp("M")
    M_val = (thrust(5)-(-tau(1)-tau(2)+tau(3)+tau(4))*cos(theta)*rz_h)/(2*rx_v);
    tau(5) = tau(5)-M_val;
    tau(6) = tau(6)+M_val;
end

disp(tau)
