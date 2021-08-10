function thrust = control_allocation(tau)
% No es perfecto, pero funciona
alpha = pi/4;
rx_v = 0.0395;
ry_v = 0.2384;
rx_h = 0.1867;
ry_h = 0.2347;
rz_h = 0.0175;
% Mapping matrix
T = [cos(alpha) cos(alpha) -cos(alpha) -cos(alpha) 0 0;
     -sin(alpha) sin(alpha) -sin(alpha) sin(alpha) 0 0;
     0 0 0 0 -1 -1;
     -sin(alpha)*rz_h sin(alpha)*rz_h -sin(alpha)*rz_h sin(alpha)*rz_h -ry_v ry_v;
     -cos(alpha)*rz_h -cos(alpha)*rz_h cos(alpha)*rz_h cos(alpha)*rz_h -rx_v rx_v;
     -(rx_h*sin(alpha)+ry_h*cos(alpha)) (rx_h*sin(alpha)+ry_h*cos(alpha)) (rx_h*sin(alpha)+ry_h*cos(alpha)) -(rx_h*sin(alpha)+ry_h*cos(alpha)) 0 0
    ];

% Moore–Penrose pseudo-inverse
%T_pseudo_inv = T.'/(T*T.');

%disp(T_pseudo_inv)
%disp(T)
% Control input
thrust = pinv(T)*tau;
disp(thrust)