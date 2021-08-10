function [] = tau_conversion(f)
% WORKS!!
% Tau to ideal --> used in dynamic model
theta = pi/4;
rx_v =  0.0395;
ry_v = 0.2384;
rx_h = 0.1867;
ry_h = 0.2347;
rz_h = 0.0175;

tau = [0 0 0 0 0 0].';
tau(1) = (f(1)+f(2)-f(3)-f(4))*cos(theta);
tau(2) = (-f(1)+f(2)-f(3)+f(4))*sin(theta);
tau(3) = -(f(5)+f(6));
tau(4) = (f(6)-f(5))*ry_v + (-f(1)+f(2)-f(3)+f(4))*sin(theta)*rz_h;
tau(5) = (f(6)-f(5))*rx_v + (-f(1)-f(2)+f(3)+f(4))*cos(theta)*rz_h;
tau(6) = (-f(1)+f(2)+f(3)-f(4))*(rx_h*sin(theta)+ry_h*cos(theta));

disp(tau)