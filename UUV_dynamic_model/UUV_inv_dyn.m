% Skew symmetric matrix
S = @(s) [0    -s(3) s(2);
          s(3) 0     -s(1);
          -s(2) s(1)  0    ];  
      
% Parameters
m = 24;
vol = 0.02306187;
W = m*9.81;
B = W*1.005; %1000*9.81*vol;

r_g = [0 0 0].';    % Distance from origin to center of mass
r_b = [0 0 -0.10726].';    % Distance from origin to center of buoyancy
I_g = [0.900121387  -0.000186482 0.0072251;
       -0.000186482 1.754494427  0.020319615;
       0.0072251    0.020319615  1.43389     ];

b = 0.25;
l = 0.18;
   
phi = deg2rad(0); % roll
theta = deg2rad(0); % pitch
psi = deg2rad(45); % yaw
alpha = deg2rad(45); % Orientacion motores

vel   = [1 0 0 0 0 0].';% Linear and angular speed in body
accel = [0 0 0 0 0 0].'; % Linear and angular acceleration in body

% Linear damping coefficients
X_u = 0.3431;
Y_v = -0.0518;
Z_w = 0.5841;
K_p = -0.0064;
M_q = -0.04;
N_r = 0.1063;

% Cuadratic damping coefficients
X_uu = 111.7397;
Y_vv = 44.4058;
Z_ww = 157.1951;
K_pp = 0.4634;
M_qq = 0.2902;
N_rr = 2.2897;

%------------ UUV kinematics ----------------%
% Linear velocities
% For a zxy rotation sequence
N_dot = vel(1)*cos(psi)*cos(theta) + vel(2)*[cos(psi)*sin(theta)*sin(phi) - sin(psi)*cos(phi)] + vel(3)*[sin(psi)*sin(phi) + cos(psi)*cos(phi)*sin(theta)];

E_dot = vel(1)*sin(psi)*cos(theta) + vel(2)*[cos(psi)*cos(phi) + sin(phi)*sin(theta)*sin(psi)]+ vel(3)*[sin(theta)*sin(psi)*cos(phi) - cos(psi)*sin(phi)];

D_dot = -vel(1)*sin(theta) + vel(2)*cos(theta)*sin(phi) + vel(3)*cos(theta)*cos(phi);

%Angular velocities
phi_dot = vel(4) + vel(5)*sin(phi)*tan(theta) + vel(6)*cos(phi)*tan(theta);

roll_dot = vel(5)*cos(phi) - vel(6)*sin(phi);

psi_dot = vel(5)*(sin(phi)/cos(theta)) + vel(6)*(cos(phi)/cos(theta));

eta_dot = [N_dot, E_dot, D_dot, phi_dot, roll_dot, psi_dot];

disp("Eta")
disp(eta_dot);
%------------ UUV dynamic model parametrization ----------------%

% Rigid-Body Inertia Matrix
M_rb = [ m*eye(3)    -m*S(r_g);
         m*S(r_g)         I_g ];
    
disp("RB mass matrix")
disp(M_rb)

% Added mass matrix
M_a = diag([16.8374 20.2748 35.3180 0.2165 0.6869 0.6157]);
disp("Added mass matrix")
disp(M_a)

% Mass matrix
M = M_rb + M_a;
disp("Mass matrix")
disp(M)

% Rigid-body Coriolis-centripetal Matrix
C_rb = [ zeros(3)             -m*S(vel(1:3))-m*S(vel(4:6))*S(r_g);
        -m*S(vel(1:3))+m*S(r_g)*S(vel(4:6))            -S(I_g*vel(4:6))];

disp("RB Coreolis Matrix")
disp(C_rb)

% Coreolis added mass matrix 
C_a = [ 0 0 0   0 -M_a(3,3)*vel(3) M_a(2,2)*vel(2) ;
        0 0 0   M_a(3,3)*vel(3) 0 -M_a(1,1)*vel(1) ;
        0 0 0  -M_a(2,2)*vel(2) M_a(1,1)*vel(1) 0  ;
        0 -M_a(3,3)*vel(3) M_a(2,2)*vel(2)  0 -M_a(6,6)*vel(6) M_a(5,5)*vel(5) ;
        M_a(3,3)*vel(3) 0 -M_a(1,1)*vel(1)  M_a(6,6)*vel(6) 0 -M_a(4,4)*vel(4) ;
       -M_a(2,2)*vel(2) M_a(1,1)*vel(1) 0   -M_a(5,5)*vel(5) M_a(4,4)*vel(4) 0  ];
disp("Added mass Coreolis matrix")
disp(C_a)

% Coreolis matrix
C = C_rb + C_a;
disp("Coreolis matrix")
disp(C_a)

% Restoring forces
g_eta = [ (W-B)*sin(theta);
          -(W-B)*cos(theta)*sin(phi);
          -(W-B)*cos(theta)*cos(phi);
          -(r_g(2)*W-r_b(2)*B)*cos(theta)*cos(phi) + (r_g(3)*W-r_b(3)*B)*cos(theta)*sin(phi);
           (r_g(3)*W-r_b(3)*B)*sin(theta)          + (r_g(1)*W-r_b(1)*B)*cos(theta)*cos(phi)
          -(r_g(1)*W-r_b(1)*B)*cos(theta)*sin(phi) - (r_g(2)*W-r_b(2)*B)*sin(theta)          ];

disp("Restoring forces and moments matrix")
disp(g_eta)
   
% Damping matrix
D_lin = -diag([X_u Y_v Z_w K_p M_q N_r]);
D_cuad = -diag([X_uu*abs(vel(1)) Y_vv*abs(vel(2)) Z_ww*abs(vel(3)) ...
               K_pp*abs(vel(4)) M_qq*abs(vel(5)) N_rr*abs(vel(6))]);
D = D_lin + D_cuad;
disp("Damping matrix")
disp(D)

disp("Damping linear matrix")
disp(D_lin)

disp("Damping cuadratic matrix")
disp(D_cuad)

% Tau
tau = M*accel + C*vel + D*vel + g_eta;
disp("Tau")
disp(tau)

tau_1 = (tau(1)/cos(alpha)/4) - (tau(2)/sin(alpha)/4) - (tau(6)/4/sqrt(b^2 + l^2));
tau_2 = (tau(1)/cos(alpha)/4) + (tau(2)/sin(alpha)/4) + (tau(6)/4/sqrt(b^2 + l^2));
tau_5 = -(tau(1)/cos(alpha)/4) - (tau(2)/sin(alpha)/4) + (tau(5)/2)*b + (tau(6)/4/sqrt(b^2 + l^2));
tau_6 = -(tau(1)/cos(alpha)/4) + (tau(2)/sin(alpha)/4) - (tau(5)/2)*b - (tau(6)/4/sqrt(b^2 + l^2));
tau_3 = -tau(3)/2;
tau_4 = -tau(3)/2;

disp(tau_1);
disp(tau_2);
disp(tau_3);
disp(tau_4);
disp(tau_5);
disp(tau_6);