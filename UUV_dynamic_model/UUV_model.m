clear
clc

%input
tau = [0 0 1.1772 0 0 20].';

% Skew symmetric matrix
S = @(s) [0    -s(3) s(2);
          s(3) 0     -s(1);
          -s(2) s(1)  0    ];  

% Vehicle parameters
m = 24;
vol = 0.02306187;
W = m*9.81;
B = W*1.005; %1000*9.81*vol;

r_g = [0 0 0].';    % Distance from origin to center of mass
r_b = [0 0 -0.10726].';    % Distance from origin to center of buoyancy
I_g = [0.900121387  -0.000186482 0.0072251;
       -0.000186482 1.754494427  0.020319615;
       0.0072251    0.020319615  1.43389     ]; % Rigid-body inertia matrix

% UUV initial state
NED     = [0 0 0 0 0 0].';    % NED:x y z phi theta psi
NED_dot = [0 0 0 0 0 0].';    
vel     = [0 0 0 0 0 0].';    % Body: u v w p q r
a       = [0 0 0 0 0 0].';    % Body

% Rigid-Body Inertia Matrix
M_rb = [ m*eye(3)    -m*S(r_g);
         m*S(r_g)         I_g ];

% Added mass matrix
M_a = diag([16.8374 20.2748 35.3180 0.2165 0.6869 0.6157]);

% Mass matrix
M = M_rb + M_a;

% Linear damping coefficients
X_u = 0.3431;
Y_v = -0.0518;
Z_w = 0.5841;
K_p = -0.0064;
M_q = -0.04;
N_r = 0.1063;

D_lin = -diag([X_u Y_v Z_w K_p M_q N_r]);

% Cuadratic damping coefficients
X_uu = -111.7397;
Y_vv = -44.4058;
Z_ww = -157.1951;
K_pp = -0.4634;
M_qq = -0.2902;
N_rr = -2.2897;

dt=0.01;
for i = 0:dt:10    
    %------------ UUV dynamic model parametrization ----------------%
    % Rigid-body Coriolis-centripetal Matrix
    C_rb = [ zeros(3)             -m*S(vel(1:3))-m*S(vel(4:6))*S(r_g);
            -m*S(vel(1:3))+m*S(r_g)*S(vel(4:6))            -S(I_g*vel(4:6))];

    % Coreolis added mass matrix 
    C_a = [ 0 0 0   0 -M_a(3,3)*vel(3) M_a(2,2)*vel(2) ;
            0 0 0   M_a(3,3)*vel(3) 0 -M_a(1,1)*vel(1) ;
            0 0 0  -M_a(2,2)*vel(2) M_a(1,1)*vel(1) 0  ;
            0 -M_a(3,3)*vel(3) M_a(2,2)*vel(2)  0 -M_a(6,6)*vel(6) M_a(5,5)*vel(5) ;
            M_a(3,3)*vel(3) 0 -M_a(1,1)*vel(1)  M_a(6,6)*vel(6) 0 -M_a(4,4)*vel(4) ;
           -M_a(2,2)*vel(2) M_a(1,1)*vel(1) 0   -M_a(5,5)*vel(5) M_a(4,4)*vel(4) 0  ];

    % Coreolis matrix
    C = C_rb + C_a;

    % Restoring forces
    g_eta = [ (W-B)*sin(NED(5));
              -(W-B)*cos(NED(5))*sin(NED(4));
              -(W-B)*cos(NED(5))*cos(NED(4));
              -(r_g(2)*W-r_b(2)*B)*cos(NED(5))*cos(NED(4)) + (r_g(3)*W-r_b(3)*B)*cos(NED(5))*sin(NED(4));
               (r_g(3)*W-r_b(3)*B)*sin(NED(5))             + (r_g(1)*W-r_b(1)*B)*cos(NED(5))*cos(NED(4))
              -(r_g(1)*W-r_b(1)*B)*cos(NED(5))*sin(NED(4)) - (r_g(2)*W-r_b(2)*B)*sin(NED(5))          ];

    % Cuadratic Damping matrix
    D_cuad = -diag([X_uu*abs(vel(1)) Y_vv*abs(vel(2)) Z_ww*abs(vel(3)) ...
                   K_pp*abs(vel(4)) M_qq*abs(vel(5)) N_rr*abs(vel(6))]);
    
    %Damping matrix
    Damp = D_lin + D_cuad;

    NED = NED + NED_dot*dt;
    if abs(NED(4)) > pi 
        NED(4) = (NED(4)/abs(NED(4)))*(abs(NED(4))-2*pi);
    end
    if abs(NED(5)) > pi 
        NED(5) = (NED(5)/abs(NED(5)))*(abs(NED(5))-2*pi);
    end
    if abs(NED(6)) > pi 
        NED(6) = (NED(6)/abs(NED(6)))*(abs(NED(6))-2*pi);
        disp(NED(6))
    end
    %------------ UUV kinematics ----------------%
    % Linear velocities in NED / For a zxy rotation sequence
    NED_dot(1) = vel(1)*cos(NED(6))*cos(NED(5)) + vel(2)*[cos(NED(6))*sin(NED(5))*sin(NED(4))... 
        - sin(NED(6))*cos(NED(4))] + vel(3)*[sin(NED(6))*sin(NED(4))... 
        + cos(NED(6))*cos(NED(4))*sin(NED(5))];

    NED_dot(2) = vel(1)*sin(NED(6))*cos(NED(5)) + vel(2)*[cos(NED(6))*cos(NED(4))... 
        + sin(NED(4))*sin(NED(5))*sin(NED(6))]+ vel(3)*[sin(NED(5))*sin(NED(6))*cos(NED(4))... 
        - cos(NED(6))*sin(NED(4))];

    NED_dot(3) = -vel(1)*sin(NED(5)) + vel(2)*cos(NED(5))*sin(NED(4)) + vel(3)*cos(NED(5))*cos(NED(4));

    %Angular velocities in NED
    NED_dot(4) = vel(4) + vel(5)*sin(NED(4))*tan(NED(5)) + vel(6)*cos(NED(4))*tan(NED(5));

    NED_dot(5) = vel(5)*cos(NED(4)) - vel(6)*sin(NED(4));

    NED_dot(6) = vel(5)*(sin(NED(4))/cos(NED(5))) + vel(6)*(cos(NED(4))/cos(NED(5)));

    vel = vel + a*dt;
    a = M\(tau-(C*vel)-(Damp*vel)-g_eta);
    
    %disp(M_rb)
    %disp("Damping * vel / M")
    %disp(M\(Damp*vel))
    
    %disp("Damping * vel")
    %disp(Damp*vel)
    
    %disp("Coreolis * vel")
    %disp(C*vel)
    
    disp("Restoring moments and forces")
    disp(g_eta)
    
    %disp("tau")
    %disp(tau)
    
    %disp("Accel")
    %disp(a)
    
    disp("NED")
    disp(NED)
    
    disp("Velocities")
    disp(vel)
    
    disp(i)
end