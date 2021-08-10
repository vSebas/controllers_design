L = 0.7326; %uuv length in m
H = 0.3626; %uuv heigth in m
W = 0.45; %uuv width in m

vl = [1 1 1].'; % linear velocity vector [u,v,w]

va = [1 1 1].'; % angular velocity vector [p,q,r]

rho = 1000;  %water density

PF = 0.15694469; %Projected area front [m^2] 
PS = 0.13786124; %Projected area side [m^2]
PT = 0.24014947; %Projected area top
A = zeros(6,6); %Added mass matrix
      
%3D empirical DNV
EMP3D=[1,0.68;2,0.36;3,0.24;4,0.19;5,0.15;6,0.14;7,0.11];

CA3D=spline(EMP3D(:,1),EMP3D(:,2));

%2D empirical DNV
EMP2D=[10,1.14,0.125;5,1.21,0.15;2,1.36,0.15;1,1.51,0.234;...
0.5,1.7,0.15;0.2,1.98,0.15;0.1,2.23,0.147];
CA2DT=spline(EMP2D(:,1),EMP2D(:,2));
CA2DR=spline(EMP2D(:,1),EMP2D(:,3));

%Coefficients
H3D=(H+W)/2; % Averaged Height( For 3D-est)
W3D=H3D; % Averaged Width ( For 3D-est)
CpXY=PT/(L*W); % Projected Area Coefficient XY
CpYZ=PF/(H*W); % Projected Area Coefficient YZ
CpXZ=PS/(L*H); % Projected Area Coefficient XZ

%Surge 3D
B1=L/H3D;
Ca1=ppval(CA3D,(B1));
Vr=L*H*W;
A(1,1)= Ca1*Vr*rho*(CpYZ)^2*CpXZ*CpXY;

%Surge 2D
B2=W/L;
Ca2=ppval(CA2DT,B2);
Ar1=pi*((W*0.5)^2);
A2D1=rho*Ca2*Ar1*(CpYZ)^2*CpXZ*CpXY;
Ai1=H*A2D1;
lambda=(A(1,1)/Ai1);
A(1,1)=Ai1*lambda;

%Sway-Heave
B3=L/W;
Ca3=ppval(CA2DT,B3);
Ar2=pi*(L*0.5)^2;
A2D2=rho*Ca3*Ar2*CpXZ^2*CpXY*CpYZ;
Ai2=A2D2*H;
A(2,2)=Ai2*lambda;
A2D3=rho*Ca3*Ar2*CpXY^2*CpXZ*CpYZ;
Ai3=A2D3*W;
A(3,3)=Ai3*lambda;

%Roll
B4=H/W;
Ca4=ppval(CA2DR,B4);
if (B4<=1)
A2D4=rho*Ca4*pi*(W*0.5)^4*CpYZ*CpXY*CpXZ;
else
A2D4=rho*Ca4*pi*(H*0.5)^4*CpYZ*CpXY*CpXZ;
end
Ai4=L*A2D4;
A(4,4)=Ai4*lambda;

%Pitch
B5=L/H;
Ca5=ppval(CA2DR,B5);
if(B5>=1)
A2D5=rho*Ca5*pi*(L*0.5)^4*CpYZ*CpXY*CpXZ;
else
A2D5=rho*Ca5*pi*(H*0.5)^4*CpYZ*CpXY*CpXZ;
end
Ai5=W*A2D5;
A(5,5)=Ai5*lambda;

%Yaw
B6=W/L;
Ca6=ppval(CA2DR,B6);
if(B6>=1)
A2D6=rho*Ca6*pi*(W*0.5)^4*CpYZ*CpXY*CpXZ;
else
A2D6=rho*Ca6*pi*(L*0.5)^4*CpYZ*CpXY*CpXZ;
end
Ai6=A2D6*H;
A(6,6)=Ai6*lambda;

CaV = [ 0 0 0   0 -A(3,3)*vl(3) A(2,2)*vl(2) ;
        0 0 0   A(3,3)*vl(3) 0 -A(1,1)*vl(1) ;
        0 0 0  -A(2,2)*vl(2) A(1,1)*vl(1) 0  ;
        0 -A(3,3)*vl(3) A(2,2)*vl(2)  0 -A(6,6)*va(3) A(5,5)*va(2) ;
        A(3,3)*vl(3) 0 -A(1,1)*vl(1)  A(6,6)*va(3) 0 -A(4,4)*va(1) ;
       -A(2,2)*vl(2) A(1,1)*vl(1) 0   -A(5,5)*va(2) A(4,4)*va(1) 0  ];

