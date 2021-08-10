vel = -[-1 -0.8 -0.6 -0.4 -0.2 0 0.2 0.4 0.6 0.8 1];

    % Drag forces and moments
% Forces obtained from solidworks CFD are equivalent to Drag forces
% experienced by the UUV
X = [-112.032 -71.838 -40.406 -17.973 -4.509 0 3.926 15.666 35.14 62.42 97.743];
Y = [-44.388 -28.335 -15.981 -7.108 -1.78 0 1.774 7.036 15.766 27.966 43.971];
Z = [-157.635 -101.307 -56.761 -25.338 -6.355 0 5.179 20.672 46.318 82.193 128.553];
K = [-0.464 -0.282	-0.169 -0.076 -0.02	0 0.023	0.085 0.148	0.326 0.494];
M = [-0.267 -0.134	-0.091 -0.044 -0.011 0 -0.006 -0.03	-0.065 -0.126 -0.265];
N = [-2.385 -1.555	-0.885 -0.4	-0.101 0 0.09 0.361	0.824 1.403	2.292];

disp("Surge")
surge_p = polyfit(vel(1:6),X(1:6),2); % To a cuadratic function
disp(surge_p);
surge_n = -polyfit(vel(6:11),X(6:11),2); % To a cuadratic function
%disp(surge_n);
%surge_c = (surge_p + surge_n)/2;
%disp(surge_c)

disp("Sway")
sway_p = polyfit(vel(1:6),Y(1:6),2); % To a cuadratic function
disp(sway_p);
sway_n = -polyfit(vel(6:11),Y(6:11),2); % To a cuadratic function
%disp(sway_n);
%sway_c = (sway_p + sway_n)/2;
%disp(sway_c)

disp("Heave")
heave_p = polyfit(vel(1:6),Z(1:6),2); % To a cuadratic function
disp(heave_p);
heave_n = -polyfit(vel(6:11),Z(6:11),2); % To a cuadratic function
%disp(heave_n);
%heave_c = (heave_p + heave_n)/2;
%disp(heave_c)

disp("Roll")
roll_p = polyfit(vel(1:6),K(1:6),2); % To a cuadratic function
disp(roll_p);
roll_n = -polyfit(vel(6:11),K(6:11),2); % To a cuadratic function
%disp(roll_n);
%roll_c = (roll_p + roll_n)/2;
%disp(roll_c)

disp("Pitch")
pitch_p = polyfit(vel(1:6),M(1:6),2); % To a cuadratic function
disp(pitch_p);
pitch_n = -polyfit(vel(6:11),M(6:11),2); % To a cuadratic function
%disp(pitch_n);
%pitch_c = (pitch_p + pitch_n)/2;
%disp(pitch_c)

disp("Yaw")
yaw_p = polyfit(vel(1:6),N(1:6),2); % To a cuadratic function
disp(yaw_p);
yaw_n = -polyfit(vel(6:11),N(6:11),2); % To a cuadratic function
%disp(yaw_n);
%yaw_c = (yaw_p + yaw_n)/2;
%disp(yaw_c)

%plot(vel(1:6),X(1:6));
%title('Surge Drag Positive Force');
%xlabel('Velocity [m/s]');
%ylabel('Drag Force [N]');

%fit(vel(1:6),X(1:6),'poly2');

%plot(vel(6:11),X(6:11));
%title('Surge Drag Negative Force');
%xlabel('Velocity [m/s]');
%ylabel('Drag Force [N]');

%plot(vel,Z);
%title('Sway Drag Force');
%xlabel('Velocity [m/s]');
%ylabel('Drag Force [N]');

%plot(vel,Z);
%title('Heave Drag Force');
%xlabel('Velocity [m/s]');
%ylabel('Drag Force [N]');

%plot(vel,K);
%title('Roll Drag Force');
%xlabel('Velocity [m/s]');
%ylabel('Drag Force [N]');

%plot(vel,M);
%title('Pitch Drag Force');
%xlabel('Velocity [m/s]');
%ylabel('Drag Force [N]');

%plot(vel,N);
%title('Yaw Drag Force');
%xlabel('Velocity [m/s]');
%ylabel('Drag Force [N]');