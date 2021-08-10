%Script to graph the simulink output 
figure
plot3(out.x,out.y,out.z,'Color','b','DisplayName', 'Trayectoria 1');
hold on
grid on
%Depending on the simulink file, the reference should be changed
%plot3(T1_Ref(:,1),T1_Ref(:,2),T1_Ref(:,3),'Color','k','DisplayName', 'Reference trajectory');
plot3(T2_Ref(:,1),T2_Ref(:,2),T2_Ref(:,3),'Color','k','DisplayName', 'Reference trajectory');
xlabel('X') 
ylabel('Y') 
zlabel('Z') 
