close all
% Quintic

%Below code adds time matrices together in order to plot all point to point
%graphs on the same figure (each graph was generated with a new tic toc
%command)
timematrix1 = run1(:,4);
timematrix2 = run1(end,4)+ run2(:,4);
timematrix3 = timematrix2(end) + run3(:,4);

Kine = Kinematics(); %create kinematics object

%Code below converts each point to point data matrix of angles into x,y,z
%coordinates
run1FK = zeros(size(run1,1),4);
for i = 1:size(run1FK,1)
    run1FK(i,4) = run1(i,4);
    run1FK(i,1:3) = Kine.PositionFK3001(run1(i,:).').';
end

run2FK = zeros(size(run2,1),4);
for i = 1:size(run2FK,1)
    run2FK(i,4) = run2(i,4);
    run2FK(i,1:3) = Kine.PositionFK3001(run2(i,:).').';
end

run3FK = zeros(size(run3,1),4);
for i = 1:size(run3FK,1)
    run3FK(i,4) = run3(i,4);
    run3FK(i,1:3) = Kine.PositionFK3001(run3(i,:).').';
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Task Space position vs time plots
figure
hold on
plot(run1(:,4),run1(:,1))
plot(run1(:,4),run1(:,2))
plot(run1(:,4),run1(:,3))
title('Position Point 1 to Point 2')
ylabel('mm')
xlabel('Time (s)')
legend('x','y','z')


plot(timematrix2,run2(:,1))
plot(timematrix2,run2(:,2))
plot(timematrix2,run2(:,3))
title('Position Point 2 to Point 3')
ylabel('mm')
xlabel('Time (s)')
legend('x','y','z')

plot(timematrix3,run3(:,1))
plot(timematrix3,run3(:,2),"Color"," #99a3a4")
plot(timematrix3,run3(:,3),"Color"," k")
title('Task Space Position vs Time')
ylabel('mm')
xlabel('Time (s)')
legend('x','y','z')
legend('X Point 1 to 2','Y Point 1 to 2','Z Point 1 to 2','X Point 2 to 3','Y Point 2 to 3','Z Point 2 to 3','X Point 3 to 1','Y Point 3 to 1','Z Point 3 to 1')

%Position^^^^^
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Velocity vs time plots
figure
hold on

% use gradient() to calculate derivative of position data to get velocity
% data
xdot1 = gradient(run1(:,1)) ./ gradient(run1(:,4));
ydot1 = gradient(run1(:,2)) ./ gradient(run1(:,4));
zdot1 = gradient(run1(:,3)) ./ gradient(run1(:,4));

plot(timematrix1,xdot1)
plot(timematrix1,ydot1,"LineStyle","-.") %Differentiate y lines from x and z
plot(timematrix1,zdot1,"LineStyle",":")%Differentiate z lines from x and y
title('Velocity Point 1 to Point 2')
ylabel('mm/s')
xlabel('Time (s)')
legend('x dot','y dot','z dot')

xdot2 = gradient(run2(:,1)) ./ gradient(run2(:,4));
ydot2 = gradient(run2(:,2)) ./ gradient(run2(:,4));
zdot2 = gradient(run2(:,3)) ./ gradient(run2(:,4));

plot(timematrix2,xdot2)
plot(timematrix2,ydot2,"LineStyle","-.")
plot(timematrix2,zdot2,"LineStyle",":")
title('Velocity Point 2 to Point 3')
ylabel('mm/s')
xlabel('Time (s)')
legend('x dot','y dot','z dot')

xdot3 = gradient(run3(:,1)) ./ gradient(run3(:,4));
ydot3 = gradient(run3(:,2)) ./ gradient(run3(:,4));
zdot3 = gradient(run3(:,3)) ./ gradient(run3(:,4));
plot(timematrix3,xdot3)
plot(timematrix3,ydot3,"Color"," #99a3a4", "LineStyle","-.")
plot(timematrix3,zdot3,"Color","k", "LineStyle",":")
title('Task Space Velocity vs Time')
ylabel('mm/s')
xlabel('Time (s)')
legend('x dot','y dot','z dot')
legend('X Point 1 to 2','Y Point 1 to 2','Z Point 1 to 2','X Point 2 to 3','Y Point 2 to 3','Z Point 2 to 3','X Point 3 to 1','Y Point 3 to 1','Z Point 3 to 1')



%Velocity^^^^^
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Acceleration vs time plots

figure
hold on

% use gradient() to calculate derivative of velocity data to get
% acceleration data
xdotdot1 = gradient(xdot1) ./ gradient(run1(:,4))
ydotdot1 = gradient(ydot1) ./ gradient(run1(:,4))
zdotdot1 = gradient(zdot1) ./ gradient(run1(:,4))
plot(timematrix1,xdotdot1)
plot(timematrix1,ydotdot1, "LineStyle","-.")
plot(timematrix1,zdotdot1, "LineStyle",":")
title('Task Space Acceleration vs Time')
ylabel('mm/s^2')
xlabel('Time (s)')
legend('x dotdot','y dotdot','z dotdot')


% use gradient() to calculate derivative of velocity data to get
% acceleration data
xdotdot2 = gradient(xdot2) ./ gradient(run2(:,4))
ydotdot2 = gradient(ydot2) ./ gradient(run2(:,4))
zdotdot2 = gradient(zdot2) ./ gradient(run2(:,4))
plot(timematrix2,xdotdot2)
plot(timematrix2,ydotdot2, "LineStyle","-.")
plot(timematrix2,zdotdot2, "LineStyle",":")
title('Acceleration Point 2 to Point 3')
ylabel('mm/s^2')
xlabel('Time (s)')
legend('x dotdot','y dotdot','z dotdot')


hold on
% use gradient() to calculate derivative of velocity data to get
% acceleration data
xdotdot3 = gradient(xdot3) ./ gradient(run3(:,4))
ydotdot3 = gradient(ydot3) ./ gradient(run3(:,4))
zdotdot3 = gradient(zdot3) ./ gradient(run3(:,4))
plot(timematrix3,xdotdot3)
plot(timematrix3,ydotdot3,"Color"," #99a3a4", "LineStyle","-.")
plot(timematrix3,zdotdot3,"Color","k", "LineStyle",":")
title('Acceleration Point 3 to Point 1')
ylabel('mm/s^2')
xlabel('Time (s)')
legend('x dotdot','y dotdot','z dotdot')
legend('X Point 1 to 2','Y Point 1 to 2','Z Point 1 to 2','X Point 2 to 3','Y Point 2 to 3','Z Point 2 to 3','X Point 3 to 1','Y Point 3 to 1','Z Point 3 to 1')



%Quintic Acceleration XYZ^^^^^^^^
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%%CUBIC JOINT ANGLEs
% figure
% hold on
% 
% plot(timematrix1, run1(:,1),"LineWidth",2)
% plot(timematrix1, run1(:,2),"LineWidth",2)
% plot(timematrix1, run1(:,3),"LineWidth",2)
% 
% 
% 
% hold on
% plot(timematrix2, run2(:,1),"LineWidth",2)
% plot(timematrix2, run2(:,2),"LineWidth",2)
% plot(timematrix2, run2(:,3),"LineWidth",2)
% 
% 
% 
% 
% hold on
% plot(timematrix3, run3(:,1), "LineWidth",2)
% plot(timematrix3, run3(:,2),"LineWidth",2, "Color"," #99a3a4")
% plot(timematrix3, run3(:,3),"LineWidth",2, "Color",	'k')
% title('Position Joint Angle vs Time')
% xlabel('Time (s)')
% ylabel('Joint Angle (degrees)')
% legend('Joint Angle 1 Point 1 to 2','Joint Angle 2 Point 1 to 2','Joint Angle 3 Point 1 to 2','Joint Angle 1 Point 2 to 3','Joint Angle 2 Point 2 to 3','Joint Angle 3 Point 2 to 3','Joint Angle 1 Point 3 to 1','Joint Angle 2 Point 3 to 1','Joint Angle 3 Point 3 to 1')
% 
% %JA^^^^
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
figure
hold on
view(3)
%Generate 3D plot
plot3(run1(:,1),run1(:,2),run1(:,3),"Color","K")
plot3(run2(:,1),run2(:,2),run2(:,3),"Color","r")
plot3(run3(:,1),run3(:,2),run3(:,3),"Color","b")
title('3D Motion Profile')
xlabel('mm')
ylabel('mm')
zlabel('mm')
legend('Point 1 to 2', 'Point 2 to 3','Point 3 to 1')
