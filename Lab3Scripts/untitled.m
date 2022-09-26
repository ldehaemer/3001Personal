data = Data(1:2816,:);
figure
hold on
plot(data(:,7),data(:,1),'DisplayName','Joint 1')
plot(data(:,7),data(:,2),'DisplayName','Joint 2')
plot(data(:,7),data(:,3),'DisplayName','Joint 3')
legend('Location','bestoutside')
xlabel('Time(s)')
ylabel('$Joint Angle (^\circ)$','Interpreter','latex')
title('Joint Angles versus Time')

figure
hold on
plot(data(:,7),data(:,4),'DisplayName','x')
plot(data(:,7),data(:,5),'DisplayName','y')
plot(data(:,7),data(:,6),'DisplayName','z')
legend('Location','bestoutside')
xlabel('Time(s)')
ylabel('Position(mm)')
title('End Effector Position versus Time')

figure
plot3(data(:,4),data(:,5),data(:,6))
xlabel('x')
ylabel('y')
zlabel('z')
title('3D motion profile')
