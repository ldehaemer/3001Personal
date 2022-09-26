figure
hold on
legend on
legend('Location','bestoutside')
%Graph joint angles vs time
plot(CSVExport(:,7),CSVExport(:,1),'LineStyle',':','DisplayName','Joint 1 Angle'); %plot joint 1 angle vs time
plot(CSVExport(:,7),CSVExport(:,2),'LineStyle','-','DisplayName','Joint 2 Angle');%plot joint 2 angle vs time
plot(CSVExport(:,7),CSVExport(:,3),'LineStyle','--','DisplayName','Joint 3 Angle');%plot joint 3 angle vs time

xlabel('time(s)')
ylabel('Joint Angle(Degrees)')

figure
hold on
legend on
legend('Location','bestoutside')
%Graph XZ plane data vs time
plot(CSVExport(:,7),CSVExport(:,4),'DisplayName','X Position(mm), w.r.t base frame'); %plot x vs time
plot(CSVExport(:,7),CSVExport(:,6),'DisplayName','Z Position(mm), w.r.t base frame'); %plot z vs time
xlabel('time(s)');
ylabel('End Effector Position(mm)');

figure
hold on
%Graph actual traversal of arm (z vs x pos data)
b = plot(CSVExport(:,4),CSVExport(:,6),'DisplayName','Robot Arm Trajectory(x-z plane)');
plot(51.2754,14.5137,'o','Color','r','MarkerFaceColor','r'); % plot triangle vertex 1
plot(92.7184,157.5393,'o','Color','r','MarkerFaceColor','r'); % plot triangle vertex 2
a = plot(139.6988,26.8643,'o','Color','r','MarkerFaceColor','r','DisplayName','Target Point'); %plot triangle vertex 3
xlabel('X Position(mm)');
ylabel('Z position(mm)');
legend([a,b], 'Location','bestoutside') %only use legend names for a and b vars
figure
hold on

%Graph actual traversal of arm (y vs x pos data)
b = plot(CSVExport(:,4),CSVExport(:,5),'DisplayName','Robot Arm Trajectory(x-y plane)');
a = plot(51.2754,0,'o','Color','r','MarkerFaceColor','r','DisplayName','Target Point'); %plot tri vertex 1
plot(92.7184,0,'o','Color','r','MarkerFaceColor','r'); %plot tri vertex 2
plot(139.6988,0,'o','Color','r','MarkerFaceColor','r'); % plot tri vertex 3
legend([a,b],'Location','bestoutside') %only use legend names for a and b vars
xlabel('X Position(mm)');
ylabel('Y position(mm)');