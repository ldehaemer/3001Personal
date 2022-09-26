close all
Exportin = csvread("Exportin.csv"); %reading from csv to readable matrix
Exportin1 = csvread("Exportin1.csv"); %These are matrices that store csv data
Exportin2 = csvread("Exportin2.csv");
Exportin3 = csvread("Exportin3.csv");
Exportnonin = csvread("Exportnonin.csv");
Exportnonin1 = csvread("Exportnonin1.csv");
Exportnonin2 = csvread("Exportnonin2.csv");
Exportnonin3 = csvread("Exportnonin3.csv");
Export45in1 = csvread("Exportnonin45in1.csv");
Export45in2 = csvread("Exportnonin45in2.csv");
Export45in3 = csvread("Exportnonin45in3.csv");
Export45nonin1 = csvread("Exportnonin45nonin1.csv");
Export45nonin2 = csvread("Exportnonin45nonin2.csv");
Export45nonin3 = csvread("Exportnonin45nonin3.csv");


%making figure with plots of Noninterpolated angle postitions over time (0 to 45 base angle only)
figure %create a new figure (window)
%subplot(2,2,1)
hold on %makes sure all plots go on one figure
plot(Export45nonin1(:,4),Export45nonin1(:,1),'DisplayName','Base Angle Noninterpolated Trial 1','Color','g');
xlabel('Time (s)')
ylabel('Angle (degrees)');
title("Moving Joints from [0,0,0] to [45,0,0] No Interpolation")

%below plot functions take in the fourth col of the data matrix that
%contains timestamps for the x-axis. The y-axis uses data from the relevant
%columns for each motor angle position

%each line is colored differently and labeled on the legend using special
%parameters for plot()
plot(Export45nonin1(:,4),Export45nonin1(:,2),'DisplayName','Joint 2 Angle Noninterpolated Trial 1','Color','b');
plot(Export45nonin1(:,4),Export45nonin1(:,3),'DisplayName','Joint 3 Angle Noninterpolated Trail 1', "Color",'r');

plot(Export45nonin2(:,4),Export45nonin2(:,1),'DisplayName','Base Angle Noninterpolated Trial 2','Color','m');
plot(Export45nonin2(:,4),Export45nonin2(:,2),'DisplayName','Joint 2 Angle Noninterpolated Trial 2','Color','k');
plot(Export45nonin2(:,4),Export45nonin2(:,3),'DisplayName','Joint 3 Angle Noninterpolated', "Color",'y');
%legend('Location','southeastoutside') %places the legend in the bottom right corner outside the graph

plot(Export45nonin3(:,4),Export45nonin3(:,1),'DisplayName','Base Angle Noninterpolated Trial 3','Color','#0072BD');
plot(Export45nonin3(:,4),Export45nonin3(:,2),'DisplayName','Joint 2 Angle Noninterpolated Trial 3','Color','#D95319');
plot(Export45nonin3(:,4),Export45nonin3(:,3),'DisplayName','Joint 3 Angle Noninterpolated Trial 3', "Color",'#7E2F8E');
legend('Location','southeastoutside') %places the legend in the bottom right corner outside the graph


%creates a histogram for the incremental step time between movements
%hold off
figure
NonInBig45 =  [Export45nonin3; Export45nonin2; Export45nonin1]; %concatenate all 3 non interpolated matrices
[row1,col] = size(NonInBig45); %extract the number of rows and columns from the resulting matrix
TimeStampDiffnoni = zeros(row1-2,1); %create a new matrix of zeros
for i = 1:row1-3 %iterate through each row and find the difference b/w the current time stamp i and the next row
   TimeStampDiffnoni(i,1) = NonInBig45(i+1,4)-NonInBig45(i,4);
end

TimeStampDiffnoni = TimeStampDiffnoni * 1000; %convert seconds to milliseconds 
histogram(TimeStampDiffnoni,[0:.10:5]) %create a histogram from range 0 to 5, with a stride of .10 for each bin
xlabel("Packet Time (ms)")
ylabel("Number of Packets")
title('Incremental Timestep for Recieved Packets Noninterpolated data going from [0,0,0] to [45,0,0]')
xlim([0,5])

medianNI = median(TimeStampDiffnoni) %calculate median of data
meanNI = mean(TimeStampDiffnoni) %mean
minNI = min(TimeStampDiffnoni) %min
maxNI = max(TimeStampDiffnoni) %max
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%making figure with plots of Interpolated angle postitions over time (0 to
%45 base angle only)
figure 
%subplot(2,2,1)
hold on %makes sure all plots go on one figure
plot(Export45in1(:,4),Export45in1(:,1),'DisplayName','Base Angle Interpolated Trial 1','Color','g');
xlabel('Time (s)')
ylabel('Angle (degrees)');
title("Moving Joints from [0,0,0] to [45,0,0] 4 Sec Interpolation")
plot(Export45in1(:,4),Export45in1(:,2),'DisplayName','Joint 2 Angle Interpolated Trial 1','Color','b');
plot(Export45in1(:,4),Export45in1(:,3),'DisplayName','Joint 3 Angle Interpolated Trail 1', "Color",'r');

plot(Export45in2(:,4),Export45in2(:,1),'DisplayName','Base Angle Interpolated Trial 2','Color','m');
plot(Export45in2(:,4),Export45in2(:,2),'DisplayName','Joint 2 Angle Interpolated Trial 2','Color','k');
plot(Export45in2(:,4),Export45in2(:,3),'DisplayName','Joint 3 Angle Interpolated', "Color",'y');
%legend('Location','southeastoutside') %places the legend in the bottom right corner outside the graph

plot(Export45in3(:,4),Export45in3(:,1),'DisplayName','Base Angle Interpolated Trial 3','Color','#0072BD');
plot(Export45in3(:,4),Export45in3(:,2),'DisplayName','Joint 2 Angle Interpolated Trial 3','Color','#D95319');
plot(Export45in3(:,4),Export45in3(:,3),'DisplayName','Joint 3 Angle Interpolated Trial 3', "Color",'#7E2F8E');
legend('Location','southeastoutside') %places the legend in the bottom right corner outside the graph

hold off
figure

%performing same incremental timestep calculations done previously with
%more comments above
InBig45 =  [Export45in3; Export45in2; Export45in1];
[row2,col] = size(InBig45);
TimeStampDiff = zeros(row2-2,1);
for i = 1:row2-2
   TimeStampDiffI(i,1) = InBig45(i+1,4)-InBig45(i,4);
end
TimeStampDiffI = TimeStampDiffI * 1000;
histogram(TimeStampDiffI,[0:.10:5])
xlabel("Packet Time (ms)")
ylabel("Number of Packets")
title('Incremental Timestep for Recieved Packets Interpolated data going from [0,0,0] to [45,0,0]')

medianI = median(TimeStampDiffI) %median
meanI = mean(TimeStampDiffI) %mean
minI = min(TimeStampDiffI) %min
maxI = max(TimeStampDiffI) %max

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% %making figure with plots of Interpolated angle postitions vs
% % Noninterpolated angle postitions over time 
 figure 
% %subplot(2,2,1)
 hold on %makes sure all plots go on one figure
 plot(Exportin(:,4),Exportin(:,1),'DisplayName','Base Angle Interpolated','Color','g');
xlabel('Time (s)')
ylabel('Angle (degrees)');
title("Moving Joints from [0,0,0] to [34,67,22] 4 Sec Interpolation vs Noninterpolated")
% %legend
% 
%subplot(2,2,2)

plot(Exportin(:,4),Exportin(:,2),'DisplayName','Joint 2 Angle Interpolated','Color','b');
%xlabel('time')
%ylabel('Joint 2 Angle (degrees)');
%title("Joint 2 Angle vs. Time")
%legend
%ylim([-3,5])

%subplot(2,2,3)
plot(Exportin(:,4),Exportin(:,3),'DisplayName','Joint 3 Angle Interpolated', "Color",'r');
%xlabel('time')
%ylabel('Joint 3 Angle (degrees)');
%title("Joint 3 Angle vs. Time");
%legend
%ylim([-3,5])

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%plotting non interpolated angles, which will be overlaid on the graph of
%interpolated angles
plot(Exportnonin(:,4),Exportnonin(:,1),'DisplayName','Base Angle Noninterpolated','Color','m');
plot(Exportnonin(:,4),Exportnonin(:,2),'DisplayName','Joint 2 Angle Noninterpolated','Color','k');
plot(Exportnonin(:,4),Exportnonin(:,3),'DisplayName','Joint 3 Angle Noninterpolated', "Color",'y');
legend('Location','southeastoutside') %places the legend in the bottom right corner outside the graph

%Below is previously used histogram code that is solely for reference

% %creates a histogram
% hold off
% figure
% TimeStampDiff = zeros(count-2,1);
% for i = 1:count-2
%     TimeStampDiff(i,1) = CSVExport(i+1,4)-CSVExport(i,4);
% end
% TimeStampDiff = TimeStampDiff * 1000;
% histogram(TimeStampDiff)
% xlabel("Packet Time (ms)")
% ylabel("Number of Packets")
% title('Incremental Timestep for Recieved Packets out of 6547 total packets')
% xlim([0,5])