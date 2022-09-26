clear
clear java
clear classes;

vid = hex2dec('16c0');
pid = hex2dec('0486');

disp (vid);
disp (pid);

javaaddpath ../lib/SimplePacketComsJavaFat-0.6.4.jar;
import edu.wpi.SimplePacketComs.*;
import edu.wpi.SimplePacketComs.device.*;
import edu.wpi.SimplePacketComs.phy.*;
import java.util.*;
import org.hid4java.*;
version -java
myHIDSimplePacketComs=HIDfactory.get();
myHIDSimplePacketComs.setPid(pid);
myHIDSimplePacketComs.setVid(vid);
myHIDSimplePacketComs.connect();

% Create a PacketProcessor object to send data to the nucleo firmware
robot = Robot(myHIDSimplePacketComs); 
try
  SERV_ID = 1848;            % we will be talking to server ID 1848 on
                           % the Nucleo
  SERVER_ID_READ =1910;% ID of the read packet
  DEBUG   = true;          % enables/disables debug prints
Waypoint = [0,77,-12;0,0,22;0,86,33]; %coords of all three vertices of triangle
CSVExport = zeros(100,7); %create matrix to record csv data
Count = 1;
robot.interpolate_jp([0,0,0],1.75); %move to 0,0,0 for reset
tic
while(toc<2) %delay 2s
end

tic
for i = 1:3 %iterate through vertices of waypoint matrix
    robot.interpolate_jp(Waypoint(i,:),2000); %have arm go to vertex
    while(toc<i*2.25) %time coordination of when to record data after each vertex pos reached
        JointAngles = robot.measured_js(1,0); %record measured joint angles
        CurrMatrix = robot.measured_cp(); %record final HT matrix from base to EE
        CSVExport(Count,1:3) =  JointAngles(1,:); %store angle data in first 3 col
        CSVExport(Count,4:6) = CurrMatrix(1:3,4).'; %col 4-6 for x,y,z pos vector data of HT matrix, base to EE
        CSVExport(Count,7) = toc; %col 7 for time data
        Count = Count + 1; %increment to next waypoint 
    end
    
end

robot.interpolate_jp(Waypoint(1,:),2000); %go back to first waypoint for full triangular cycle
while(toc<4*2.25) %perform same operations as last while loop code
        JointAngles = robot.measured_js(1,0);
        CurrMatrix = robot.measured_cp();
        CSVExport(Count,1:3) =  JointAngles(1,:);
        CSVExport(Count,4:6) = CurrMatrix(1:3,4).';
        CSVExport(Count,7) = toc;
        Count = Count + 1;    
end


writematrix(CSVExport,'TriangleWaypointMotionPlanning'); %export CSV matrix with data as CSV file

catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
robot.shutdown()

toc %stops the clock and reads the time that has passed
