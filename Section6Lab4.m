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
tic
try
    %send the robot to an arbitrary NON singularity
    robot.interpolate_jp([45,45,45],1000);
    tic
    while toc<1.05
    end

    %send the robot to a singularity
    CSV = zeros(100000,5);
    count = 1;
    robot.interpolate_jp([0,0,-90],4000)
    tic
    while toc<4.01
        Data = robot.measured_js(1,0);
        CSV(count,1:3) = robot.transform.PositionFK3001(Data(1,:).').';
        Jacob = robot.transform.jacob3001Fast(Data(1,:).');
        Jacob = Jacob(1:3,:);
        CSV(count,4) = det(Jacob);
        CSV(count,5) = toc;
        count = count + 1;
    end
    CSV = CSV(1:count-1,:);

 
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
 end

% Clear up memory upon termination
robot.shutdown()

toc %stops the clock and reads the time that has passed
