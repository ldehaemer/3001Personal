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
Points = [91,177,77;108,-5,-122;20,160,70]; %Arbitrary points to have arm move to for trianglular pathing
tic
try
    tic
    robot.interpolate_jp([0,0,0],1000);
    while toc < 1.05
    end
    %     CSV = robot.runTriangleConstVelocity(Points); %PART 1
    
    %Parts 2 and 3
      Target = Points(:,3)
      Numerical_IK_Solution = robot.ik_3001_numerical(Target,50)
      FK = robot.transform.PositionFK3001(Numerical_IK_Solution.')


      Target
      Numerical_IK_Solution
      FK
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
 end

% Clear up memory upon termination
robot.shutdown()

toc %stops the clock and reads the time that has passed
