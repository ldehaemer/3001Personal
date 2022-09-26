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
Points = [91,177,77;108,-5,-122;20,160,70];
Data = zeros(200000,7);
tic
try
  SERV_ID = 1848;            % we will be talking to server ID 1848 on
                           % the Nucleo
  SERVER_ID_READ =1910;% ID of the read packet
  DEBUG   = true;          % enables/disables debug prints
  count = 1;
  tic
  for i = 1:3
      robot.interpolate_jp(robot.transform.ik3001(Points(:,i)).',2000);
      while toc < 2.05*i
          Angles = robot.measured_js(1,0);
          Data(count,1:3) = Angles(1,:);
          Data(count,4:6) = robot.transform.PositionFK3001(Angles(1,:).');
          Data(count,7) = toc;
          count = count +1;
      end
  end
      robot.interpolate_jp(robot.transform.ik3001(Points(:,1)).',2000);
      while toc < 2.05*4
          Angles = robot.measured_js(1,0);
          Data(count,1:3) = Angles(1,:);
          Data(count,4:6) = robot.transform.PositionFK3001(Angles(1,:).');
          Data(count,7) = toc;
          count = count +1;
      end

catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
robot.shutdown()

toc %stops the clock and reads the time that has passed
