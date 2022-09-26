%%
% RBE3001 - Laboratory 1 - Team 17
% 
% Instructions
% ------------
% Welcome again! This MATLAB script is your starting point for Lab
% 1 of RBE3001. The sample code below demonstrates how to establish
% communication between this script and the Nucleo firmware, send
% setpoint commands and receive sensor data.
% 
% IMPORTANT - understanding the code below requires being familiar
% with the Nucleo firmware. Read that code first.

% Lines 15-37 perform necessary library initializations. You can skip reading
% to line 38.
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

  % Instantiate a packet - the following instruction allocates 60
  % bytes for this purpose. Recall that the HID interface supports
  % packet sizes up to 64 bytes.
  packet = zeros(15, 1, 'single');

  % The following code generates a sinusoidal trajectory to be
  % executed on joint 1 of the arm and iteratively sends the list of
  % setpoints to the Nucleo firmware. 
%   viaPts = [0,0,0];

%34,67,22
%1: -50,20,10
%2: 74,56,52
%3: 27,-10,15
%robot.interpolate_jp([0,0,0],4000); %move base joint 45deg with 4s interpolation

figure %create a new figure

robot.interpolate_jp([0,0,30],4000); %set arm to [0,0,30] angle position
tic %start counting in secs
while(toc<4) %while less than 4s has passed, do nothing (this is used as a delay)
    
end
HTMatricies = zeros(4,4,10); %create HT matrices variable that stores all 10 4x4 HT matrices for part 4

for i = 1:10 %part 4: create random configurations for joint angles. Do this 10 times (automated process)
robot.interpolate_jp([round(-90+rand()*180),round(-45+rand()*90), round(-45+rand()*90)],2000)
tic 
while(toc<2) %for 2 secs
    Joints = robot.measured_js(1,0); %measure joint angles, store in Joints var
    Joints = Joints(1,:).'  %remove second row, which has velocity data; then transpose to 3x1
    robot.RobotModel.plot_arm(Joints) %plot joint positions
end
currHTMatrix = robot.goal_cp() %store current HT matrix for goal positions
HTMatricies(:,:,i) = currHTMatrix;
robot.interpolate_jp([0,0,0],1000); %reset to [0,0,0] position
tic
while(toc<1.5) %for 1.5s
    Joints = robot.measured_js(1,0); %measure joint angles, store in Joints var
    Joints = Joints(1,:).'  %remove second row, which has velocity data; then transpose to 3x1
    robot.RobotModel.plot_arm(Joints) %plot joint positions
end

end



% tic
% while(toc<30)
%     Joints = robot.measured_js(1,0);
%     Joints = Joints(1,:).'
%     robot.RobotModel.plot_arm(Joints)
% end
% 
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
robot.shutdown()

toc %stops the clock and reads the time that has passed
