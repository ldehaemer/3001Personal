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
  SERV_ID = 1848;            % we will be talking to server ID 1848 on
                             % the Nucleo
  SERVER_ID_READ =1910;% ID of the read packet
  DEBUG   = true;          % enables/disables debug prints
%   Point1IK = robot.transform.ik3001(Points(:,1));
%   Point2IK = robot.transform.ik3001(Points(:,2));
%   Point3IK = robot.transform.ik3001(Points(:,3));
%   robot.interpolate_jp(Point1IK.',1000)
  tic
  while toc< 1.1
  end
  Trajectories = zeros(3,6,3);

  %Convert each triangle vertice to angle set for joints
  Point1IK = robot.transform.ik3001(Points(:,1));
  Point2IK = robot.transform.ik3001(Points(:,2));
  Point3IK = robot.transform.ik3001(Points(:,3));

  %Create quintic trajectories using desired start and end time values;
  %pos, vel, and accel are 0 to start and end
  Trajectories(1,:,1) = robot.Trajectory.quintic_traj(0,2,0,0,0,0,Points(1,1),Points(1,2)).';
  Trajectories(2,:,1) = robot.Trajectory.quintic_traj(0,2,0,0,0,0,Points(2,1),Points(2,2)).';
  Trajectories(3,:,1) = robot.Trajectory.quintic_traj(0,2,0,0,0,0,Points(3,1),Points(3,2)).';
 run1 = robot.run_trajectory(Trajectories(:,:,1),2,1);
  Trajectories(1,:,2) = robot.Trajectory.quintic_traj(0,2,0,0,0,0,Points(1,2),Points(1,3)).';
  Trajectories(2,:,2) = robot.Trajectory.quintic_traj(0,2,0,0,0,0,Points(2,2),Points(2,3)).';
  Trajectories(3,:,2) = robot.Trajectory.quintic_traj(0,2,0,0,0,0,Points(3,2),Points(3,3)).';
 run2 = robot.run_trajectory(Trajectories(:,:,2),2,1);
  Trajectories(1,:,3) = robot.Trajectory.quintic_traj(0,2,0,0,0,0,Points(1,3),Points(1,1)).';
  Trajectories(2,:,3) = robot.Trajectory.quintic_traj(0,2,0,0,0,0,Points(2,3),Points(2,1)).';
  Trajectories(3,:,3) = robot.Trajectory.quintic_traj(0,2,0,0,0,0,Points(3,3),Points(3,1)).';
 run3 = robot.run_trajectory(Trajectories(:,:,3),2,1);

 CSV = zeros(size(run1,1) + size(run2,1) + size(run3,1), 7);
 CSV(1:size(run1,1),:) = run1;
 CSV(size(run1,1)+1:size(run1,1)+size(run2,1),:) = run2;
 CSV(size(run1,1)+size(run2,1)+1:end,:) = run3;
  
 
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
 end

% Clear up memory upon termination
robot.shutdown()

toc %stops the clock and reads the time that has passed
