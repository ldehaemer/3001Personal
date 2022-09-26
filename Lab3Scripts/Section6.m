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


robot = Robot(myHIDSimplePacketComs); 
Points = [91,177,77;108,-5,-122;20,160,70]; %arbitrary desired triangle vertice points
tic
try
  SERV_ID = 1848;            % we will be talking to server ID 1848 on
                             % the Nucleo
  SERVER_ID_READ =1910;% ID of the read packet
  DEBUG   = true;          % enables/disables debug prints

  %Convert desired triangle vertices to angle coords for each joint
  Point1IK = robot.transform.ik3001(Points(:,1));
  Point2IK = robot.transform.ik3001(Points(:,2));
  Point3IK = robot.transform.ik3001(Points(:,3));

  robot.interpolate_jp(Point1IK.',1000)
  tic
  while toc< 1.1
  end
  Trajectories = zeros(3,4,3);

  %create cubic trajectories for each point in joint space
  Trajectories(1,:,1) = robot.Trajectory.cubic_traj(0,.5,0,0,Point1IK(1,1),Point2IK(1,1)).';
  Trajectories(2,:,1) = robot.Trajectory.cubic_traj(0,0.5,0,0,Point1IK(2,1),Point2IK(2,1)).';
  Trajectories(3,:,1) = robot.Trajectory.cubic_traj(0,0.5,0,0,Point1IK(3,1),Point2IK(3,1)).';
 run1 =robot.run_trajectory(Trajectories(:,:,1),0.65,0);
  Trajectories(1,:,2) = robot.Trajectory.cubic_traj(0,0.5,0,0,Point2IK(1,1),Point3IK(1,1)).';
  Trajectories(2,:,2) = robot.Trajectory.cubic_traj(0,0.5,0,0,Point2IK(2,1),Point3IK(2,1)).';
  Trajectories(3,:,2) = robot.Trajectory.cubic_traj(0,0.5,0,0,Point2IK(3,1),Point3IK(3,1)).';
 run2 = robot.run_trajectory(Trajectories(:,:,2),0.65,0);
  Trajectories(1,:,3) = robot.Trajectory.cubic_traj(0,0.75,0,0,Point3IK(1,1),Point1IK(1,1)).';
  Trajectories(2,:,3) = robot.Trajectory.cubic_traj(0,0.75,0,0,Point3IK(2,1),Point1IK(2,1)).';
  Trajectories(3,:,3) = robot.Trajectory.cubic_traj(0,0.75,0,0,Point3IK(3,1),Point1IK(3,1)).';
 run3 = robot.run_trajectory(Trajectories(:,:,3),.85,0);

 JArun1 = run1;
 JArun2 = run2;
 JArun3 = run3;

% Create a PacketProcessor object to send data to the nucleo firmware
robot = Robot(myHIDSimplePacketComs); 
Points = [91,177,77;108,-5,-122;20,160,70];
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
  Trajectories = zeros(3,4,3);
  %create cubic trajectories for each point in task space
  Trajectories(1,:,1) = robot.Trajectory.cubic_traj(0,.5,0,0,Points(1,1),Points(1,2)).';
  Trajectories(2,:,1) = robot.Trajectory.cubic_traj(0,0.5,0,0,Points(2,1),Points(2,2)).';
  Trajectories(3,:,1) = robot.Trajectory.cubic_traj(0,0.5,0,0,Points(3,1),Points(3,2)).';
 run1 =robot.run_trajectory(Trajectories(:,:,1),0.65,1);
  Trajectories(1,:,2) = robot.Trajectory.cubic_traj(0,0.5,0,0,Points(1,2),Points(1,3)).';
  Trajectories(2,:,2) = robot.Trajectory.cubic_traj(0,0.5,0,0,Points(2,2),Points(2,3)).';
  Trajectories(3,:,2) = robot.Trajectory.cubic_traj(0,0.5,0,0,Points(3,2),Points(3,3)).';
 run2 = robot.run_trajectory(Trajectories(:,:,2),0.65,1);
  Trajectories(1,:,3) = robot.Trajectory.cubic_traj(0,0.75,0,0,Points(1,3),Points(1,1)).';
  Trajectories(2,:,3) = robot.Trajectory.cubic_traj(0,0.75,0,0,Points(2,3),Points(2,1)).';
  Trajectories(3,:,3) = robot.Trajectory.cubic_traj(0,0.75,0,0,Points(3,3),Points(3,1)).';
 run3 = robot.run_trajectory(Trajectories(:,:,3),.85,1);

  %Testing times to reach each point
%  tic
%  robot.servo_jp(Point3IK.');
%  while toc<1
%      JointAngles = robot.measured_js(1,0);
%      if abs(sum(JointAngles(1,:).' - Point3IK,"all")) < 2
%          toc
%          break
%      end
%  end

  
 
  
 
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end
end

% Clear up memory upon termination
robot.shutdown()

toc %stops the clock and reads the time that has passed
