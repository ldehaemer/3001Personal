classdef Robot < handle
    
    properties
        transform = Kinematics();
        Trajectory = Traj_planner();
        RobotModel = Model();
        myHIDSimplePacketComs;
        pol; 
        GRIPPER_ID = 1962
        CurrentGoal = zeros(1,3);%property that holds current set joint angles to be achieves
    end
    
    methods
        
        %The is a shutdown function to clear the HID hardware connection
        function  shutdown(self)
	    %Close the device
            self.myHIDSimplePacketComs.disconnect();
        end
        %Moves the arm from point 1 to 2 to 3 back to point 1 at a constant
        %velocity
        %Points 3x3
        function CSV =  runTriangleConstVelocity(self, Points)
            CSV = zeros(100000,4);
            %Navigate to point 1
            self.interpolate_jp(self.transform.ik3001(Points(:,1)).',1000)
            tic
            while(toc<1.05)
            end            
            Data = self.measured_js(1,0);
            CurrTargetAngle = Data(1,:); % 1x3 matrix of Joint Angles
            Velocity = 60;%mm/s
            time = norm(Points(:,2) - self.transform.PositionFK3001(CurrTargetAngle.'),2)/Velocity;
            prevTime = 0;
            count = 1;
            tic
            while(toc<time)
                Data = self.measured_js(1,0);
                Jacobian = self.transform.jacob3001(CurrTargetAngle.');
                Vector = Points(:,2) - self.transform.PositionFK3001(Data(1,:).');
                JointVelocities = inv(Jacobian(1:3,:))*((Vector/norm(Vector,2))*Velocity);
                CurrTargetAngle = CurrTargetAngle + JointVelocities.'*(toc - prevTime);
                prevTime = toc;
                self.servo_jp(CurrTargetAngle);
                CSV(count,1:3) = self.transform.PositionFK3001(Data(1,:).').';
                CSV(count,4) = toc;
                count = count + 1;
            end
            Data = self.measured_js(1,0);
            CurrTargetAngle = Data(1,:); % 1x3 matrix of Joint Angles
            Velocity = 60;%mm/s
            time = norm(Points(:,3) - self.transform.PositionFK3001(CurrTargetAngle.'),2)/Velocity;
            prevTime = 0;
            tic
            while(toc<time)
                Data = self.measured_js(1,0);
                Jacobian = self.transform.jacob3001(CurrTargetAngle.');
                Vector = Points(:,3) - self.transform.PositionFK3001(Data(1,:).');
                JointVelocities = inv(Jacobian(1:3,:))*((Vector/norm(Vector,2))*Velocity);
                CurrTargetAngle = CurrTargetAngle + JointVelocities.'*(toc - prevTime);
                prevTime = toc;
                self.servo_jp(CurrTargetAngle);
                CSV(count,1:3) = self.transform.PositionFK3001(Data(1,:).').';
                CSV(count,4) = toc;
                count = count + 1;
            end
            

            Data = self.measured_js(1,0);
            CurrTargetAngle = Data(1,:); % 1x3 matrix of Joint Angles
            Velocity = 60;%mm/s
            time = norm(Points(:,1) - self.transform.PositionFK3001(CurrTargetAngle.'),2)/Velocity;
            prevTime = 0;
            tic
            while(toc<time)
                Data = self.measured_js(1,0);
                Jacobian = self.transform.jacob3001(CurrTargetAngle.');
                Vector = Points(:,1) - self.transform.PositionFK3001(Data(1,:).');
                JointVelocities = inv(Jacobian(1:3,:))*((Vector/norm(Vector,2))*Velocity);
                CurrTargetAngle = CurrTargetAngle + JointVelocities.'*(toc - prevTime);
                prevTime = toc;
                self.servo_jp(CurrTargetAngle);
                CSV(count,1:3) = self.transform.PositionFK3001(Data(1,:).').';
                CSV(count,4) = toc;
                count = count + 1;
            end

            CSV = CSV(1:count-1,:);
        end
        
        % Create a packet processor for an HID device with USB PID 0x007
        function self = Robot(dev)
             self.myHIDSimplePacketComs = dev; 
            self.pol = java.lang.Boolean(false);
        end
        
        %Perform a command cycle. This function will take in a command ID
        %and a list of 32 bit floating point numbers and pass them over the
        %HID interface to the device, it will take the response and parse
        %them back into a list of 32 bit floating point numbers as well
        function com = command(self, idOfCommand, values)
                com= zeros(15, 1, 'single');
                try
                    ds = javaArray('java.lang.Double',length(values));
                    for i=1:length(values)
                        ds(i)= java.lang.Double(values(i));
                    end
                    % Default packet size for HID
                    intid = java.lang.Integer(idOfCommand);
                    self.myHIDSimplePacketComs.writeFloats(intid,  ds);
                    ret = 	self.myHIDSimplePacketComs.readFloats(intid) ;
                    for i=1:length(com)
                       com(i)= ret(i).floatValue();
                    end
                catch exception
                    getReport(exception)
                    disp('Command error, reading too fast');
                end
        end
        
        function com = read(self, idOfCommand)
                com= zeros(15, 1, 'single');
                try

                    % Default packet size for HID
                    intid = java.lang.Integer(idOfCommand);
                    ret = 	self.myHIDSimplePacketComs.readFloats(intid) ;
                    for i=1:length(com)
                       com(i)= ret(i).floatValue();
                    end
                catch exception
                  getReport(exception)
                    disp('Command error, reading too fast');
                end
        end
        
        function  write(self, idOfCommand, values)
                try
                    ds = javaArray('java.lang.Double',length(values));
                    for i=1:length(values)
                        ds(i)= java.lang.Double(values(i));
                    end
                    % Default packet size for HID
                    intid = java.lang.Integer(idOfCommand);
                    self.myHIDSimplePacketComs.writeFloats(intid,  ds,self.pol);

                catch exception
                    getReport(exception)
                    disp('Command error, reading too fast');
                end
        end
        
        % Specifies a position to the gripper
        function writeGripper(self, value)
            try
                ds = javaArray('java.lang.Byte',length(1));
                ds(1)= java.lang.Byte(value);
                intid = java.lang.Integer(self.GRIPPER_ID);
                self.myHIDSimplePacketComs.writeBytes(intid, ds, self.pol);
            catch exception
                getReport(exception)
                disp('Command error, reading too fast');
            end
        end
        
        % Opens the gripper
        function openGripper(self)
            self.writeGripper(180);
        end        
        % Closes the gripper
        function closeGripper(self)
            self.writeGripper(0);
        end

        %Takes in 1x3 array of servo joint angles and interpolation time
                %servo_jp()
        function interpolate_jp(self, ListOfServoJointAngles,InterpolationTime)
            ToBeSent = zeros(15,1,'single'); %create "packet", which is a 15 by 1 matrix
            ToBeSent(1) = InterpolationTime; % stores duration of interpolation in ms
            ToBeSent(2) = 0; %type of interpolation 1 for linear 0 for sinusoidal
            ToBeSent(3) = ListOfServoJointAngles(1,1); % joint 1/base angle is stored here
            ToBeSent(4) = ListOfServoJointAngles(1,2); % joint 2 angle is stored here
            ToBeSent(5) = ListOfServoJointAngles(1,3); % joint 3 angle is stored here
            self.write(1848,ToBeSent); %using write to send the matrix packet with stored data
            %1848 indicates the packet is of a certain type, with data
            %about interpolation mode, duration, and target motor positions
            self.CurrentGoal = ListOfServoJointAngles; %record current set joint positions
        end
        
        %takes in 1x3 matrix of joint angles to set motors to (bypasses
        %interpolation)
        function servo_jp(self, ListOfServoJointAngles)
            ToBeSent = zeros(15,1,'single'); %create packet matrix
            ToBeSent(1) = 1; %One ms interpolation time
            ToBeSent(2) = 0; %Linear Interpolation
            ToBeSent(3) = ListOfServoJointAngles(1,1); % base angle 1 pos
            ToBeSent(4) = ListOfServoJointAngles(1,2); % joint 2 pos
            ToBeSent(5) = ListOfServoJointAngles(1,3); % joint 3 pos
            self.write(1848,ToBeSent); % send packet as certain type, ID of 1848
            self.CurrentGoal = ListOfServoJointAngles;%record current set joint positions
        end
        
        %Returns the motor positions in degrees and/or the velocity of each
        %joint in 2x3 matrix
        %
        %Enter 1 to include the data, 0 to exclude
        function PosVelArray = measured_js(self,GETPOS,GETVEL)
            PosVelArray = zeros(2,3);%create a 2x3 matrix of zeros
            if(GETPOS)%if true, add positon data to first row
                RecievedPacket = self.read(1910); %read the next data packet that has motor position and setpoint data
                PosVelArray(1,1) = RecievedPacket(3); % store motor 1 pos
                PosVelArray(1,2) = RecievedPacket(5); % motor 2 pos
                PosVelArray(1,3) = RecievedPacket(7); % motor 3 pos
            end
            if(GETVEL) %if true, add velocity data to 2nd row
                RecievedPacket = self.read(1822); %read data packet of type 1822, with motor velocity data
                PosVelArray(2,1) = RecievedPacket(3); %motor 1 velocity
                PosVelArray(2,2) = RecievedPacket(6); % motor 2 velocity
                PosVelArray(2,3) = RecievedPacket(9); % motor 3 velocity
            end
        end
       
        %returns the current set joint positions for each joint
        function CurrGoal = goal_js(self)
            CurrGoal = self.CurrentGoal; %return current motor goal
        end

        %returns the current set point position of each joint in degrees
        function setPoints = setpoint_js(self)
            packet = self.read(1910); %read the packet containing set point and position data
            setPoints = [packet(2); packet(4); packet(6)]; %return just the set point data
        end


    %Which takes data from measured_js() and returns a 4x4 homogeneous transformation 
    %matrix based upon the current joint positions in degrees

    %function mat = measured_cp(self, PosVelArray) %takes data from measured_js() based upon the current joint positions in degrees
%              JointAngles = [PosVelArray(1,:)]'; %only take 1st of position values and then %transpose 
 %            mat = self.fk3001(JointAngles);

        function mat = measured_cp(self) %takes data from measured_js() based upon the current joint positions in degrees
             PosVelArray = self.measured_js(1,0);
             JointAngles = [PosVelArray(1,:)]';
             mat = self.transform.fk3001(JointAngles);
        end
        
        %returns the homogenous 4x4 transformation matrix for the goal
        %angles from goal_js()
        function mat = goal_cp(self)
           currGoalMat = self.goal_js(); %get the current goal angles matrix from goal_js()
           mat = self.transform.fk3001([currGoalMat(1,1); currGoalMat(1,2); currGoalMat(1,3)]); %input the goal_js() angles in 3x1 vector form into fk3001
         end 
        
        %returns a 4x4 transformation matrix representing the
        %transformation from base to end effector frames for the current
        %setpoint joint angles
        function TransformMatrix = setpoint_cp(self)
            TransformMatrix = self.transform.fk3001(self.setpoint_js()); %call fk3001 with the return value of setpoint_js which is a 3x1 array of setpoint joint angles
        end
        
        %{
        function takes in 3 arguments: 
        ● 3x4 matrix (3 joints or 3 components (x,y,z), 4 coefficients) of trajectory coefficients from cubic_traj(). 
        ● The total amount of time the trajectory should take. This must be the same duration you passed to 
            cubic_traj() to generate the trajectories for each joint.
        ● space type (joint or task): task = 1, joint = all else (default)
        
        Makes arm follow given trajectory based on coeff. matrix
        returns nx4 matrix of joint angles and timestamps
        %}
        function savedMat = run_trajectory(self,CoefficentMatrix,totalTime, spaceType)
            count = 1; %count var to help store data into separate rows
            savedMat = zeros(10000,7); %initialize data matrix
            tic %start clock
            while(toc < totalTime) %during trajectory time
                %CurrTime vars are used to assemble trajectory polynomial 
                CurrTime = toc; 
                CurrTime2 = CurrTime^2;
                CurrTime3 = CurrTime^3;
                CurrTime4 = CurrTime^4;
                CurrTime5 = CurrTime^5;
%                 saveD = self.measured_js(1,0); %Measure current joint angles
%                 savedMat(count,1:3) = saveD(1,:); %Save current joint angles in data matrix
%                 savedMat(count,4) = CurrTime; %also save current time of clock 
%                 count = count + 1; %increment count for next row of data

                %Check size of given coefficient matrix; if nx4, use cubic
                %poly, else use quintic poly
                if(size(CoefficentMatrix,2) == 4) %If given cubic function coefficients
                    posMatrix = [CoefficentMatrix(1,1) + CoefficentMatrix(1,2)*CurrTime + CoefficentMatrix(1,3)*CurrTime2 + CoefficentMatrix(1,4)*CurrTime3, ...
                                     CoefficentMatrix(2,1) + CoefficentMatrix(2,2)*CurrTime + CoefficentMatrix(2,3)*CurrTime2 + CoefficentMatrix(2,4)*CurrTime3, ...
                                     CoefficentMatrix(3,1) + CoefficentMatrix(3,2)*CurrTime + CoefficentMatrix(3,3)*CurrTime2 + CoefficentMatrix(3,4)*CurrTime3];
                else %if given quintic function coefficents 
                    posMatrix = [CoefficentMatrix(1,1) + CoefficentMatrix(1,2)*CurrTime + CoefficentMatrix(1,3)*CurrTime2 + CoefficentMatrix(1,4)*CurrTime3 + CoefficentMatrix(1,5)*CurrTime4 + CoefficentMatrix(1,6)*CurrTime5,...
                                     CoefficentMatrix(2,1) + CoefficentMatrix(2,2)*CurrTime + CoefficentMatrix(2,3)*CurrTime2 + CoefficentMatrix(2,4)*CurrTime3 + CoefficentMatrix(2,5)*CurrTime4 + CoefficentMatrix(2,6)*CurrTime5, ...
                                     CoefficentMatrix(3,1) + CoefficentMatrix(3,2)*CurrTime + CoefficentMatrix(3,3)*CurrTime2 + CoefficentMatrix(3,4)*CurrTime3 + CoefficentMatrix(3,5)*CurrTime4 + CoefficentMatrix(3,6)*CurrTime5];
                end

                if(spaceType == 1) %for task space input                   
                    self.servo_jp(self.transform.ik3001(posMatrix.').');
                else %for joint space input
                    self.servo_jp(posMatrix);
                end
                
                JAV = self.measured_js(1,1);
                PVector = self.transform.fdk3001(JAV(1,:).',JAV(2,:).');

                savedMat(count,1:6) = PVector.';
                savedMat(count,7) = toc;
                count = count+1;
%                 j = self.transform.jacob3001Fast(JAV(1,:).');
                
%                 if(self.transform.detectSingularity(j(1:3,:)))
% %                     self.RobotModel.plot_arm(JAV(1,:).',PVector(1:3,1),1);
%                 else
% %                     self.RobotModel.plot_arm(JAV(1,:).',PVector(1:3,1),0);
%                 end
              
            end
            savedMat = savedMat(1:count-1,:);
        end
            
        function data = run_spiral(self)
                data = zeros(100000,4);
                count = 1;
                tic
                while (toc < 40)
                    x = sin(toc) * 50 + 100;
                    y = cos(toc) * 50;
                    z = toc*2.5;
                    data(count,1:3) = [x,y,z];
                    data(count,4) = toc;
                    self.servo_jp(self.transform.ik3001([x;y;z]).')
                    count = count + 1;
                end
               
            end
        function solution =  ik_3001_numerical(self, Point,Velocity)
            TargetJAngles = self.transform.ik3001(Point).';
            Data = self.measured_js(1,0);
            CurrJAngles = Data(1,:);
            PointReached = 0;
            prevTime = 0;
            tic
            while PointReached == 0 
                Jacobian = self.transform.jacob3001(CurrJAngles.'); %Calculate the Jacobian
                Vector = Point - self.transform.PositionFK3001(CurrJAngles.'); %Calculate the vector from our current position to the target
                JointVelocities = inv(Jacobian(1:3,:))*((Vector/norm(Vector,2))*Velocity); %Calculate the Inst. Joint Velocity
                CurrJAngles = CurrJAngles + JointVelocities.'*(toc-prevTime); % Figure out the new Joint Angles with the given INST Joint Velocity
                prevTime = toc; %Update prevTime
                self.RobotModel.plot_arm(CurrJAngles.',Jacobian(1:3,:)*JointVelocities,0); %Plot the arm
                CurrPositionError = norm(self.transform.PositionFK3001(CurrJAngles.') - Point,2); %Figure out how close we are
                if CurrPositionError<3 %Algorithm has converged if the position error is small enough
                    self.interpolate_jp(CurrJAngles,1000);
                    tic
                    while toc<1.05
                    end
                    solution = CurrJAngles;
                    PointReached = 1;
                end
            end
        end
    end
end
