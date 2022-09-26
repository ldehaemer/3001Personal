classdef Kinematics < handle
    properties
        Solutions  = zeros(3,16);
        CurrHTMatricies = zeros(4,4,4); %This matrix stores the HT matricies for EACH frame w.r.t. the base frame
        %e.g. t(0 -> 1), t(0 ->2), t(0 -> 3), t(0 ->4)


    end


    methods
        function obj = Kinematics()
            obj.CurrHTMatricies(:,:,1) = [1,0,0,0;0,1,0,0;0,0,1,55;0,0,0,1];
        end
        % This method takes in a 1x4 array corresponding to a row of the DH parameter table for
        % a given link. It then generates the associated intermediate transformation and returns
        % a corresponding symbolic 4x4 homogeneous transformation matrix.
        function mat = dh2mat(~, dhParam)  %dhParama = 1x4 array of DH paramaters, ordered (theta, d, a, alpha), for 1 link
            mat = [cosd(dhParam(1,1)),-sind(dhParam(1,1))*cosd(dhParam(1,4)), sind(dhParam(1,1))*sind(dhParam(1,4)), dhParam(1,3)*cosd(dhParam(1,1));
                sind(dhParam(1,1)),cosd(dhParam(1,1))*cosd(dhParam(1,4)), -cosd(dhParam(1,1))*sind(dhParam(1,4)), dhParam(1,3)*sind(dhParam(1,1));
                0,sind(dhParam(1,4)),cosd(dhParam(1,4)),dhParam(1,2);
                0,0,0,1];

        end

        % this method takes in an nx4 array corresponding to the n rows
        % of  the  full  DH  parameter  table.  It  then  generates  a  corresponding  symbolic  4x4
        % homogeneous  transformation  matrix  for  the  composite  transformation.
        function mat = dh2fk(self, dhTable) %dhTable = nx4 matrix
            mat = eye(4,4); % create 4x4 identity matrix
            for i = 1:size(dhTable,1) %iterate through table rows
                mat = mat*self.dh2mat(dhTable(i,:)); %for each row, create mat transform for the DH parameters and multiply new matrix to previous ones
                self.CurrHTMatricies(:,:,i) = mat; %store intermediate frame for reference
            end
        end

        % This method takes n joint configurations as inputs in the form of an nx1 vector. Returns a 4x4
        % homogeneous transformation matrix of the position and orientation of the
        % EE frame w.r.t. to the base frame
        function mat = fk3001(self, JointVariables)
            dhTable = zeros(4,4);
            dhTable(1,:) = [0,55,0,0];% t(0->1)
            dhTable(2,:) = [JointVariables(1,1),40,0,-90];% t(1->2)
            dhTable(3,:) = [JointVariables(2,1)-90,0,100,0];% t(2 -> 3)
            dhTable(4,:) = [90+JointVariables(3,1),0,100,0];% t(3-> End Effector)
            mat = self.dh2fk(dhTable); %0->End Effector
        end

        %This method takes 3 joint angles as inputs in the form of an nx1
        %vector and Returns the HT matrix from base to tip
        %Note: it does not update the CurrHTMatrcies
        function mat = fk3001Fast(~, JointVariables)
            mat = [cos((pi*JointVariables(1,1))/180)*cos((pi*(JointVariables(2,1)-90))/180)*cos((pi*(JointVariables(3,1)+90))/180)-cos((pi*JointVariables(1,1))/180)*sin((pi*(JointVariables(2,1)-90))/180)*sin((pi*(JointVariables(3,1)+90))/180),-cos((pi*JointVariables(1,1))/180)*cos((pi*(JointVariables(2,1)-90))/180)*sin((pi*(JointVariables(3,1)+90))/180)-cos((pi*JointVariables(1,1))/180)*cos((pi*(JointVariables(3,1)+90))/180)*sin((pi*(JointVariables(2,1)-90))/180),-sin((pi*JointVariables(1,1))/180),100*cos((pi*JointVariables(1,1))/180)*cos((pi*(JointVariables(2,1)-90))/180)+100*cos((pi*JointVariables(1,1))/180)*cos((pi*(JointVariables(2,1)-90))/180)*cos((pi*(JointVariables(3,1)+90))/180)-100*cos((pi*JointVariables(1,1))/180)*sin((pi*(JointVariables(2,1)-90))/180)*sin((pi*(JointVariables(3,1)+90))/180);
                sin((pi*JointVariables(1,1))/180)*cos((pi*(JointVariables(2,1)-90))/180)*cos((pi*(JointVariables(3,1)+90))/180)-sin((pi*JointVariables(1,1))/180)*sin((pi*(JointVariables(2,1)-90))/180)*sin((pi*(JointVariables(3,1)+90))/180),-sin((pi*JointVariables(1,1))/180)*cos((pi*(JointVariables(2,1)-90))/180)*sin((pi*(JointVariables(3,1)+90))/180)-sin((pi*JointVariables(1,1))/180)*cos((pi*(JointVariables(3,1)+90))/180)*sin((pi*(JointVariables(2,1)-90))/180),cos((pi*JointVariables(1,1))/180),100*sin((pi*JointVariables(1,1))/180)*cos((pi*(JointVariables(2,1)-90))/180)+100*sin((pi*JointVariables(1,1))/180)*cos((pi*(JointVariables(2,1)-90))/180)*cos((pi*(JointVariables(3,1)+90))/180)-100*sin((pi*JointVariables(1,1))/180)*sin((pi*(JointVariables(2,1)-90))/180)*sin((pi*(JointVariables(3,1)+90))/180);
                -cos((pi*(JointVariables(2,1)-90))/180)*sin((pi*(JointVariables(3,1)+90))/180)-cos((pi*(JointVariables(3,1)+90))/180)*sin((pi*(JointVariables(2,1)-90))/180),sin((pi*(JointVariables(2,1)-90))/180)*sin((pi*(JointVariables(3,1)+90))/180)-cos((pi*(JointVariables(2,1)-90))/180)*cos((pi*(JointVariables(3,1)+90))/180),0,95-100*cos((pi*(JointVariables(2,1)-90))/180)*sin((pi*(JointVariables(3,1)+90))/180)-100*cos((pi*(JointVariables(3,1)+90))/180)*sin((pi*(JointVariables(2,1)-90))/180)-100*sin((pi*(JointVariables(2,1)-90))/180);
                0,0,0,1];
            %             self.CurrHTMatricies(:,:,4) = mat;
            %Commented out for speed but the following code calculates the
            %intermediate transformation matricies from base to joint
            %             if(CalculateIntermediateMatricies)
            %                 self.CurrHTMatricies(:,:,2) = [cos((pi*JointVariables(1,1))/180),  0, -sin((pi*JointVariables(1,1))/180),  0;
            %                                                 sin((pi*JointVariables(1,1))/180),  0,  cos((pi*JointVariables(1,1))/180),  0;
            %                                                 0, -1,                     0, 95;
            %                                                 0,  0,                     0,  1];
            %                 self.CurrHTMatricies(:,:,3) = [cos((pi*JointVariables(1,1))/180)*cos((pi*(JointVariables(2,1) - 90))/180), -cos((pi*JointVariables(1,1))/180)*sin((pi*(JointVariables(2,1) - 90))/180), -sin((pi*JointVariables(1,1))/180), 100*cos((pi*JointVariables(1,1))/180)*cos((pi*(JointVariables(2,1) - 90))/180);
            %                                                 sin((pi*JointVariables(1,1))/180)*cos((pi*(JointVariables(2,1) - 90))/180), -sin((pi*JointVariables(1,1))/180)*sin((pi*(JointVariables(2,1) - 90))/180),  cos((pi*JointVariables(1,1))/180), 100*sin((pi*JointVariables(1,1))/180)*cos((pi*(JointVariables(2,1) - 90))/180);
            %                                                  -sin((pi*(JointVariables(2,1) - 90))/180),                      -cos((pi*(JointVariables(2,1) - 90))/180),                     0,                 95 - 100*sin((pi*(JointVariables(2,1) - 90))/180);
            %                                                0,                                                 0,                     0,                                                    1];
            %
            %             end
        end

        %Takes in joint angles and returns end effector position in nx1
        %matrix
        function mat = PositionFK3001(~,JointVariables)
            mat = [100*cos((pi*JointVariables(1,1))/180)*cos((pi*(JointVariables(2,1)-90))/180)+100*cos((pi*JointVariables(1,1))/180)*cos((pi*(JointVariables(2,1)-90))/180)*cos((pi*(JointVariables(3,1)+90))/180)-100*cos((pi*JointVariables(1,1))/180)*sin((pi*(JointVariables(2,1)-90))/180)*sin((pi*(JointVariables(3,1)+90))/180);
                100*sin((pi*JointVariables(1,1))/180)*cos((pi*(JointVariables(2,1)-90))/180)+100*sin((pi*JointVariables(1,1))/180)*cos((pi*(JointVariables(2,1)-90))/180)*cos((pi*(JointVariables(3,1)+90))/180)-100*sin((pi*JointVariables(1,1))/180)*sin((pi*(JointVariables(2,1)-90))/180)*sin((pi*(JointVariables(3,1)+90))/180);
                95-100*cos((pi*(JointVariables(2,1)-90))/180)*sin((pi*(JointVariables(3,1)+90))/180)-100*cos((pi*(JointVariables(3,1)+90))/180)*sin((pi*(JointVariables(2,1)-90))/180)-100*sin((pi*(JointVariables(2,1)-90))/180)];
        end

        %Position is a 3x1 vector of positions
        function JointAngles = ik3001(self, Position)
            if self.CheckPointInWorkSpace(Position) %First check the point to see if it is in the valid workspace
                JointAngles = nan(3,1); % Initialize joint angles
                %Calculate r and s
                r = sqrt(power(Position(1,1),2) + power(Position(2,1),2));
                s = Position(3,1) - 95;
                
                %solve for the base angle solutions 
                C = sqrt(1-power(Position(1,1)/r,2));
                Joint1Solution1 = rad2deg(atan2(C,Position(1,1)/r));
                Joint1Solution2= rad2deg(atan2(-C,Position(1,1)/r));

                %Solve for the joint 3 solutions
                D = -((20000 - (power(r,2) + power(s,2)))/(20000));
                C = sqrt(1-power(D,2));
                Joint3Solution1 = rad2deg(atan2(C,D))-90;
                Joint3Solution2 = rad2deg(atan2(-C,D))-90;

                %Solve for the joint 2 solutions
                D = (power(r,2) + power(s,2))/(200*sqrt(power(r,2) + power(s,2)));
                C = sqrt(1-D^2);
                beta1 = atan2(C,D);
                beta2 = atan2(-C,D);
                alpha1 = atan2(r,s);

                Joint2Solution1 = rad2deg(alpha1 - beta1);
                Joint2Solution2 = rad2deg(alpha1 - beta2);

                %Check for a valid solution that doesn't exceed joint
                %angles

                if(Joint2Solution1 <96.4 && Joint2Solution1 > -55)
                    if(Joint1Solution1<180 && Joint1Solution1>-104)
                        if(Joint3Solution1 > -100 && Joint3Solution1 < 77)
                            if(self.CheckFK([Joint1Solution1;Joint2Solution1;Joint3Solution1],Position))
                                JointAngles = [Joint1Solution1;Joint2Solution1;Joint3Solution1];
                            
                            end
                        end

                        if(Joint3Solution2 > -100 && Joint3Solution2 < 77)
                            if(self.CheckFK([Joint1Solution1;Joint2Solution1;Joint3Solution2],Position))
                                JointAngles = [Joint1Solution1;Joint2Solution1;Joint3Solution2];
                            end
                        end
                    end

                    if(Joint1Solution2<180 && Joint1Solution2>-104)
                        if(Joint3Solution1 > -100 && Joint3Solution1 < 77)
                            if(self.CheckFK([Joint1Solution2;Joint2Solution1;Joint3Solution1],Position))
                                JointAngles = [Joint1Solution2;Joint2Solution1;Joint3Solution1];
                            end
                        end
                        if(Joint3Solution2 > -100 && Joint3Solution2 < 77)
                            if(self.CheckFK([Joint1Solution2;Joint2Solution1;Joint3Solution2],Position))
                                JointAngles = [Joint1Solution2;Joint2Solution1;Joint3Solution2];
                            end
                        end
                    end
                end

                if(Joint2Solution2 <96.4 && Joint2Solution2 > -55)
                    if(Joint1Solution1<180 && Joint1Solution1>-104)
                        if(Joint3Solution1 > -100 && Joint3Solution1 < 77)
                            if(self.CheckFK([Joint1Solution1;Joint2Solution2;Joint3Solution1],Position))
                                JointAngles = [Joint1Solution1;Joint2Solution2;Joint3Solution1];
                            end
                        end
                        if(Joint3Solution2 > -100 && Joint3Solution2 < 77)
                            if(self.CheckFK([Joint1Solution1;Joint2Solution2;Joint3Solution2],Position))
                                JointAngles = [Joint1Solution1;Joint2Solution2;Joint3Solution2];
                            end
                        end
                    end

                    if(Joint1Solution2<180 && Joint1Solution2>-104)
                        if(Joint3Solution1 > -100 && Joint3Solution1 < 77)
                            if(self.CheckFK([Joint1Solution2;Joint2Solution2;Joint3Solution1],Position))
                                JointAngles = [Joint1Solution2;Joint2Solution2;Joint3Solution1];
                            end
                        end
                        if(Joint3Solution2 > -100 && Joint3Solution2 < 77)
                            if(self.CheckFK([Joint1Solution2;Joint2Solution2;Joint3Solution2],Position))
                                JointAngles = [Joint1Solution2;Joint2Solution2;Joint3Solution2];
                            end
                        end
                    end
                   
                end

                %if a valid solution isn't found throw an error
                if isnan(JointAngles(1,1))
                    disp(Position)
                    error('No Valid Solution found')
                else 
                    j = self.jacob3001Fast(JointAngles);
                    self.detectSingularity(j(1:3,:));
                end
            else %Throw an error if the point is not in the workspace
                disp(Position)
                error('Point not in workspace')
            end
        end

        %Takes a set of JointAngles (3x1) and Position (3x1) and returns 1
        %if the FK of the JointAngles is equal to Position within 4 mm of
        %error
        %else return 1 
        function bool = CheckFK(self,JointAngles,Position)
            ErrorVector = self.PositionFK3001(JointAngles) - Position;
            if sqrt(ErrorVector(1,1)^2 + ErrorVector(2,1)^2 + ErrorVector(3,1)^2) < 4
                bool = 1;
            else
                bool = 0;
            end
        end
        %Takes in a nx1 point and checks to see if it is within the robot's
        %workspace
        function bool = CheckPointInWorkSpace(~,Point)
            if(sqrt(power(Point(1,1),2) + power(Point(2,1),2) + power(Point(3,1)-95,2)) <= 200) %See if the point is within the sphere defined by the two segments of the robot arm while straight
                bool = 1;
            else
                bool = 0;
            end
        end

        %Takes in 3x1 vector of joint angles, returns 6x3 jacobian matrix
        %for velocity kinematics
        function jacobMat = jacob3001(self, jointAngles)
            HT_ee = self.fk3001(jointAngles);
            p_ee = HT_ee(1:3,4);
            jacobMat = zeros(6,3);

            for i = 1:3
                p_0_to_i = self.CurrHTMatricies(1:3,4,i);
                z_i = self.CurrHTMatricies(1:3,3,i);
                jacobMat(1:3,i) = cross(z_i,p_ee - p_0_to_i);
                jacobMat(4:6,i) = z_i;
            end
            jacobMat = jacobMat/57.2958;
        end
        
        %Takes in an nx1 matrix and returns a 6x3 jacobian
        function jacobMat = jacob3001Fast(~,JointAngles)
            jacobMat = [(5*pi*sin((pi*JointAngles(1,1))/180)*sin((pi*(JointAngles(2,1) - 90))/180)*sin((pi*(JointAngles(3,1) + 90))/180))/9 - (5*pi*sin((pi*JointAngles(1,1))/180)*cos((pi*(JointAngles(2,1) - 90))/180)*cos((pi*(JointAngles(3,1) + 90))/180))/9 - (5*pi*sin((pi*JointAngles(1,1))/180)*cos((pi*(JointAngles(2,1) - 90))/180))/9, - (5*pi*cos((pi*JointAngles(1,1))/180)*sin((pi*(JointAngles(2,1) - 90))/180))/9 - (5*pi*cos((pi*JointAngles(1,1))/180)*cos((pi*(JointAngles(2,1) - 90))/180)*sin((pi*(JointAngles(3,1) + 90))/180))/9 - (5*pi*cos((pi*JointAngles(1,1))/180)*cos((pi*(JointAngles(3,1) + 90))/180)*sin((pi*(JointAngles(2,1) - 90))/180))/9, - (5*pi*cos((pi*JointAngles(1,1))/180)*cos((pi*(JointAngles(2,1) - 90))/180)*sin((pi*(JointAngles(3,1) + 90))/180))/9 - (5*pi*cos((pi*JointAngles(1,1))/180)*cos((pi*(JointAngles(3,1) + 90))/180)*sin((pi*(JointAngles(2,1) - 90))/180))/9;
(5*pi*cos((pi*JointAngles(1,1))/180)*cos((pi*(JointAngles(2,1) - 90))/180))/9 + (5*pi*cos((pi*JointAngles(1,1))/180)*cos((pi*(JointAngles(2,1) - 90))/180)*cos((pi*(JointAngles(3,1) + 90))/180))/9 - (5*pi*cos((pi*JointAngles(1,1))/180)*sin((pi*(JointAngles(2,1) - 90))/180)*sin((pi*(JointAngles(3,1) + 90))/180))/9, - (5*pi*sin((pi*JointAngles(1,1))/180)*sin((pi*(JointAngles(2,1) - 90))/180))/9 - (5*pi*sin((pi*JointAngles(1,1))/180)*cos((pi*(JointAngles(2,1) - 90))/180)*sin((pi*(JointAngles(3,1) + 90))/180))/9 - (5*pi*sin((pi*JointAngles(1,1))/180)*cos((pi*(JointAngles(3,1) + 90))/180)*sin((pi*(JointAngles(2,1) - 90))/180))/9, - (5*pi*sin((pi*JointAngles(1,1))/180)*cos((pi*(JointAngles(2,1) - 90))/180)*sin((pi*(JointAngles(3,1) + 90))/180))/9 - (5*pi*sin((pi*JointAngles(1,1))/180)*cos((pi*(JointAngles(3,1) + 90))/180)*sin((pi*(JointAngles(2,1) - 90))/180))/9;
                                                                                                                                                                                                                                        0,                                                                  (5*pi*sin((pi*(JointAngles(2,1) - 90))/180)*sin((pi*(JointAngles(3,1) + 90))/180))/9 - (5*pi*cos((pi*(JointAngles(2,1) - 90))/180)*cos((pi*(JointAngles(3,1) + 90))/180))/9 - (5*pi*cos((pi*(JointAngles(2,1) - 90))/180))/9,                                             (5*pi*sin((pi*(JointAngles(2,1) - 90))/180)*sin((pi*(JointAngles(3,1) + 90))/180))/9 - (5*pi*cos((pi*(JointAngles(2,1) - 90))/180)*cos((pi*(JointAngles(3,1) + 90))/180))/9;
                                                                                                                                                                                                                                        0,                                                                                                                                                                                                                       -sin((pi*JointAngles(1,1))/180),                                                                                                                                                           -sin((pi*JointAngles(1,1))/180);
                                                                                                                                                                                                                                        0,                                                                                                                                                                                                                        cos((pi*JointAngles(1,1))/180),                                                                                                                                                            cos((pi*JointAngles(1,1))/180);
                                                                                                                                                                                                                                       1,                                                                                                                                                                                                                                           0,                                                                                                                                                                               0];
        end
        


        function fdk = fdk3001(self,JointAngles,JointVelocities)
            %Multiplication of a 6x3 Jacobian by a 3x1 Velocity
%             self.jacob3001Fast(JointAngles)
            fdk = self.jacob3001Fast(JointAngles)*JointVelocities;
        end

        function bool = detectSingularity(self, jp)
            det(jp)
            if(det(jp) < 0.75)
                bool = 1;
                %error('Error: Singularity Detected');
            else
                bool = 0;
            end
        end
    end
end
