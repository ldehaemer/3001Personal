classdef Traj_planner < handle
    properties
    end
    methods

        %takes a planned start time, end time, start velocity, end
        %velocity, start position, and end position and calcualtes the
        %polynomial coefficents and returns them in a 4x1 array
        function Polynomial_Coefficients = cubic_traj(self,startTime,endTime,startVelocity,endVelocity,startPosition,endPosition)
            B = [startPosition; startVelocity; endPosition; endVelocity]; %matrix of known values

            %Matrix of time constants and derivatives used in trajectory
            %polynomial
            TimeMatrix = [1 startTime startTime^2 startTime^3; 0 1 (2*startTime) (3*startTime^2); 1 endTime endTime^2 endTime^3; 0 1 (2*endTime) (3*endTime^2)];
            Polynomial_Coefficients = inv(TimeMatrix)*B; %Solving for coefficients
        end

         %takes a planned start time, end time, start velocity, end
        %velocity, start acceleration, end acceleration, start position, and end position and calcualtes the
        %polynomial coefficents and returns them in a 6x1 array
        function Polynomial_Coefficients = quintic_traj(self,startTime,endTime,startVelocity,endVelocity,startAcceleration, endAcceleration, startPosition,endPosition)
            %Matrix of time constants and derivatives used in trajectory
            %polynomial
            TimeMatrix = [1 startTime startTime^2 startTime^3 startTime^4 startTime^5;... 

             0 1 (2*startTime) (3*startTime^2) (4*startTime^3) (5*startTime^4);... 
             0 0 2 (6*startTime) (12*startTime^2) (20*startTime^3);...
             1 endTime endTime^2 endTime^3 endTime^4 endTime^5;...
             0 1 (2*endTime) (3*endTime^2) (4*endTime^3) (5*endTime^4);... 
             0 0 2 (6*endTime) (12*endTime^2) (20*endTime^3)];

            %Known values
            Given = [startPosition,startVelocity, startAcceleration, endPosition, endVelocity, endAcceleration]';
            Polynomial_Coefficients = inv(TimeMatrix)*Given; %solve for trajectory coefficients
        end
    end
end