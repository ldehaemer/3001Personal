classdef Model < handle
    properties
        transform = Kinematics(); % Class object
%         WorkSpace = load("Data/workspace.mat");
    end

    methods
        %Takes in current joint angles (3x1) of arm and generates stick plot
        %representing arm positions in workspace for given set of angles
        function plot = plot_arm(self,JointAngles,VelocityVector, isSingularity)
            
            self.transform.fk3001(JointAngles); %make HT matrices from given angle set
            vector = zeros(3,size(self.transform.CurrHTMatricies,3)); %create vector 3xn (n = # of HTMatrices, 4 in this case for 5 frames)
            for i = 1:size(self.transform.CurrHTMatricies,3) %iterate through # of 4x4 HT Matrices (4 in this case)
                vector(:,i) = self.transform.CurrHTMatricies(1:3,4,i); %for each HT Matrix, store position vector X, Y, Z in rows 1-3 of vector var
                %vector structure: rows are to store position vector X, Y,
                %Z; Columns represent each HT Matrix (note HT matrices are
                %all w.r.t. to base from (i.e t(0 -> 2), t(0 -> 3), etc.)
            end
%             vector(:,4);

            axs=axes; %create a global axis 
            view(axs, 3); %set axs to be a 3D axis
            hold(axs, 'on') %holds both reference frame plotting and position vector plotting together

            xlim(axs, [-200, 200]) %set x bounds of axs
            ylim(axs, [-200, 200]) %set y bounds of axs
            zlim(axs, [0, 200]) % set z bounds of axs
          
            %plot the reference frames for each joint
            % 'Parent' assigns the global axis to be used
            % 'Scale' sizes length of the frame arrows
            % 'Matrix' takes in the HT Matrix
%             for i = 1:size(self.transform.CurrHTMatricies,3)
%                 triad('Parent',axs,'Scale',20,'LineWidth',2,'Matrix',self.transform.CurrHTMatricies(:,:,i));
%             end

            quiver3(vector(1,4),vector(2,4),vector(3,4),VelocityVector(1,1),VelocityVector(2,1),VelocityVector(3,1),1);
           
            %Plot all position vector coordinates for each HT Matrix and
            %apply open circle at ends of each to indicate joints
            plot3(axs, vector(1,:),vector(2,:),vector(3,:),vector(1,4),vector(2,4),vector(3,4),'o',vector(1,3),vector(2,3),vector(3,3),'o',vector(1,2),vector(2,2),vector(3,2),'o',vector(1,1),vector(2,1),vector(3,1),'o');
            xlabel("x") %x label
            ylabel('y') %y label
            zlabel('z') %z label

            %{
            hold on

            % functions below assist in plotting the workspace of the arm,
            % works together with normal live plot, but commented out due
            % to GPU strain
            
            surf(self.WorkSpace.x1,self.WorkSpace.y1,self.WorkSpace.z1);
            surf(self.WorkSpace.x,self.WorkSpace.y,self.WorkSpace.z);
            alpha .05

            hold off
            %}
            if(isSingularity)
                dim = [.2 .5 .3 .3];
                str = 'SINGULARITY DETECTED';
                annotation('textbox',dim,'String',str,'FitBoxToText','on');
                error('SINGULARITY DETECTED')
            end
            
            drawnow; %used to immediately update the figure for real time plotting
        end
    end
end
