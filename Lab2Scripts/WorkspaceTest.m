%Tests whether the correct looking workspace is generated for the robotic
%arm

% hold on
% 
% 
% [x,y,z] = sphere;      %# Makes a 21-by-21 point sphere
% r = 200;                 %# A radius value
% x1 = r.*x(11:end,:);       %# Keep top 11 x points
% y1 = r.*y(11:end,:);       %# Keep top 11 y points
% z1 = (r.*z(11:end,:))+95;       %# Keep top 11 z points
% surf(x1,y1,z1);  %# Plot the surface
% alpha .1
% 
% [theta,phi] = meshgrid(linspace(0,2*pi,50));
% r=100;
% R=100;
% x=(R+r*cos(theta)).*cos(phi);
% y=(R+r*cos(theta)).*sin(phi);
% z = nan(50,50);
% for i = 1:50
%     for j =1:50
%         test = r*sin(theta(i,j))+95;
%         if(test<95 && sqrt(power(x(i,j),2)+power(y(i,j),2))>131)
%             z(i,j) = test;
%             
%         end
%     
%     end
% end
% surf(x,y,z);
% alpha .1
% 
% hold on
%             surf(x1,y1,z1);
%             surf(x,y,z);
% alpha .1