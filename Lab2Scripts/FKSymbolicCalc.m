syms theta1 theta2 theta3
HTMatrix0to1 = [1,0,0,0;0,1,0,0;0,0,1,55;0,0,0,1];
HTMatrix1to2 = [cosd(theta1),-sind(theta1)*cosd(-90), sind(theta1)*sind(-90), 0*cosd(theta1); 
            sind(theta1),cosd(theta1)*cosd(-90), -cosd(theta1)*sind(-90), 0*sind(theta1);
            0,sind(-90),cosd(-90),40;
            0,0,0,1];
HTMatrix2to3 = [cosd((theta2-90)),-sind((theta2-90))*cosd(0), sind((theta2-90))*sind(0), 100*cosd((theta2-90)); 
            sind((theta2-90)),cosd((theta2-90))*cosd(0), -cosd((theta2-90))*sind(0), 100*sind((theta2-90));
            0,sind(0),cosd(0),0;
            0,0,0,1]; 
HTMatrix3to4EE = [cosd((90+theta3)),-sind((90+theta3))*cosd(0), sind((90+theta3))*sind(0), 100*cosd((90+theta3)); 
            sind((90+theta3)),cosd((90+theta3))*cosd(0), -cosd((90+theta3))*sind(0), 100*sind((90+theta3));
            0,sind(0),cosd(0),0;
            0,0,0,1];
HTMatrix0EE = HTMatrix0to1*HTMatrix1to2*HTMatrix2to3*HTMatrix3to4EE;