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


PositonVector = HTMatrix0EE(1:3,4);
Jacobian = sym('a',6);
Jacobian = Jacobian(:,1:3);

%Calculate Jacobian first row take partial derivative of Xc
Jacobian(1,1) = diff(PositonVector(1,1),theta1);
Jacobian(1,2) = diff(PositonVector(1,1),theta2);
Jacobian(1,3) = diff(PositonVector(1,1),theta3);

%Calculate Jacobian second row take partial derivative of Yc
Jacobian(2,1) = diff(PositonVector(2,1),theta1);
Jacobian(2,2) = diff(PositonVector(2,1),theta2);
Jacobian(2,3) = diff(PositonVector(2,1),theta3);

%Calculate Jacobian third row take partial derivative of Zc
Jacobian(3,1) = diff(PositonVector(3,1),theta1);
Jacobian(3,2) = diff(PositonVector(3,1),theta2);
Jacobian(3,3) = diff(PositonVector(3,1),theta3);

%Calculate Jo 
Jacobian(4:6,1) = HTMatrix0to1(1:3,3);

CurrIntermediateHTMatrix = HTMatrix0to1*HTMatrix1to2;
Jacobian(4:6,2) = CurrIntermediateHTMatrix(1:3,3);

CurrIntermediateHTMatrix = HTMatrix0to1*HTMatrix1to2*HTMatrix2to3;
Jacobian(4:6,3) = CurrIntermediateHTMatrix(1:3,3);
Jacobian
