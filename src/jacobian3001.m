function J = jacobian3001(q)


theta1 = countsToDegrees(q(1));
theta2 = countsToDegrees(q(2));
theta3 = countsToDegrees(q(3));

qDots = [countsToDegrees(q(4)); countsToDegrees(q(5)); countsToDegrees(q(6))];
%disp("qDots: ");
%disp(qDots);

L1 = 135; %mm
L2 = 175;
L3 = 169.28;

%Partial Derivatives
dxtheta1 = -L3*sind(theta1)*sind(theta2)*cosd(theta3) - L3*cosd(theta2)*sind(theta3)*sind(theta1) - L2*cosd(theta2)*sind(theta1);

dxtheta2 = -L2*cosd(theta1)*sind(theta2) + L3*cosd(theta1)*cosd(theta2)*cosd(theta3) - L3*cosd(theta1)*sind(theta3)*sind(theta2);

dxtheta3 = L3*cosd(theta1)*cosd(theta2)*cosd(theta3) - L3*cosd(theta1)*sind(theta3)*sind(theta2);


dytheta1 = -L3*cosd(theta1)*sind(theta2)*cosd(theta3) - L3*cosd(theta1)*cosd(theta2)*sind(theta3) - L2*cosd(theta1)*cosd(theta2);

dytheta2 = L2*sind(theta1)*sind(theta2) - L3*cosd(theta2)*sind(theta1)*cosd(theta3) + L3*sind(theta3)*sind(theta1)*sind(theta2);

dytheta3 = -L3*cosd(theta2)*sind(theta1)*cosd(theta3) + L3*sind(theta3)*sind(theta1)*sind(theta2);


dztheta1 = 0;

dztheta2 = L2*cosd(theta2) + L3*cosd(theta2)*sind(theta3) + L3*sind(theta2)*cosd(theta3);

dztheta3 = L3*cosd(theta2)*sind(theta3) + L3*sind(theta2)*cosd(theta3);


Jv = [dxtheta1, dxtheta2, dxtheta3;
        dytheta1,dytheta2,dytheta3;
        dztheta1,dztheta2,dztheta3];
    
zZero = [0; 0; 1];
zOne = [0;0;1]; 
zTwo = [-sind(theta1);-cosd(theta1);0];

    
Jw = [zZero, zOne, zTwo];
completeJ = [Jv; Jw];
disp(rank( completeJ, 5));

%This will stop the program and stop sending updated positions to the arm.
if( rank( completeJ, 10) < 3 )
    text(50, 50, 'ERROR! Singularity!', 'Color', 'r');
    disp("Theta1: "+theta1+" Theta2: "+theta2+" Theta3: "+theta3);
    disp(completeJ);
    error('Error! Approached Singularity!');
end

% qDots should be diff of degrees
J = completeJ * qDots;
%disp(completeJ);

end
