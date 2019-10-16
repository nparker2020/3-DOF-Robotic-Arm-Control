function Q = ikin(endEffectorPos)
L1 = 135; %mm
L2 = 175;
L3 = 169.28;



x = endEffectorPos(1);
y = endEffectorPos(2);
z = endEffectorPos(3);

if(y > 300) 
   error('You idiot');
end



r = sqrt(x^2+y^2);
s = z - L1;
beta = atand(s/r);

L4 = sqrt(s^2+r^2);

delta = acosd((L4^2 - L2^2 - L3^2)/(-2*L2*L3));
alpha = acosd((L3^2 - L2^2 - L4^2)/(-2*L2*L4));

theta1 = atan2d(y,x);

theta3 = delta - 90;

theta2 = alpha + beta;


Q = [theta1, theta2, theta3];

end

