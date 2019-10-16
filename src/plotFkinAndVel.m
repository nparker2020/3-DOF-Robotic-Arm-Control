function T = plotFkinAndVel(q)

    theta1 = q(1);
    theta2 = q(2);
    theta3 = q(3);
    L1 = 135; %mm
    L2 = 175;
    L3 = 169.28;
    
    xDotOne = q(4);
    xDotTwo = q(5);
    xDotThree = q(6);
    
    zeroToOne = tdh(-theta1, 0, 0, 0);
    oneToTwo = tdh(0, L1, 0, 90);
    twoToThree = tdh(theta2, 0, L2, 0);
    threeToFour = tdh(theta3 - 90, 0, L3, 0);
    
    finalMatrix = zeroToOne * oneToTwo * twoToThree * threeToFour;

    frameTwo = zeroToOne * oneToTwo;
    position2 = [frameTwo(1,4), frameTwo(2,4), frameTwo(3,4)];
    frameThree = frameTwo * twoToThree;
    position3 = [frameThree(1,4), frameThree(2,4), frameThree(3,4)];
    finalFrame = frameThree * threeToFour;
    position4 = [finalFrame(1,4), finalFrame(2,4), finalFrame(3,4)];
    
    
   plot3([zeroToOne(1,4),position2(1), position3(1), position4(1)], [zeroToOne(2,4),position2(2), position3(2), position4(2)], [zeroToOne(3,4),position2(3), position3(3), position4(3)]);
axis([0 400 -200 200 0 400])

    hold on
    quiver3(finalMatrix(1,4) , finalMatrix(2,4), finalMatrix(3,4), xDotOne, xDotTwo, xDotThree);
    hold off
    
end