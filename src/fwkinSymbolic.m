function T = fwkinSymbolic()

    syms L1 L2 L3 theta1 theta2 theta3
    
    zeroToOne = tdhsyms(-theta1, 0, 0, 0);
    oneToTwo = tdhsyms(0, L1, 0, pi/2);
    twoToThree = tdhsyms(theta2, 0, L2, 0);
    threeToFour = tdhsyms(theta3 - pi/2, 0, L3, 0);
    
    finalMatrix = zeroToOne * oneToTwo * twoToThree * threeToFour;

    frameTwo = zeroToOne * oneToTwo;
    position2 = [frameTwo(1,4), frameTwo(2,4), frameTwo(3,4)];
    frameThree = frameTwo * twoToThree;
    position3 = [frameThree(1,4), frameThree(2,4), frameThree(3,4)];
    finalFrame = frameThree * threeToFour;
    position4 = [finalFrame(1,4), finalFrame(2,4), finalFrame(3,4)];
    
    
%plot3([zeroToOne(1,4),position2(1), position3(1), position4(1)], [zeroToOne(2,4),position2(2), position3(2), position4(2)], [zeroToOne(3,4),position2(3), position3(3), position4(3)]);
%axis([0 400 -200 200 0 400])

%     drawnow;
T = [finalMatrix(1,4) , finalMatrix(2,4), finalMatrix(3,4)];
disp(T);
%disp(T);
%disp(zeroToOne);
%disp(zeroToOne * oneToTwo* twoToThree*threeToFour );

end
