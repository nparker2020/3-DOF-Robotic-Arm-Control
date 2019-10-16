%%
% RBE3001 - Laboratory 2
% 

clear
clear java
clear classes;

vid = hex2dec('3742');
pid = hex2dec('0007');

disp (vid);
disp (pid);

javaaddpath ../lib/SimplePacketComsJavaFat-0.6.4.jar;
import edu.wpi.SimplePacketComs.*;
import edu.wpi.SimplePacketComs.device.*;
import edu.wpi.SimplePacketComs.phy.*;
import java.util.*;
import org.hid4java.*;
version -java
myHIDSimplePacketComs=HIDfactory.get();
myHIDSimplePacketComs.setPid(pid);
myHIDSimplePacketComs.setVid(vid);
myHIDSimplePacketComs.connect();
X
% Create a PacketProcessor object to send data to the nucleo firmware
pp = PacketProcessor(myHIDSimplePacketComs); 

% testMatrix = fwkin3001([0, 90, 90]);
SERV_ID = 01;
PIDTUNE_ID = 02;
STATUS_ID = 03;

%      Trial 1
% %   2.9560 %hip
%     2.9545 %elbow
%     1.9988 %wrist
% 
%       Trail 2
% %2.9525
%     2.9573
%     1.9935

%     Trial 3
% %    2.9540
%     2.9635
%     1.9952
% 

%Trail 4
% 2.9645
%     2.9558
%     2.0032

%Trial 5
% 2.9560
%     2.9583
%     1.9992

hipCalibrationValues = [2956.0, 2952.5, 2954.0, 2964.5, 2956.0];%2956.6
elbowCalibrationValues = [2954.5, 2957.25, 2963.5, 2995.75, 2958.3];%2965.86
writeCalibrationValues = [1998.75, 1993.5, 1995.2, 2003.2, 1999.2];%1997.97
statusPacketLogging = single.empty;
endEffectorPosLogging = single.empty;

Pos1 = [10 33 257];
Pos2 = [-279 208 246];
Pos3 = [558 779 74];
Pos4 = [-527 481 -39];
Pos5 = [214 153 -296];

% positionMatrix = [countsToDegrees(degreesToCounts(0) + 2956.6),
%                     countsToDegrees(degreesToCounts(0)+2965.86),
%                     countsToDegrees(degreesToCounts(0)+1997.97)];

%positionMatrix = [0,0,-90];



%plotInThreeDimensions(positionMatrix);
statusPacket = zeros(15, 1, 'single');

tunePacket = zeros(15, 1, 'single');
  
  %Hip
  tunePacket(1) = 0.0025;%P
  tunePacket(2) = .0005;%I
  tunePacket(3) = .0275;%D
  
    %Elbow
  tunePacket(4) = .0125;%P
  tunePacket(5) = 0.0001;%I
  tunePacket(6) = .015;%D %.15
  
    %Wrist
  tunePacket(7) = 0.004;%P
  tunePacket(8) = 0.000125;%I
  tunePacket(9) = 0.0075;%D
%disp(countsToDegrees(degreesToCounts(0) + 2956.6));
%disp(degreesToCounts(0) + 2956.6);
%testMatrix = fwkin3001(positionMatrix);
%disp(testMatrix);

  pp.write(PIDTUNE_ID, tunePacket);
      pause(0.01);
       packet = zeros(15, 1, 'single');
      
%       packet(1) = -279;
%       packet(4) = 208;
%       packet(7) = 246;
%       % Send packet to the server and get the response      
%       %pp.write sends a 15 float packet to the micro controller
%        pp.write(SERV_ID, packet); 
%        pause(0.003); 
%       
disp("Sending status request.");
        pp.write(STATUS_ID, statusPacket);
        pause(.003 );
        returnStatusPacket = pp.read(STATUS_ID);
        disp('Return Status');
        disp(returnStatusPacket);
        
        currentHip = returnStatusPacket(1);
        currentElbow = returnStatusPacket(2);
        currentWrist = returnStatusPacket(3);

        %trajectory plan for first move
        totalTfinal = .802;
        initialPositionElbow = currentElbow;
        targetPositionElbow = 436;
        
        initialPositionWrist = currentWrist;
        targetPositionWrist = -248;
        
        elbowMoves = single.empty;
        
        degreesMatrix = [countsToDegrees(returnStatusPacket(1)), countsToDegrees(returnStatusPacket(2)),  countsToDegrees(returnStatusPacket(3))-90];
% axis([0 200 0 200 0 200])

        testMatrix = fwkin3001(degreesMatrix);
        disp("testMatrix: " + testMatrix);
        disp("degrees Matrix: "+ degreesMatrix);
        
        
        firstElbowMoves = trajectoryGenHelper(30, 0, .802, 0, 0, currentElbow, 436 );
        firstWristMoves = trajectoryGenHelper(30, 0, .802, 0, 0, currentWrist, -248);
        
        secondElbowMoves = trajectoryGenHelper(30, 0, .145, 0, 0, 436, 255);
        secondWristMoves = trajectoryGenHelper(30, 0, .145, 0, 0, -248, 45);
        
        thirdElbowMoves = trajectoryGenHelper(30, 0, .147, 0, 0, 255, 209);
        thirdWristMoves = trajectoryGenHelper(30, 0, .147, 0, 0, 45, -296);
        
        
        
currentElbowMoves = firstElbowMoves;
currentWristMoves = firstWristMoves;
qElbowLog = single.empty;
qWristLog = single.empty;
timeDeltaLog = single.empty;
tic
for i = 1 : 3
    
    if( i == 2)
        currentElbowMoves = secondElbowMoves;
        currentWristMoves = secondWristMoves;
        pause(1);
    elseif ( i == 3 )
        currentElbowMoves = thirdElbowMoves;
        currentWristMoves = thirdWristMoves;
        pause(1);
    end
    
    timeDelta = currentWristMoves(1, 6);
    for c = 1 : 30
       disp("Sending status request.");
        pp.write(STATUS_ID, statusPacket);
        pause(.003 );
        returnStatusPacket = pp.read(STATUS_ID);
        disp('Return Status');
        disp(returnStatusPacket);
   
        statusPacketLogging(c, 1) = toc;
        statusPacketLogging(c, 2) = returnStatusPacket(1);
        statusPacketLogging(c, 3) = returnStatusPacket(2);
        statusPacketLogging(c, 4) = returnStatusPacket(3);
               
        qElbow = currentElbowMoves(c, 1) + currentElbowMoves(c, 2) * timeDelta + currentElbowMoves(c, 3) * timeDelta^2 + currentElbowMoves(c, 4) * timeDelta^3;
        qWrist = currentWristMoves(c, 1) + currentWristMoves(c, 2) * timeDelta + currentWristMoves(c, 3) * timeDelta^2 + currentWristMoves(c, 4) * timeDelta^3;
        qElbowLog(c) = qElbow;
        qWristLog(c) = qWrist;
        
        packet(1) = 0;
        packet(4) = qElbow;
        packet(7) = qWrist;
        
        pp.write(SERV_ID, packet); 
        timeDeltaLog(c,i) = timeDelta;
        pause(timeDelta);
             
        degreesMatrix = [countsToDegrees(returnStatusPacket(1)), countsToDegrees(returnStatusPacket(2)),  countsToDegrees(returnStatusPacket(3))-90];
        % axis([0 200 0 200 0 200])

        testMatrix = fwkin3001(degreesMatrix);
        posX = testMatrix(1, 1);
        posZ = testMatrix(1, 3);
        endEffectorPosLogging(c, 1) = toc;
        endEffectorPosLogging(c, 2) = posX;
  %endEffectorPosLogging(endEffectorLoggingCount, 3) = posY;
        endEffectorPosLogging(c, 3) = posZ;
    end
    
    
%     if(toc > 10)
%        packet(1) = Pos5(1);
%       packet(4) = Pos5(2);
%       packet(7) = Pos5(3);
%          elseif(toc > 8)
%       packet(1) = Pos4(1);
%       packet(4) = Pos4(2);
%       packet(7) = Pos4(3);
%          elseif(toc > 6)
%       packet(1) = Pos3(1);
%       packet(4) = Pos3(2);
%       packet(7) = Pos3(3);
%          elseif(toc > 4)
%       packet(1) = Pos2(1);
%       packet(4) = Pos2(2);
%       packet(7) = Pos2(3);
%     elseif(toc > 2)
%       packet(1) = Pos1(1);pp.write(SERV_ID, packet); 
%        pause(0.003); 
%       packet(4) = Pos1(2);
%       packet(7) = Pos1(3);
%     end

% 
        currentHip = returnStatusPacket(1);
        currentElbow = returnStatusPacket(2);
        currentWrist = returnStatusPacket(3);
%      if(toc > 6)
%         packet(1) = 0;
%        packet(4) = 209;
%        packet(7) = -296;
%           elseif(toc > 4)
%        packet(1) = 0;
%        packet(4) = 255;
%        packet(7) = 45;
%           elseif(toc > 2)
%        
%        packet(1) = 0;
%        packet(4) = 436;
%        packet(7) = -248;
     
     
end
             
   

%     if(toc > 2)
%        pp.write(SERV_ID, packet); 
%        pause(0.003); 
%     end
    
        
     degreesMatrix = [countsToDegrees(returnStatusPacket(1)), countsToDegrees(returnStatusPacket(2)),  countsToDegrees(returnStatusPacket(3))-90];
% axis([0 200 0 200 0 200])

  testMatrix = fwkin3001(degreesMatrix);
  
  posX = testMatrix(1, 1);
  posZ = testMatrix(1, 3);
  endEffectorPosLogging(i, 1) = toc;
  endEffectorPosLogging(i, 2) = posX;
  %endEffectorPosLogging(endEffectorLoggingCount, 3) = posY;
  endEffectorPosLogging(i, 3) = posZ;
  
 
 
%end

xPos = subplot(2,3,1);


answer = trajectoryGen(0, 5, 0, 0, 0, 50);
disp(answer);

plot(xPos, endEffectorPosLogging(:,1), endEffectorPosLogging(:,2))
title('X Position');
zPos = subplot(3,3,2);
plot(zPos, endEffectorPosLogging(:,1), endEffectorPosLogging(:,3))
title('Z Position')

hipPos = subplot(3,3,3);
plot(hipPos, statusPacketLogging(:,1), statusPacketLogging(:,2));
title('Hip Position')

elbowPos = subplot(3,3,4);
plot(elbowPos, statusPacketLogging(:,1), statusPacketLogging(:,3));
title('Elbow Position')

wristPos = subplot(3,3,5);
plot(wristPos, statusPacketLogging(:,1), statusPacketLogging(:,4));
title('Wrist Position')

endEffectorPos = subplot(3,3,6);
plot(endEffectorPos, endEffectorPosLogging(:,3), endEffectorPosLogging(:,2));
title('End Effector Position')

elbowVelocity = subplot(3,3,7);
plot(elbowVelocity, diff(statusPacketLogging(:,3)));
title('Elbow Velocity');

wristVelocity = subplot(3,3,8);
plot(wristVelocity, diff(statusPacketLogging(:,4)));
title('Wrist Velocity');

csvwrite('endEffectorPos.csv', endEffectorPosLogging);

%         
%       disp("Sending status request.");
%         pp.write(STATUS_ID, statusPacket);
%         pause(.003 );
%         returnStatusPacket = pp.read(STATUS_ID);
%         disp('Return Status');
%         disp(returnStatusPacket);
%         pause(.01)  
% 
   pp.shutdown()      
%         
% #4.)
function C = degreesToCounts(theta)

C = (4096/360)*theta;

end
%move = trajectoryGen(0, split30(i, totalTfinal), 0, 0, initialPositionElbow, split30((30 - i), targetPositionElbow)); 
function M = trajectoryGenHelper(totalSteps, tZero, totalTfinal, omegaZero, omegaFinal, startEncoder, finalEncoder )

moves = single.empty;
initialPosition = startEncoder;
timeInitial = tZero;
timeFinal = 0;
%overall encoder tick difference.
encoderDifference = finalEncoder - startEncoder;
timeFinal = splitOver30(1, totalTfinal);
currentFinalEncoder = initialPosition + splitOver30(1, encoderDifference);
for i = 1 : totalSteps
           %timeFinal = splitOver30(i, totalTfinal);
           currentFinalEncoder = startEncoder + splitOver30(i, encoderDifference);
           %      trajectoryGen(tZero, tFinal, omegaZero, omegaFinal, startPosition, endPosition)
           move = trajectoryGen(0, timeFinal, omegaZero, omegaFinal, initialPosition, currentFinalEncoder); 
         %  disp("----------------------------------------------------------------------------------------");
         %  disp(move);
         %  disp("----------------------------------------------------------------------------------------");
            disp("initialPosition: "+initialPosition+", finalPosition: "+currentFinalEncoder);
           
             %now the previous target encoder value
            initialPosition = currentFinalEncoder;
         
           moves(i, 1) = move(1);
           moves(i, 2) = move(2);
           moves(i, 3) = move(3);
           moves(i, 4) = move(4);
           moves(i, 5) = timeInitial;
           moves(i, 6) = timeFinal;
           timeInitial = timeFinal;
end

 M = moves;
end

function S = splitOver30(index, value)
    tempValue = value/30;
    S = tempValue * index;
end

function D = countsToDegrees(counts)

D = (360/4096)*counts;

end

function T = trajectoryGen(tZero, tFinal, omegaZero, omegaFinal, startPosition, endPosition)

syms aZero aOne aTwo aThree

qZero = aZero + aOne * tZero + aTwo * tZero^2 + aThree * tZero^3 == startPosition;
qFinal = aZero + aOne * tFinal + aTwo * tFinal^2 + aThree * tFinal^3 == endPosition;
vZero = aOne + 2*aTwo*tZero + 3*aThree*tZero^2 == omegaZero;
vFinal = aOne + 2*aTwo*tFinal + 3*aThree*tFinal^2 == omegaFinal;

[A,B] = equationsToMatrix([qZero, qFinal, vZero, vFinal], [aZero, aOne, aTwo, aThree]);
T = linsolve(A, B);
disp(T);

end


function T = trotz(theta)
    
templateArray = [cosd(theta),-sind(theta),0,0;
                 sind(theta),cosd(theta),0,0;
                 0,0,1,0;
                 0,0,0,1];

T = templateArray;

end

function T = trotx(alpha)
    
templateArray = [1,0,0,0;disp(zeroToOne);
                 0,cosd(alpha),-sind(alpha),0;
                 0,sind(alpha),cosd(endalpha),0;
                 0,0,0,1];

T = templateArray;

end


function T = troty(beta)
    
templateArray = [cosd(beta),0,sind(beta),0;
                 0,1,0,0;
                 -sind(beta),0,cosd(beta),0;
                 0,0,0,1];

T = templateArray;

end

% #5.)
function result = tdh(theta, d, a, alpha)   
result = [cosd(theta), -sind(theta)*cosd(alpha),  sind(theta)*sind(alpha), a*cosd(theta);   
    sind(theta),  cosd(theta)*cosd(alpha), -cosd(theta)*sind(alpha), a*sind(theta);     
    0,sind(alpha),cosd(alpha), d;          
    0,0,0,1];
end


function T = plotInThreeDimensions(q)

    theta1 = q(1);
    theta2 = q(2);
    theta3 = q(3);
    L1 = 135; %mm
    L2 = 175;
    L3 = 169.28;
    
    zeroToOne = tdh(theta1, 0, 0, 0);
    oneToTwo = tdh(0, L1, 0, 90);
    twoToThree = tdh(theta2, 0, L2, 0);
    threeToFour = tdh(theta3, 0, L3, 0);
    
    frameTwo = zeroToOne * oneToTwo;
    position2 = [frameTwo(1,4), frameTwo(2,4), frameTwo(3,4)];
    frameThree = frameTwo * twoToThree;
    position3 = [frameThree(1,4), frameThree(2,4), frameThree(3,4)];
    finalFrame = frameThree * threeToFour;
    position4 = [finalFrame(1,4), finalFrame(2,4), finalFrame(3,4)];
    
    plot3([position2(1), position3(1), position4(1)], [position2(2), position3(2), position4(2)], [position2(3), position3(3), position4(3)]);
    
    finalMatrix = zeroToOne * oneToTwo * twoToThree * threeToFour;


T = [finalMatrix(1,4) ,finalMatrix(2,4), finalMatrix(3,4)];


end

function T = fwkin3001(q)

    theta1 = q(1);
    theta2 = q(2);
    theta3 = q(3);
    L1 = 135; %mm
    L2 = 175;
    L3 = 169.28;
    
    zeroToOne = tdh(theta1, 0, 0, 0);
    oneToTwo = tdh(0, L1, 0, 90);
    twoToThree = tdh(theta2, 0, L2, 0);
    threeToFour = tdh(theta3, 0, L3, 0);
    
    finalMatrix = zeroToOne * oneToTwo * twoToThree * threeToFour;

    frameTwo = zeroToOne * oneToTwo;
    position2 = [frameTwo(1,4), frameTwo(2,4), frameTwo(3,4)];
    frameThree = frameTwo * twoToThree;
    position3 = [frameThree(1,4), frameThree(2,4), frameThree(3,4)];
    finalFrame = frameThree * threeToFour;
    position4 = [finalFrame(1,4), finalFrame(2,4), finalFrame(3,4)];
    
    
    plot3([zeroToOne(1,4),position2(1), position3(1), position4(1)], [zeroToOne(2,4),position2(2), position3(2), position4(2)], [zeroToOne(3,4),position2(3), position3(3), position4(3)]);
axis([0 400 -200 200 0 400])
%     drawnow;
T = [finalMatrix(1,4) ,finalMatrix(2,4), finalMatrix(3,4)];

%disp(zeroToOne);
disp(zeroToOne * oneToTwo* twoToThree*threeToFour );

end

