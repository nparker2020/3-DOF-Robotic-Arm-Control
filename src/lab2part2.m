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
myHIDSimplePacketComs.connect();0

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

%CURRENT HOME POSITION CALIBRATION VALUES:
% 2956.6,2965.86,1997.97


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
  tunePacket(7) = 0.0025;%P
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
        %disp('Return Status');
        %disp(returnStatusPacket);
           
        %degreesMatrix = [countsToDegrees(returnStatusPacket(1)), countsToDegrees(returnStatusPacket(2)),  countsToDegrees(returnStatusPacket(3))-90];

        %testMatrix = fwkin3001(degreesMatrix);
        %disp("testMatrix: " + testMatrix);
        %disp("degrees Matrix: "+ degreesMatrix);

        
        
     degreesMatrix = [countsToDegrees(returnStatusPacket(1)), countsToDegrees(returnStatusPacket(2)),  countsToDegrees(returnStatusPacket(3))];
        
    desiredXs = [206, 175, 170];%221
    desiredYs = [-184, 0.0, 140]; %142.331
    desiredZs = [217, 36, 88];
    
    trianglePt1 = [206, -184, 217];
    trianglePt2 = [175,0,36];
    trianglePt3 = [170,140,88];  
   
    currentEndEffectorPos = fwkin3001(degreesMatrix);
   
%   invMatrix = ikin([desiredXs(currentMove),desiredYs(currentMove),desiredZs(currentMove)]);     
%   adaptedEncoderValues = degreesToCountsJoints(invMatrix(1), invMatrix(2), invMatrix(3));
   zeroMatrix = [0,0,0];
   firstMove = trajectoryGenHelper3D(30, 0, 1, 0, 0, currentEndEffectorPos, trianglePt1, zeroMatrix, zeroMatrix);
   secondMove = trajectoryGenHelper3D(30, 0, 1, 0, 0, trianglePt1, trianglePt2, zeroMatrix, zeroMatrix);
   thirdMove = trajectoryGenHelper3D(30, 0, 1, 0, 0, trianglePt2, trianglePt3, zeroMatrix, zeroMatrix);

   
   singularity = degreesToCountsJoints(5.3999023, -68.610107, 89.280029);
   
   q = [singularity(1),singularity(2),singularity(3), 0, 0, 0];
   
  jMatrix = jacobian3001(q);
   
 tic

 currentMove = firstMove;
 disp("DONE COMPUTING TRAJECTORY......");
 pause(2)
 qXLog = single.empty;
 qYLog = single.empty;
 qZLog = single.empty;
 timeLog = single.empty;
 an = animatedline(double(currentEndEffectorPos(1)), double(currentEndEffectorPos(2)), double(currentEndEffectorPos(3)));
 title('Task Space Position')
 for i = 1 : 3
    
    if( i == 2)
        currentMove = secondMove;
        pause(1);
    elseif ( i == 3 )
        currentMove = thirdMove;
        pause(1);
    end
    
%     timeDelta = currentMove(1, 6);
    timeDelta = .03;
    for c = 1 : 30
        %  disp("Sending status request.");
        timeDelta = splitOver30(c, 1);
        pp.write(STATUS_ID, statusPacket);
        pause(.003 );
        returnStatusPacket = pp.read(STATUS_ID);
       % disp('Return Status');
        disp(returnStatusPacket);
   
        statusPacketLogging(c, 1) = toc;
        statusPacketLogging(c, 2) = returnStatusPacket(1);
        statusPacketLogging(c, 3) = returnStatusPacket(2);
        statusPacketLogging(c, 4) = returnStatusPacket(3);
               
      %  q = currentMove(c, 1) + currentMove(c, 2) * timeDelta + currentMove(c, 3) * timeDelta^2 + currentMove(c, 4) * timeDelta^3;
        
        qX = currentMove(1, 1) + currentMove(2, 1) * timeDelta + currentMove(3, 1) * timeDelta^2 + currentMove(4, 1) * timeDelta^3 + currentMove(5,1) * timeDelta ^4 + currentMove(6,1) * timeDelta ^ 5;
        qY = currentMove(1, 2) + currentMove(2, 2) * timeDelta + currentMove(3, 2) * timeDelta^2 + currentMove(4, 2) * timeDelta^3 + currentMove(5,2) * timeDelta ^4 + currentMove(6,2) * timeDelta ^ 5;
        qZ = currentMove(1, 3) + currentMove(2, 3) * timeDelta + currentMove(3, 3) * timeDelta^2 + currentMove(4, 3) * timeDelta^3 + currentMove(5,3) * timeDelta ^4 + currentMove(6,3) * timeDelta ^ 5;
        qXLog(i, c) = qX;
        qYLog(i, c) = qY;
        qZLog(i, c) = qZ;
        timeLog(c) = toc;
        %         disp("qX: ")
%         disp(qX)
%         disp("qY: ")
%         disp(qY)
%         disp("qZ: ")
%         disp(qZ)
%         
       degreesMatrix = [countsToDegrees(returnStatusPacket(1)), countsToDegrees(returnStatusPacket(2)),  countsToDegrees(returnStatusPacket(3))];
    
       currentEndEffectorPos = fwkin3001(degreesMatrix);
        posX = currentEndEffectorPos(1);
        posY = currentEndEffectorPos(2);
        posZ = currentEndEffectorPos(3);
        endEffectorPosLogging(c, 1) = toc;
        endEffectorPosLogging(c, 2) = posX;
  %endEffectorPosLogging(endEffectorLoggingCount, 3) = posY;
  endEffectorPosLogging(c, 3) = posY;
  endEffectorPosLogging(c, 4) = posZ;
        
%         disp("PosX: "+posX);
%         disp("PosY: "+posY);
%         disp("PosZ: "+posZ);
        testX = posX + qX;
        testY = posY + qY;
        testZ = posZ + qZ;
        
        
       invMatrix = ikin([qX, qY, qZ]);     
        
        adaptedEncoderValues = degreesToCountsJoints(invMatrix(1), invMatrix(2), invMatrix(3));
        
        packet(1) = adaptedEncoderValues(1);
        packet(4) = adaptedEncoderValues(2);
        packet(7) = adaptedEncoderValues(3);
        
        pp.write(SERV_ID, packet); 

        pause(.03);
             
    
    end
    
        currentHip = returnStatusPacket(1);
        currentElbow = returnStatusPacket(2);
        currentWrist = returnStatusPacket(3);
     addpoints(an, double(endEffectorPosLogging(:,2)), double(endEffectorPosLogging(:,3)), double(endEffectorPosLogging(:,4)));
 end
             
   


% plot3(qXLog(1), qYLog(1), qZLog(1));
% hold on
% plot3(qXLog(2), qYLog(2), qZLog(2));
% hold on
% plot3(qXLog(3), qYLog(3), qZLog(3));
% title("Task Space Path");

% xPos = subplot(2,2,1);
% plot(xPos, timeLog, qXLog);
% hold on
% plot(xPos, timeLog, qYLog);
% hold on
% plot(xPos, timeLog, qZLog);
% title('Task Space Positions vs. Time');
% 
% velocity = subplot(2,2,2);
% velX = diff(qXLog);
% plot(velocity, timeLog(1:length(timeLog) - 1), velX);
% hold on
% velY = diff(qYLog);
% plot(velocity, timeLog(1:length(timeLog) - 1), velY);
% hold on
% velZ = diff(qZLog);
% plot(velocity, timeLog(1:length(timeLog) - 1), velZ);
% title("Velocity vs. Time");
% 
% accel = subplot(2,2,3);
% plot(accel, timeLog(1:length(timeLog) - 2), diff(velX));
% hold on
% plot(accel, timeLog(1:length(timeLog) - 2), diff(velY));
% hold on
% plot(accel, timeLog(1:length(timeLog) - 2), diff(velZ));
% title('Acceleration vs. Time');

%  xPos = subplot(2,2,1);
%  plot(xPos, endEffectorPosLogging(:,1), endEffectorPosLogging(:,2))
%  hold on
%  plot(xPos, endEffectorPosLogging(:,3))
%  hold on
%  plot(xPos, endEffectorPosLogging(:,4))
%  title('End Effector Position');
% 
%  jointsPlot = subplot(2,2,2);
%  plot(jointsPlot, statusPacketLogging(:,2));
%  hold on
%  plot(jointsPlot, statusPacketLogging(:,3));
%  hold on
%  plot(jointsPlot, statusPacketLogging(:,4));
%  title('Joint Position');
%  
 
   pp.shutdown()      
%         
% #4.)s


function S = splitOver30(index, value)
    tempValue = value/30;
    S = tempValue * index;
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


% function T = fwkin3001(q)
% 
%     theta1 = q(1);
%     theta2 = q(2);
%     theta3 = q(3);
%     L1 = 135; %mm
%     L2 = 175;
%     L3 = 169.28;
%     
%     zeroToOne = tdh(-theta1, 0, 0, 0);
%     oneToTwo = tdh(0, L1, 0, 90);
%     twoToThree = tdh(theta2, 0, L2, 0);
%     threeToFour = tdh(theta3 - 90, 0, L3, 0);
%     
%     finalMatrix = zeroToOne * oneToTwo * twoToThree * threeToFour;
% 
%     frameTwo = zeroToOne * oneToTwo;
%     position2 = [frameTwo(1,4), frameTwo(2,4), frameTwo(3,4)];
%     frameThree = frameTwo * twoToThree;
%     position3 = [frameThree(1,4), frameThree(2,4), frameThree(3,4)];
%     finalFrame = frameThree * threeToFour;
%     position4 = [finalFrame(1,4), finalFrame(2,4), finalFrame(3,4)];
%     
%     
% %     plot3([zeroToOne(1,4),position2(1), position3(1), position4(1)], [zeroToOne(2,4),position2(2), position3(2), position4(2)], [zeroToOne(3,4),position2(3), position3(3), position4(3)]);
% % axis([0 400 -200 200 0 400])
% 
% %     drawnow;
% T = [finalMatrix(1,4) , finalMatrix(2,4), finalMatrix(3,4)];
% disp(T);
% %disp(zeroToOne);
% %disp(zeroToOne * oneToTwo* twoToThree*threeToFour );
% 
% end

