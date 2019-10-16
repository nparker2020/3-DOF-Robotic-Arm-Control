%%
% RBE3001 - Laboratory 3
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

% Create a PacketProcessor object to send data to the nucleo firmware
pp = PacketProcessor(myHIDSimplePacketComs);

SERV_ID = 01;
PIDTUNE_ID = 02;
STATUS_ID = 03;



hipCalibrationValues = [2956.0, 2952.5, 2954.0, 2964.5, 2956.0];%2956.6
elbowCalibrationValues = [2954.5, 2957.25, 2963.5, 2995.75, 2958.3];%2965.86
writeCalibrationValues = [1998.75, 1993.5, 1995.2, 2003.2, 1999.2];%1997.97
statusPacketLogging = single.empty;
endEffectorPosLogging = single.empty;

%CURRENT HOME POSITION CALIBRATION VALUES:
% 2956.6,2965.86,1997.97


statusPacket = zeros(15, 1, 'single');

%PID Tuning Packet
tunePacket = zeros(15, 1, 'single');

%Hip
tunePacket(1) = 0.0025;%P
tunePacket(2) = .0005;%I
tunePacket(3) = .0275;%D

%Elbow
tunePacket(4) = .0025;%P
tunePacket(5) = 0.0001;%I
tunePacket(6) = .0015;%D %.15

%Wrist
tunePacket(7) = 0.0025;%P
tunePacket(8) = 0.000125;%I
tunePacket(9) = 0.0075;%D

pp.write(PIDTUNE_ID, tunePacket);
pause(0.01);
packet = zeros(15, 1, 'single');

disp("Sending status request.");
pp.write(STATUS_ID, statusPacket);
pause(.003 );
returnStatusPacket = pp.read(STATUS_ID);

disp(returnStatusPacket);

%statusPacket(1) = 1;

cam = webcam();
%cameraParams = cameraCalibrationData();
loadedCameraParams = load('cameraParams3rdTry.mat');
calibratedCameraParams = loadedCameraParams.cameraParams3rdtry;
%B = baseToCheckerLandmark();
%T = getCamToCheckerboard(cam, calibratedCameraParams);
%Tinvert = inv(T);
 
loadCheckerboardTransform =load('checkerBoardLandmarkTransformation.mat');
loadedCheckerboardTransform = loadCheckerboardTransform.T;
% 
% baseToCam = B/T;
% disp(baseToCam);
close all;

blueSmall = [110, 210, 100];
yellowSmall = [220, 210, 100];
greenSmall = [150, 210, 100];

blueLarge = [110, -210, 100];
yellowLarge = [220, -210, 100];
greenLarge = [150, -210, 100];

lookingState = 0; %arm is in upper position and we're taking photos;
movingState = 1; %we found an object and we're moving towards it;
grippingState = 2; %we are closing the gripper
placingState = 3; %we are moving the object to the desired location;
releasingState = 4; %we are releasing the object;
returningState = 5; %we are returning to the upper position to return to the looking state
computingState = 6;
dropState = 7;
currentState = returningState;
postComputingState = movingState;
postMoveState = lookingState; %maybe?
dropOffLocation = blueSmall;

degreesMatrix = [countsToDegrees(returnStatusPacket(1)), countsToDegrees(returnStatusPacket(2)),  countsToDegrees(returnStatusPacket(3))];
currentEndEffectorPos = fwkin3001(degreesMatrix);

desiredPosition = [250, 0, 211];
%resultingAngles = ikin(desiredPosition);
%adaptedJointvalues = degreesToCountsJoints(resultingAngles);

startPositionMatrix = [currentEndEffectorPos(1), currentEndEffectorPos(2), currentEndEffectorPos(3)];
endPositionMatrix = [250, 0, 211];

currentMove = single.empty;
moveTimeGlobal = 1;%3
timeDelta = 0;
globalCounter = 0;
currentDiskSize = 0;

%
%*********************STATE MACHINE**********************
%
%
newImage = snapshot(cam);

figure(1);
imshow(newImage);

figure(2);
imshow(newImage);

%raw image
figure(3);
imshow(newImage);

%undistorted
figure(4);
imshow(newImage);

%blue mask
figure(5);
imshow(newImage);

%yellow mask
figure(6);
imshow(newImage);

%blackMask
figure(8);
imshow(newImage);

[TR, TT] = TMatrixtoRT(loadedCheckerboardTransform);

n = 2;
tic
while n > 1
      
    switch currentState
       case lookingState
           disp("Looking...");
           pp.write(STATUS_ID, statusPacket);
            pause(.003 );
            returnStatusPacket = pp.read(STATUS_ID);
   
            %make sure gripper is open
           
           statusPacket(1) = 2;
           image = snapshot(cam);
            [undistortedImage,newOrigin] = undistortImage(image,calibratedCameraParams);

          [blueCenters, blueRadii, blueImage] = getCircles(undistortedImage, @blue_mask_2, 15, 30);
           [blackCenters, blackRadii, blackImage] = getCircles(undistortedImage, @blackMask, 35, 70);
           [yellowCenters, yellowRadii, yellowImage] = getCircles(undistortedImage, @yellowMask2, 15, 30);
           [greenCenters, greenRadii, greenImage] = getCircles(undistortedImage, @greenMask4, 15, 30);
            figure(2);
            imshow(greenImage);
            viscircles(greenCenters, greenRadii,'EdgeColor','g');
           
            newImage = undistortedImage;
 
            newImage = insertCenters(newImage, blueCenters);
            newImage = insertCenters(newImage, blackCenters);
            newImage = insertCenters(newImage, yellowCenters);
            newImage = insertCenters(newImage, greenCenters);
        
            %raw image
            figure(3);
            title("Raw Image");
            imshow(image);

            %undistorted
            figure(4);
            title("undistorted");
            imshow(undistortedImage);

            %blue mask
            figure(5);
            imshow(blueImage);
            viscircles(blueCenters, blueRadii,'EdgeColor','b');
            
            %yellow mask
            figure(6);
            imshow(yellowImage);
            viscircles(yellowCenters, yellowRadii,'EdgeColor','y');

            %blackMask
            figure(8);
            imshow(blackImage);
            viscircles(blackCenters, blackRadii,'EdgeColor','b');
            
        %turn this into a function to prioritize disk size??
        maxDiskSize = 0;
        for i = 1 : size(blackCenters)
            xCoord = blackCenters(i, 1);
            yCoord = blackCenters(i, 2);
            radius = blackRadii(i);
            %size = getSizeOfDisk(yCoord, radius);
            diskSize = getSizeOfDisk(calibratedCameraParams, TR, TT, xCoord, yCoord, radius);
            if(diskSize > maxDiskSize) 
                maxDiskSize = diskSize;
            end
            
            %disp("("+xCoord+", "+yCoord+") -- size: "+diskSize);    
        end
        
        [pieceX, pieceY, color] = getPrioritizedColorPos(blueCenters, greenCenters, yellowCenters);
        
        mmFromCheckerBoardLM = pointsToWorld(calibratedCameraParams, TR, TT,[pieceX, pieceY]);
        pieceWorldPoints = checkerToBase(mmFromCheckerBoardLM(1),mmFromCheckerBoardLM(2));
        
        for i = 1 : size(blackCenters)
           blackX = blackCenters(i, 1);
           blackY = blackCenters(i, 2);
           
           mmFromCheckerBoardLM = pointsToWorld(calibratedCameraParams, TR, TT,[blackX, blackY]);
           diskWorldPoints = checkerToBase(mmFromCheckerBoardLM(1),mmFromCheckerBoardLM(2));
           xDiff = abs(diskWorldPoints(1) - pieceWorldPoints(1));
           yDiff = abs(diskWorldPoints(2) - pieceWorldPoints(2));
           
           if( xDiff <= 22 && yDiff <= 22 )
               
               %set global disk size to this so that we know later!
               currentDiskSize = getSizeOfDisk(calibratedCameraParams, TR, TT, blackX, blackY, blackRadii(i));
               disp("Going to Location: ("+pieceWorldPoints(1)+", "+pieceWorldPoints(2)+", -20)");
               endPositionMatrix = [pieceWorldPoints(1), pieceWorldPoints(2), 20];
               disp("endPositionMatrix: "+endPositionMatrix);
               switch(color)
                    case 1
                        if(currentDiskSize == 0)
                            dropOffLocation = blueSmall;
                        else
                            dropOffLocation = blueLarge;
                        end
                        
                    case 2
                        if(currentDiskSize == 0)
                            dropOffLocation = greenSmall;
                        else
                            dropOffLocation = greenLarge;
                        end
                    case 3
                        if(currentDiskSize == 0)
                            dropOffLocation = yellowSmall;
                        else
                            dropOffLocation = yellowLarge;
                        end
                   otherwise
                        disp("No color found!");
                %no color found
                
                end
                currentState = computingState;
                postComputingState = movingState;
                postMoveState = dropState;
           else
                disp("xDiff: "+xDiff+", yDiff: "+yDiff);
           end
            
        end
        if( color == 0)
            currentState = lookingState;
        end
        
           %Figure out how to update the newest image figure in a loop
           %clf(markersFigure);
           %markersFigures = figure, imshow(newImage);
           figure(1);
           imshow(newImage);
            pause(.25);
            newImage = [];
            image = [];
            greenImage = [];
            blueImage = [];
            yellowImage = [];
            blackImage = [];
    case dropState
        disp("Drop State!");
        endPositionMatrix(3) = -30;
        
        currentState = computingState;
        postComputeState = movingState;
        postMoveState = grippingState;
        
    case movingState
        % disp("Moving State.");
         globalCounter = globalCounter + 1;
         timeDelta = splitOver30(globalCounter, moveTimeGlobal);
         %   disp("timeDelta: "+timeDelta);
         qX = currentMove(1, 1) + currentMove(2, 1) * timeDelta + currentMove(3, 1) * timeDelta^2 + currentMove(4, 1) * timeDelta^3 + currentMove(5,1) * timeDelta ^4 + currentMove(6,1) * timeDelta ^ 5;
         qY = currentMove(1, 2) + currentMove(2, 2) * timeDelta + currentMove(3, 2) * timeDelta^2 + currentMove(4, 2) * timeDelta^3 + currentMove(5,2) * timeDelta ^4 + currentMove(6,2) * timeDelta ^ 5;
         qZ = currentMove(1, 3) + currentMove(2, 3) * timeDelta + currentMove(3, 3) * timeDelta^2 + currentMove(4, 3) * timeDelta^3 + currentMove(5,3) * timeDelta ^4 + currentMove(6,3) * timeDelta ^ 5;
         
         invMatrix = ikin([qX, qY, qZ]);
         
         adaptedEncoderValues = degreesToCountsJoints(invMatrix(1), invMatrix(2), invMatrix(3));
         
         packet(1) = adaptedEncoderValues(1);
         packet(4) = adaptedEncoderValues(2);
         packet(7) = adaptedEncoderValues(3);
         
         pp.write(SERV_ID, packet);
         pause(0.03);
         pause(splitOver30(1, moveTimeGlobal));
         
         if( timeDelta >= moveTimeGlobal) 
             currentState = postMoveState;
             globalCounter = 0;
        end
       case grippingState
        disp("gripping state!");
        statusPacket(1) = 1;
        
        pp.write(STATUS_ID, statusPacket);
        pause(.003 );
        returnStatusPacket = pp.read(STATUS_ID);
   
        disp("globalCounter: "+globalCounter);
        %use a counter instead! or send the packet now
        pause(1);
        endPositionMatrix = dropOffLocation;
        
        currentState = computingState;
        postComputeState = movingState;
        postMoveState = releasingState;
       case releasingState
           disp("Releasing State!");
           statusPacket(1) = 2;
           
           pp.write(STATUS_ID, statusPacket);
           pause(.003 );
           returnStatusPacket = pp.read(STATUS_ID);
           disp("globalCounter: "+globalCounter);   
           pause(1);
           
           currentState = returningState;
       case returningState %in the the looking position and call computing! in the future
           disp("Returning State!");
           endPositionMatrix = [250, 0, 211];
           
           currentState = computingState;
           postComputeState = movingState;
           postMoveState = lookingState;
       case computingState
           pp.write(STATUS_ID, statusPacket);
           pause(.005);
           returnStatusPacket = pp.read(STATUS_ID);
   
           %check for zero?
           degreesMatrix = [countsToDegrees(returnStatusPacket(1)), countsToDegrees(returnStatusPacket(2)),  countsToDegrees(returnStatusPacket(3))];
           currentEndEffectorPos = fwkin3001(degreesMatrix);
                
           startPositionMatrix = currentEndEffectorPos;
            
            disp("COMPUTING TRAJECTORY!!!!");
            accelerationVector = [0, 0, 0];
            %        trajectoryGenHelper3D(totalSteps, tZero, totalTfinal, omegaZero, omegaFinal, startPosition, finalPosition, startAcceleration, finishAcceleration )
            currentMove = trajectoryGenHelper3D(30, 0, moveTimeGlobal, 0, 0, startPositionMatrix, endPositionMatrix, accelerationVector, accelerationVector);
       
            currentState = postComputingState;
            disp("DONE COMPUTING.");
            globalCounter = 0;
            tic
        otherwise
           disp("Default case hit! Whoops!");
   end
end

% [camToBaseR,camToBaseT] = TMatrixtoRT(inv(baseToCam));
% [TR, TT] = TMatrixtoRT(loadedCheckerboardTransform);
% 
% image = snapshot(cam);
% 
% mmFromCheckerBoardLM = pointsToWorld(calibratedCameraParams,TR, TT,[488,158]);
% disp(mmFromCheckerBoardLM);
% 
% worldPoints = checkerToBase(mmFromCheckerBoardLM(1),mmFromCheckerBoardLM(2));
% 
%  disp(worldPoints);



 %blueFilterImage = blue_mask_2(undistortedImage);
 %disk_structure = strel('disk', 10);
 
 %dilated_image = imdilate(blueFilterImage, disk_structure);

%  [blueCenters,blueRadii] = imfindcircles(blueFilterImage,[15, 30]);
%  figure, imshow(blueFilterImage);
% viscircles(blueCenters, blueRadii,'EdgeColor','b');
% disp("blueCenters:"+size(blueCenters));  
 

% 
% 
% mmFromCheckerBoardLM = pointsToWorld(calibratedCameraParams, TR, TT,[blueCenters(1,1),blueCenters(1,2)]);
% worldPoints = checkerToBase(mmFromCheckerBoardLM(1),mmFromCheckerBoardLM(2));
% 
% invMatrix = ikin([worldPoints(1), worldPoints(2), -20]);
%  adaptedEncoderValues = degreesToCountsJoints(invMatrix(1), invMatrix(2), invMatrix(3));
        


%         packet(1) = adaptedEncoderValues(1);
%         packet(4) = adaptedEncoderValues(2);
%         packet(7) = adaptedEncoderValues(3);
%         
%         pp.write(SERV_ID, packet);
%           pause(.03);
%  disp("Wrote Packet: "+packet);
%           


 % %send to three pre-configured positions.
tic
% for i = 1 : 3
%     
%     if( i == 2)
%         currentMove = secondMove;
%         pause(.003);
%     elseif ( i == 3 )
%         currentMove = thirdMove;
%         pause(.003);
%     end
%     
%     
%     
%     
%     for c = 1 : 30
%         timeDelta = splitOver30(c, 1);
%         
%         pp.write(STATUS_ID, statusPacket);
%         pause(.003 );
%         returnStatusPacket = pp.read(STATUS_ID);
%         
%         hipEncoderLog(c) = returnStatusPacket(1);
%         elbowEncoderLog(c) = returnStatusPacket(2);
%         wristEncoderLog(c) = returnStatusPacket(3);
%         
%         degreesMatrix = [countsToDegrees(returnStatusPacket(1)), countsToDegrees(returnStatusPacket(2)),  countsToDegrees(returnStatusPacket(3))];
%         currentEndEffectorPos = fwkin3001(degreesMatrix);
%         
%         qX = currentMove(1, 1) + currentMove(2, 1) * timeDelta + currentMove(3, 1) * timeDelta^2 + currentMove(4, 1) * timeDelta^3 + currentMove(5,1) * timeDelta ^4 + currentMove(6,1) * timeDelta ^ 5;
%         qY = currentMove(1, 2) + currentMove(2, 2) * timeDelta + currentMove(3, 2) * timeDelta^2 + currentMove(4, 2) * timeDelta^3 + currentMove(5,2) * timeDelta ^4 + currentMove(6,2) * timeDelta ^ 5;
%         qZ = currentMove(1, 3) + currentMove(2, 3) * timeDelta + currentMove(3, 3) * timeDelta^2 + currentMove(4, 3) * timeDelta^3 + currentMove(5,3) * timeDelta ^4 + currentMove(6,3) * timeDelta ^ 5;
%         
%         invMatrix = ikin([qX, qY, qZ]);
%         
%         adaptedEncoderValues = degreesToCountsJoints(invMatrix(1), invMatrix(2), invMatrix(3));
%         
%         packet(1) = adaptedEncoderValues(1);
%         packet(4) = adaptedEncoderValues(2);
%         packet(7) = adaptedEncoderValues(3);
%         
%         pp.write(SERV_ID, packet);
%         
%         previous = c;
%         if(c > 1 )
%             previous = c - 1;
%             %disp(diff(hipEncoderLog(previous:c)));
%             qVector = [returnStatusPacket(1), returnStatusPacket(2), returnStatusPacket(3), diff(hipEncoderLog(previous:c)), diff(elbowEncoderLog(previous:c)) diff(wristEncoderLog(previous:c))];
%             
%             xDot = jacobian3001(qVector);
%             %xDotLog(c) = xDot;
%             %disp("xDot1: "+xDot(1) + " xDot2: "+xDot(2) + " xDot3: "+xDot(3));
%             plotVector = [degreesMatrix(1), degreesMatrix(2), degreesMatrix(3), xDot(1), xDot(2), xDot(3)];
%             plotFkinAndVel(plotVector);
%         end
%         
%         pause(.03);
%         
%     end
%     
%     
% end


pp.shutdown()
