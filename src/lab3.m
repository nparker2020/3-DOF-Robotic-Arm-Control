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
myHIDSimplePacketComs.connect();0

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
tunePacket(4) = .0125;%P
tunePacket(5) = 0.0001;%I
tunePacket(6) = .015;%D %.15

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

L1 = 135; %mm
L2 = 175;
L3 = 169.28;
%used for testing sending to singularity!
%trianglePt1 = [0, 0, L1+L2+L3];
trianglePt1 = [206, -184, 217];
trianglePt2 = [175,0,36];
trianglePt3 = [170,140,88];

%Convert joint angles from status to degrees
degreesMatrix = [countsToDegrees(returnStatusPacket(1)), countsToDegrees(returnStatusPacket(2)),  countsToDegrees(returnStatusPacket(3))];
currentEndEffectorPos = fwkin3001(degreesMatrix);
zeroMatrix = [0,0,0];

firstMove = trajectoryGenHelper3D(30, 0, 1, 0, 0, currentEndEffectorPos, trianglePt1, zeroMatrix, zeroMatrix);
secondMove = trajectoryGenHelper3D(30, 0, 1, 0, 0, trianglePt1, trianglePt2, zeroMatrix, zeroMatrix);
thirdMove = trajectoryGenHelper3D(30, 0, 1, 0, 0, trianglePt2, trianglePt3, zeroMatrix, zeroMatrix);

currentMove = firstMove;
disp("DONE COMPUTING TRAJECTORY......");

hipEncoderLog = single.empty;
elbowEncoderLog = single.empty;
wristEncoderLog = single.empty;
hipVelLog = single.empty;
elbowVelLog = single.empty;
wristVelLog = single.empty;
packet = zeros(15, 1, 'single');

position = zeros(3, 1, 'single');
xDotLog = single.empty;
timeDelta = .03;
currentMove = firstMove;
diffVelLog = single.empty;

%send to three pre-configured positions.
tic
for i = 1 : 3
    
    if( i == 2)
        currentMove = secondMove;
        pause(.003);
    elseif ( i == 3 )
        currentMove = thirdMove;
        pause(.003);
    end
    
    for c = 1 : 30
        timeDelta = splitOver30(c, 1);
        
        pp.write(STATUS_ID, statusPacket);
        pause(.003 );
        returnStatusPacket = pp.read(STATUS_ID);
        
        hipEncoderLog(c) = returnStatusPacket(1);
        elbowEncoderLog(c) = returnStatusPacket(2);
        wristEncoderLog(c) = returnStatusPacket(3);
        
        degreesMatrix = [countsToDegrees(returnStatusPacket(1)), countsToDegrees(returnStatusPacket(2)),  countsToDegrees(returnStatusPacket(3))];
        currentEndEffectorPos = fwkin3001(degreesMatrix);
        
        qX = currentMove(1, 1) + currentMove(2, 1) * timeDelta + currentMove(3, 1) * timeDelta^2 + currentMove(4, 1) * timeDelta^3 + currentMove(5,1) * timeDelta ^4 + currentMove(6,1) * timeDelta ^ 5;
        qY = currentMove(1, 2) + currentMove(2, 2) * timeDelta + currentMove(3, 2) * timeDelta^2 + currentMove(4, 2) * timeDelta^3 + currentMove(5,2) * timeDelta ^4 + currentMove(6,2) * timeDelta ^ 5;
        qZ = currentMove(1, 3) + currentMove(2, 3) * timeDelta + currentMove(3, 3) * timeDelta^2 + currentMove(4, 3) * timeDelta^3 + currentMove(5,3) * timeDelta ^4 + currentMove(6,3) * timeDelta ^ 5;
        
        invMatrix = ikin([qX, qY, qZ]);
        
        adaptedEncoderValues = degreesToCountsJoints(invMatrix(1), invMatrix(2), invMatrix(3));
        
        packet(1) = adaptedEncoderValues(1);
        packet(4) = adaptedEncoderValues(2);
        packet(7) = adaptedEncoderValues(3);
        
        pp.write(SERV_ID, packet);
        
        previous = c;
        if(c > 1 )
            previous = c - 1;
            %disp(diff(hipEncoderLog(previous:c)));
            qVector = [returnStatusPacket(1), returnStatusPacket(2), returnStatusPacket(3), diff(hipEncoderLog(previous:c)), diff(elbowEncoderLog(previous:c)) diff(wristEncoderLog(previous:c))];
            
            xDot = jacobian3001(qVector);
            %xDotLog(c) = xDot;
            %disp("xDot1: "+xDot(1) + " xDot2: "+xDot(2) + " xDot3: "+xDot(3));
            plotVector = [degreesMatrix(1), degreesMatrix(2), degreesMatrix(3), xDot(1), xDot(2), xDot(3)];
            plotFkinAndVel(plotVector);
        end
        
        pause(.03);
        
    end
    
    
end


pp.shutdown()
