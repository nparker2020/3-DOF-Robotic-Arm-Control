
function M = trajectoryGenHelper3D(totalSteps, tZero, totalTfinal, omegaZero, omegaFinal, startPosition, finalPosition, startAcceleration, finishAcceleration )

moves = single.empty;
initialPosition = startPosition;
timeInitial = tZero;
timeFinal = 0;
%overall encoder tick difference.
positionDifference = finalPosition - startPosition;
totalDiffX = finalPosition(1) - startPosition(1);
totalDiffY = finalPosition(2) - startPosition(2);
totalDiffZ = finalPosition(3) - startPosition(3);


timeDelta = splitOver30(1, totalTfinal);
currentFinalPosition= initialPosition + splitOver30(1, positionDifference);

xCoefficients = trajectoryGen3D(0, totalTfinal, omegaZero, omegaFinal, startPosition(1), finalPosition(1), 0, 0);
yCoefficients = trajectoryGen3D(0, totalTfinal, omegaZero, omegaFinal, startPosition(2), finalPosition(2), startAcceleration(2), finishAcceleration(2));
zCoefficients = trajectoryGen3D(0, totalTfinal, omegaZero, omegaFinal, startPosition(3), finalPosition(3), startAcceleration(3), finishAcceleration(3));

% 
% for i = 1 : totalSteps
%            %timeFinal = splitOver30(i, totalTfinal);
%            qX = xCoefficients(1) + xCoefficients(2) * timeDelta + xCoefficients(3) * timeDelta^2 + xCoefficients(4) * timeDelta^3;
%            qY = yCoefficients(1) + yCoefficients(2) * timeDelta + yCoefficients(3) * timeDelta^2 + yCoefficients(4) * timeDelta^3;
%            qZ = zCoefficients(1) + zCoefficients(2) * timeDelta + zCoefficients(3) * timeDelta^2 + zCoefficients(4) * timeDelta^3;
%            
%            moves(i, 1) = qX;
%            moves(i, 2) = qY;
%            moves(i, 3) = qZ;
%            moves(i, 4) = timeInitial;
%            moves(i, 6) = timeFinal;
%            timeInitial = timeFinal;
% end


 M = [xCoefficients yCoefficients zCoefficients];
end

