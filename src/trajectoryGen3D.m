

function T = trajectoryGen3D(tZero, tFinal, omegaZero, omegaFinal, startPosition, endPosition, startAcceleration, finishAcceleration)

% disp("tZero: "+ tZero);
% disp("tFinal: "+tFinal);
% disp("startPosition: "+startPosition);
% disp("endPosition: "+endPosition);

syms aZero aOne aTwo aThree aFour aFive

qZero = aZero + aOne * tZero + aTwo * tZero^2 + aThree * tZero^3 + aFour * tZero^4 + aFive * tZero^5 == startPosition;
qFinal = aZero + aOne * tFinal + aTwo * tFinal^2 + aThree * tFinal^3 + aFour * tFinal^4 + aFive * tFinal^5 == endPosition;
vZero = aOne + 2*aTwo*tZero + 3*aThree*tZero^2 + 4*aFour*tZero^3 + 5*aFive*tZero^4 == omegaZero;
vFinal = aOne + 2*aTwo*tFinal + 3*aThree*tFinal^2 + 4*aFour*tFinal^3 + 5*aFive*tFinal^4 == omegaFinal;
alphaZero = 2 * aTwo + 6*aThree*tZero == startAcceleration;
alphaFinal = 2 * aTwo + 6 * aThree * tFinal == finishAcceleration;

[A,B] = equationsToMatrix([qZero, qFinal, vZero, vFinal, alphaZero, alphaFinal], [aZero, aOne, aTwo, aThree, aFour, aFive]);
T = double(linsolve(A, B));


% disp("LINSOLVE RESULT: ");
% disp(T)

end

