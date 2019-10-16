%%
function T = baseToCheckerLandmark()


    L1 = 135; %mm
    L2 = 175;
    L3 = 169.28;
    
    zeroToOne = tdh(0, 0, L2+91, -90);
    oneToTwo = tdh(180, 101.6, 0, 90);

    
    finalMatrix = zeroToOne * oneToTwo;
    
T = finalMatrix;


end
