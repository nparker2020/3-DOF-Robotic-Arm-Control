function T = getSizeOfDisk(cameraParams, R, t, xCoord, yCoord, radius) 
    mmTopOfDisk = pointsToWorld(cameraParams, R, t, [xCoord, (yCoord - radius)]);
    mmBottomOfDisk = pointsToWorld(cameraParams, R, t, [xCoord, (yCoord + radius)]);
   
    difference = sqrt( (mmBottomOfDisk(2) - mmTopOfDisk(2))^2 + (mmTopOfDisk(1) - mmBottomOfDisk(1))^2);
    
    if(difference < 59)
        
        T = 0; %small
    
    elseif( difference < 70 )
        T = 1; %medium
    else
        T = 2; %large
    end
    
    disp("Difference: "+ difference + "size: " + T);
end