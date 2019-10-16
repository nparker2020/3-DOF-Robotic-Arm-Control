function T = degreesToCountsJoints(thetaHip, thetaElbow, thetaWrist)
    hipAdapted =  degreesToCounts(-thetaHip);%angle sign is correct, but encoder is reverse
    elbowAdapted = degreesToCounts(thetaElbow);
    wristAdapted =  degreesToCounts(thetaWrist);
    
    T = [hipAdapted, elbowAdapted, wristAdapted];
end