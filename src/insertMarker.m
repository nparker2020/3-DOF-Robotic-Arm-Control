function insertMarker(image, centersMatrix) 

    for i = 1: size(centersMatrix)
       
        image = insertMarker(image, [centersMatrix(i, 1) centersMatrix(i, 2)]);
        
    end

end