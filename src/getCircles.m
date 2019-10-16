function [centers, radii] = getCircles(undistortedImage, mask, lowerRBound, upperRBound)

    filterImage = mask(undistortedImage);
    [centers, radii] = imfindcircles(filterImage,[lowerRBound, upperRBound]); 
    figure, imshow(filterImage);
    viscircles(centers, radii,'EdgeColor','b');

end