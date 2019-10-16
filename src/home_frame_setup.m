%% home frame setup
%
% a quick script to grab a screenshot of the board to display the axes
% run this to help you calculate the transformation matrix from the
% checkerboard coordinates to the robot's coordinates.

close all; clear;

cam = webcam;

params;

imOrig = snapshot(cam);
T_cam_to_checker = getCamToCheckerboard(imOrig, cameraParams, worldPoints)