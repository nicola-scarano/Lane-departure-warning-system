% SINGLE TRACK MODEL ========== FULL Version ============
% close all; 
clear; clc;
close all;

sceneName = 'curved';
[sceneImage, sceneRef] = helperGetSceneImage(sceneName);
sceneRef.XWorldLimits   % (in meters)
sceneRef.YWorldLimits   % (in meters)
hScene = figure;

helperShowSceneImage(sceneImage, sceneRef)
title(sceneName)
hFig = helperSelectSceneWaypoints(sceneImage, sceneRef);
% Load variables to workspace if they do not exist
if exist('refPoses', 'var')==0 || exist('wayPoints', 'var')==0

    % Load MAT-file containing preselected waypoints
    data = load('waypointsLargeParkingLot');
    data = data.waypointsLargeParkingLot;

    % Assign to caller workspace
    assignin('caller', 'wayPoints', {data.waypoints});
    assignin('caller', 'refPoses', {data.refPoses});
end
numPoses = size(refPoses{1}, 1);

refDirections  = ones(numPoses,1);   % Forward-only motion
numSmoothPoses = 20 * numPoses;      % Increase this to increase the number of returned poses

[smoothRefPoses,~,cumLengths] = smoothPathSpline(refPoses{1}, refDirections, numSmoothPoses);

% Configure the model to stop simulation at 5 seconds.
simStopTime = 5;
% set_param(gcs, 'StopTime', num2str(simStopTime));

% Create a constant velocity profile by generating a time vector
% proportional to the cumulative path length.
timeVector = normalize(cumLengths, 'range', [0, simStopTime]);

% Create variables required by the Simulink model.
refPosesX = [timeVector, smoothRefPoses(:,1)];
refPosesY = [timeVector, smoothRefPoses(:,2)];
refPosesT = [timeVector, smoothRefPoses(:,3)];
