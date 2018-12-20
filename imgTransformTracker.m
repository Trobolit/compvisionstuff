%% Init object to detect
clear all;
close all;


% Create original
deckImg = imread('book.jpg');
deckPoints = detectSURFFeatures(rgb2gray(deckImg));
[deckFeatures, deckPoints] = extractFeatures(rgb2gray(deckImg), deckPoints);

% STart webcam and player
cam = webcam;
vidobj = imaq.VideoDevice();
videoPlayer = vision.VideoPlayer('Position',[100,100,680,520]);
videoPlayer.show

%%

figure(1);
while 1
    % Grab first frame to explore
    objectFrame = ycbcr2rgb(vidobj());

    % Find object
    scenePoints = detectSURFFeatures(rgb2gray(objectFrame),'MetricThreshold',500);
    [sceneFeatures, scenePoints] = extractFeatures(rgb2gray(objectFrame), scenePoints);
    featurePairs = matchFeatures(deckFeatures, sceneFeatures);
    % Show matches
    matchedBoxPoints = deckPoints(featurePairs(:, 1), :);
    matchedScenePoints = scenePoints(featurePairs(:, 2), :);
    
    if matchedScenePoints.Count < 3 
        videoPlayer(objectFrame);
        fprintf('not enough points\n');
        continue;
    end
    
    matchedScenePoints.Count
    
    % Find transformation for object
    try
        [tform, inlierBoxPoints, inlierScenePoints] = ...
            estimateGeometricTransform(matchedBoxPoints, matchedScenePoints, 'affine');
    catch
        videoPlayer(objectFrame);
        fprintf('not enough inliers?\n');
        continue;
    end
    boxPolygon = [1, 1;...                           % top-left
            size(rgb2gray(deckImg), 2), 1;...                 % top-right
            size(rgb2gray(deckImg), 2), size(rgb2gray(deckImg), 1);... % bottom-right
            1, size(rgb2gray(deckImg), 1);...                 % bottom-left
            1, 1]; 
    newBoxPolygon = transformPointsForward(tform, boxPolygon);
    tmp = newBoxPolygon';
    out = insertShape(objectFrame, 'Line',tmp(:)','Color', 'y');
    out = insertMarker(out,matchedScenePoints,'x','color','cyan','size', 6);
    
    videoPlayer(out);
    
    showMatchedFeatures(deckImg, objectFrame, inlierBoxPoints, ...
    inlierScenePoints, 'montage');
    
end


% Release the video player.
release(videoPlayer);



























