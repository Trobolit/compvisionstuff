
% Idea for improvement:
% Use old point tracker to supply a ROI and then find image.
% If points get too few for tracker reinit it with ROI.
% Let the ROI size increase with increased speed, ie relative motion since
% last frame.


clear all;
close all;

pthresh = 200;

% Create original
deckImg = imread('target.jpg');
deckPoints = detectSURFFeatures(rgb2gray(deckImg),'MetricThreshold',pthresh);
[deckFeatures, deckPoints] = extractFeatures(rgb2gray(deckImg), deckPoints );

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
    scenePoints = detectSURFFeatures(rgb2gray(objectFrame),'MetricThreshold',pthresh );
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
    out = insertShape(objectFrame, 'Line',tmp(:)','Color', 'r');
    out = insertMarker(out,matchedScenePoints,'x','color','magenta','size', 6);
    videoPlayer(out);
    showMatchedFeatures(deckImg, objectFrame, inlierBoxPoints, ...
    inlierScenePoints, 'montage');
    
    
    if matchedScenePoints.Count < 6 % Try suppling a ROI for more matches
        fprintf('trying to improve...\n');
        
        roix = max(1,trimmean(newBoxPolygon(:,1),20));
        roiy = max(1,trimmean(newBoxPolygon(:,2),20));
        
        if roix>=size(objectFrame,2) || roiy>=size(objectFrame,1)
            continue;
        end
        
        w = max(1,max(newBoxPolygon(:,1)) - min(newBoxPolygon(:,1)));
        h = max(1,max(newBoxPolygon(:,2)) - min(newBoxPolygon(:,2)));
        
        w = min(roix+w,size(objectFrame,2)-roix);
        h = min(roiy+h,size(objectFrame,1)-roiy);
        roi = [roix, roiy, w,h]
        
        scenePoints = detectSURFFeatures(rgb2gray(objectFrame),'MetricThreshold',pthresh, 'ROI', roi );
        [sceneFeatures, scenePoints] = extractFeatures(rgb2gray(objectFrame), scenePoints);
        featurePairs = matchFeatures(deckFeatures, sceneFeatures);
        % Show matches
        matchedBoxPoints = deckPoints(featurePairs(:, 1), :);
        matchedScenePoints = scenePoints(featurePairs(:, 2), :);
        
        matchedScenePoints.Count
        
        if matchedScenePoints.Count < 3 
            videoPlayer(objectFrame);
            fprintf('not enough points\n');
            continue;
        end
        
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
    

    

    
end


% Release the video player.
release(videoPlayer);



























