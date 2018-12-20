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

while 1
    % Grav first fram to explore
    objectFrame = ycbcr2rgb(vidobj());

    % Find object
    scenePoints = detectSURFFeatures(rgb2gray(objectFrame));
    [sceneFeatures, scenePoints] = extractFeatures(rgb2gray(objectFrame), scenePoints);
    featurePairs = matchFeatures(deckFeatures, sceneFeatures);
    % Show matches
    matchedBoxPoints = deckPoints(featurePairs(:, 1), :);
    matchedScenePoints = scenePoints(featurePairs(:, 2), :);
    
    if matchedScenePoints.Count/2 >4 
        break;
    end
end
% Find transformation for object
[tform, inlierBoxPoints, inlierScenePoints] = ...
    estimateGeometricTransform(matchedBoxPoints, matchedScenePoints, 'affine');
boxPolygon = [1, 1;...                           % top-left
        size(rgb2gray(deckImg), 2), 1;...                 % top-right
        size(rgb2gray(deckImg), 2), size(rgb2gray(deckImg), 1);... % bottom-right
        1, size(rgb2gray(deckImg), 1);...                 % bottom-left
        1, 1]; 
newBoxPolygon = transformPointsForward(tform, boxPolygon);
points = matchedScenePoints;

% Create a tracker object.
tracker = vision.PointTracker('MaxBidirectionalError',1);
initialize(tracker,points.Location,objectFrame);

% Read, track, display points, and results in each video frame.
while 1
      frame = ycbcr2rgb(vidobj());
      [points,validity] = tracker(frame);
      fprintf('valid points: %d \n',numel(points(validity, :))/2 );
      while numel(points(validity, :))/2 <= 4
            % Reidentify matching points
            fprintf('not enough valid points...\n');
            scenePoints = detectSURFFeatures(rgb2gray(frame));
            [sceneFeatures, scenePoints] = extractFeatures(rgb2gray(frame), scenePoints);
            featurePairs = matchFeatures(deckFeatures, sceneFeatures);
            % Match points
            matchedBoxPoints = deckPoints(featurePairs(:, 1), :);
            matchedScenePoints = scenePoints(featurePairs(:, 2), :);
            % Calculate transform
            try
                [tform, inlierBoxPoints, inlierScenePoints] = ...
                    estimateGeometricTransform(matchedBoxPoints, matchedScenePoints, 'affine');
            catch ME
                fprintf('Not enough matches, grabbing new frame and continuing.\n');
                frame = ycbcr2rgb(vidobj());
                videoPlayer(frame);
                continue;
            end
            release(tracker);

            points = matchedScenePoints;
            initialize(tracker,points.Location,frame);
            [points,validity] = tracker(frame);
            fprintf('end of identify loop, have %d val points\n', numel(points(validity, :))/2 );
            
            if numel(points(validity, :))/2 <= 4 % So we dont get stuck here.
                frame = ycbcr2rgb(vidobj());
            end
      end
      out = insertMarker(frame,points(validity, :),'c','color','cyan','size', 6);
      videoPlayer(out);
end

% Release the video player.
release(videoPlayer);
























