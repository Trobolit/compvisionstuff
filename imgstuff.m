%% Init object to detect
clear all;
close all;


deckImg = imread('book.jpg');
deckPoints = detectSURFFeatures(rgb2gray(deckImg));

figure();
imshow(deckImg);
hold on;
plot(selectStrongest(deckPoints, 200));

[deckFeatures, deckPoints] = extractFeatures(rgb2gray(deckImg), deckPoints);

title('Image to identify');


%%


cam = webcam;
%preview(cam);

vidobj = imaq.VideoDevice();

%triggerconfig(vidobj, 'manual');
%start(vidobj)


%videoFileReader = vision.VideoFileReader('visionface.avi');
videoPlayer = vision.VideoPlayer('Position',[100,100,680,520]);
videoPlayer.show
%% 
% As an alternative, you can use the following commands to select the object 
% region using a mouse. The object must occupy the majority of the region:
flip(ycbcr2rgb(vidobj()),2); % dumy to allow camera to warm up
pause(3)
objectFrame = ycbcr2rgb(vidobj());

scenePoints = detectSURFFeatures(rgb2gray(objectFrame));
[sceneFeatures, scenePoints] = extractFeatures(rgb2gray(objectFrame), scenePoints);
featurePairs = matchFeatures(deckFeatures, sceneFeatures);

% Show matches
matchedBoxPoints = deckPoints(featurePairs(:, 1), :);
matchedScenePoints = scenePoints(featurePairs(:, 2), :);
figure;
showMatchedFeatures(deckImg, rgb2gray(objectFrame), matchedBoxPoints, ...
    matchedScenePoints, 'montage');
title('Putatively Matched Points (Including Outliers)');

%figure; imshow(objectFrame); 
%objectRegion=round(getPosition(imrect))

% Show initial frame with a red bounding box.
%% Show where in image using transform

[tform, inlierBoxPoints, inlierScenePoints] = ...
    estimateGeometricTransform(matchedBoxPoints, matchedScenePoints, 'affine');
boxPolygon = [1, 1;...                           % top-left
        size(rgb2gray(deckImg), 2), 1;...                 % top-right
        size(rgb2gray(deckImg), 2), size(rgb2gray(deckImg), 1);... % bottom-right
        1, size(rgb2gray(deckImg), 1);...                 % bottom-left
        1, 1]; 
newBoxPolygon = transformPointsForward(tform, boxPolygon);
figure;
imshow(objectFrame);
hold on;
line(newBoxPolygon(:, 1), newBoxPolygon(:, 2), 'Color', 'y');
title('Detected Box');

%%

tmp = newBoxPolygon';
lines = tmp(:)'; % Need to transpose before flattening for insertShape to work.
objectImage = insertShape(objectFrame,'line',lines,'Color','red'); 
figure;
imshow(objectImage);
title('Red box shows object region');
%% 
% Detect interest points in the object region.
%points = detectMinEigenFeatures(rgb2gray(objectFrame),'ROI',objectRegion);

points = matchedScenePoints;
% Display the detected points.
pointImage = insertMarker(objectFrame,points.Location,'+','Color','white');
figure;
imshow(pointImage);
title('Detected interest points');

% Create a tracker object.
tracker = vision.PointTracker('MaxBidirectionalError',1);

% Initialize the tracker.
initialize(tracker,points.Location,objectFrame);

% Read, track, display points, and results in each video frame.
for i=1:1000
      frame = ycbcr2rgb(vidobj());
      frame = frame;
      [points,validity] = tracker(frame);
      if numel(points(validity, :)) < 10
            release(tracker);
            % Reidentify matching points
            scenePoints = detectSURFFeatures(rgb2gray(frame));
            [sceneFeatures, scenePoints] = extractFeatures(rgb2gray(frame), scenePoints);
            featurePairs = matchFeatures(deckFeatures, sceneFeatures);
            % Match points
            matchedBoxPoints = deckPoints(featurePairs(:, 1), :);
            matchedScenePoints = scenePoints(featurePairs(:, 2), :);
            % Calculate transform
            [tform, inlierBoxPoints, inlierScenePoints] = ...
                estimateGeometricTransform(matchedBoxPoints, matchedScenePoints, 'affine');
            tmp = newBoxPolygon';
            lines = tmp(:)';
            points = matchedScenePoints;
            initialize(tracker,points.Location,frame);
            [points,validity] = tracker(frame);
            %if numel(points(validity, :)) < 3
            %    continue;
            %end

      end
      out = insertMarker(frame,points(validity, :),'+');
      videoPlayer(out);
end
%% 
% Release the video reader and player.
%%
release(videoPlayer);
release(videoFileReader);