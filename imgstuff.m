%cam = webcam;
%preview(cam);
clear all;
close all;

vidobj = imaq.VideoDevice();

%triggerconfig(vidobj, 'manual');
%start(vidobj)


%videoFileReader = vision.VideoFileReader('visionface.avi');
videoPlayer = vision.VideoPlayer('Position',[100,100,680,520]);
videoPlayer.show
%% 
% As an alternative, you can use the following commands to select the object 
% region using a mouse. The object must occupy the majority of the region:
objectFrame = flip(ycbcr2rgb(vidobj()),2);

figure; imshow(objectFrame); 
objectRegion=round(getPosition(imrect))

% Show initial frame with a red bounding box.
%%
objectImage = insertShape(objectFrame,'Rectangle',objectRegion,'Color','red'); 
figure;
imshow(objectImage);
title('Red box shows object region');
%% 
% Detect interest points in the object region.
points = detectMinEigenFeatures(rgb2gray(objectFrame),'ROI',objectRegion);

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
      frame = flip(frame,2);
      [points,validity] = tracker(frame);
      %if numel(points(validity, :)) < 10
      %    setPoints(tracker,points)
      %end
      out = insertMarker(frame,points(validity, :),'+');
      videoPlayer(out);
end
%% 
% Release the video reader and player.
%%
release(videoPlayer);
release(videoFileReader);