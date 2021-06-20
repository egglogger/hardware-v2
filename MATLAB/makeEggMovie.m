% Plot a checkered Egg pattern

clear all;
clc;

% Plot one frame every frameDt seconds
frameDt = 60; 
maxTime = 60*60*6; % 1 day's worth

n = 48; % Control paneling on egg, should be a multiple of 4.

% Rotate so that pointy end of the egg is in the x direction
[X,Y,Z] = sphere(n);
sX = Z;
sY = Y;
sZ = -X;
[X,Y,Z] = ellipsoid(0,0,0,1,1,1.5,n);
eX = Z;
eY = Y;
eZ = -X;


%% Load orientation data here
filename = [];
while (isempty(filename))
    [filename,pathname] = uigetfile;
end
load([pathname,filename]);

% Create directory for images to go in
videoDir = '~/Desktop/Video/';
if ~exist(videoDir,'dir')
    mkdir(videoDir);
end
delete([videoDir,'*.png']);

numFrames = floor(min(length(NewMat.rawData(:,2)),maxTime)/frameDt);

%% Start for loop here to loop over orientations and make a movie
for frame = 1:numFrames
    
    % Rotate based on current roll/pitch/yaw attitude
    roll = NewMat.rawData((frame-1)*frameDt+1,2);
    pitch = NewMat.rawData((frame-1)*frameDt+1,3);
    yaw = NewMat.rawData((frame-1)*frameDt+1,4);
    
    DCM = [cos(pitch)*cos(yaw), sin(roll)*sin(pitch)*cos(yaw)-cos(roll)*sin(yaw), cos(roll)*sin(pitch)*cos(yaw)+sin(roll)*sin(yaw);
        cos(pitch)*sin(yaw), sin(roll)*sin(pitch)*sin(yaw)+cos(roll)*cos(yaw), cos(roll)*sin(pitch)*sin(yaw)-sin(roll)*cos(yaw);
        -sin(pitch)        , sin(roll)*cos(pitch)                            , cos(roll)*cos(pitch)];
    
    [a,b] = size(sX);
    sphereX = zeros(size(sX));
    sphereY = zeros(size(sX));
    sphereZ = zeros(size(sX));
    ellipX = zeros(size(eX));
    ellipY = zeros(size(eX));
    ellipZ = zeros(size(eX));
    for i = 1:a
        for j = 1:b
            in = [sX(i,j); sY(i,j); sZ(i,j)];
            out = DCM * in;
            sphereX(i,j) = out(1);
            sphereY(i,j) = out(2);
            sphereZ(i,j) = out(3);
            
            in = [eX(i,j); eY(i,j); eZ(i,j)];
            out = DCM * in;
            ellipX(i,j) = out(1);
            ellipY(i,j) = out(2);
            ellipZ(i,j) = out(3);
            
        end
    end
    
    figure(1);
    clf;
    
    % Set background color and turn off axes
    set(gcf, 'color',[0.1,0.3,0.9])
    hold on;
    plot3([0,1.5],[1.8,1.8],[0,0],'k->','LineWidth',2)
    plot3([0,0],[1.8,3.3],[0,0],'k->','LineWidth',2)
    text(1.5,2,0,'N','FontSize',18)
    text(0,3.5,0,'E','FontSize',18)
    text(0,0,-3,['t = ',num2str((frame-1)*frameDt+1,'%05d'),' sec'],'HorizontalAlignment','center','FontSize',18);
    surfColors = gray(4); % Set the colors of the egg to be from the gray colormap
    for i = 1:4
        surf(sphereX(1:(n/2+1),((i-1)*n/4+1):(i*n/4+1)), sphereY(1:(n/2+1),((i-1)*n/4+1):(i*n/4+1)), ...
            sphereZ(1:(n/2+1),((i-1)*n/4+1):(i*n/4+1)), 'EdgeAlpha',0.05,'EdgeColor','k','FaceColor',surfColors(mod(i,4)+1,:));
        surf(ellipX((n/2+1):(n+1),((i-1)*n/4+1):(i*n/4+1)), ellipY((n/2+1):(n+1),((i-1)*n/4+1):(i*n/4+1)), ...
            ellipZ((n/2+1):(n+1),((i-1)*n/4+1):(i*n/4+1)), 'EdgeAlpha',0.05,'EdgeColor','k','FaceColor',surfColors(mod(i+2,4)+1,:)); % Offset the ellipsoid colors by 2, to get more of a checkerboard pattern
    end
    axis equal
    view(-30,45)
    set(gca, 'color',[0,0,1], 'CameraViewAngle',8, 'visible','off','ZDir','reverse','YDir','reverse');
    set(gca, 'CameraPosition', [-13.0296   24.1510  -26.5592])
    set(gca, 'CameraTarget', [0,0,0]);
     
    % Grab an image of the figure here to make a movie
    posData = get(gcf,'Position');
    Frames = getframe(gcf,[0,0,posData(3:4)]);
    imwrite(Frames.cdata,[videoDir,num2str(frame,'%05d'),'.png']); % Be sure to pad with 0's
end


% Possible to do's:
% 1. Change time text to be date and time
% 2. Change background color with time of day?
% 3. Include a plot of temperature vs. time and indicate the
%    current time/temperature with a dot on that plot
%    Alternatively, change the base color of the egg to indicate
%    temperature (i.e. a range of blue to red, instead of just greys)
% 4. Low pass filter the orientations to smooth out some of the noise

% To make the video:
% 1. cd to the Video directory on your desktop
% 2. Run the following 'ffmpeg -r 30 -i %05d.png -vcodec mpeg4 -qscale 1 videoNameHere.mp4'
% 3. To optionally make the video smaller, convert using Handbrake using 
%    the H264 codec to .m4v to with little loss in quality, but ~1/4 the 
%    file size (RF = 20).

