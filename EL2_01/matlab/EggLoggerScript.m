%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Egg Logger Script                 %
%                                   %
% by: Geoff Bower and Alex Naimain  %
%                                   %
% Started: March 12, 2012           %
% Last Updated: September 9, 2020   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all;
close all;
clc;
fclose('all');

debug = 0;

startdate = input('Enter start date as yyyymmdd:  '); % in yyyymmdd format
species = input('Enter species name (e.g. CAAU):  ', "s");
location = input('Enter location name (e.g. SEFI, ANI):  ', "s");

% Choose folder with data files
folder_name = uigetdir('.','Choose Directory with Data Files');

% Prompt user whether or not to show plots
plotsOn = input('Enter 0 to suppress plots (or press ''Enter'' for default, show plots): ');
if isempty(plotsOn)
    plotsOn = 1;
end

setTime = input('Enter 0 to trust the logger clock (or press ''Enter'' to set the power on and power off times): ');
if isempty(setTime)
    setTime = 1;
end
if setTime
    startYear = input('Enter the power-on year: ');
    startMonth = input('Enter the power-on month number (1 = Jan, 12 = Dec): ');
    startDay = input('Enter the power-on day number: ');
    startHour = input('Enter the power-on hour (24 hour format): ');
    startMin = input('Enter the power-on minute: ');
    startSec = input('Enter the power-on second: ');
    endYear = input('Enter the power-off year: ');
    endMonth = input('Enter the power-off month number (1 = Jan, 12 = Dec): ');
    endDay = input('Enter the power-off day number: ');
    endHour = input('Enter the power-off hour (24 hour format): ');
    endMin = input('Enter the power-off minute: ');
    endSec = input('Enter the power-off second: ');
    startDateNum = datenum([startYear,startMonth,startDay,startHour,startMin,startSec]);
    endDateNum = datenum([endYear,endMonth,endDay,endHour,endMin,endSec]);
end

pointedAxis = input('Which logger axis is most closely aligned with the long axis of the egg? (1 = x axis (default), 2 = y axis, 3 = z axis: ');
if isempty(pointedAxis)
    pointedAxis = 1;
end

cd(folder_name);

% See how many files there are
files = dir('*DAT*.TXT');
nestnum = files(1,1).name(1,1:2);
loggernum = str2num(nestnum);
startdate = num2str(startdate);
direc = '/Work/Data/EggLoggers/MatFiles/';
outfile = strcat('new_',nestnum,'_',startdate,'_RESULTS.mat');

% Allocate memory for data
dataSize = 0;
lastFileEmpty = 0;
for i = 1:length(files)
    if ~isempty(files(i).bytes)
        dataSize = dataSize + files(i).bytes/32;
    else
        lastFileEmpty = 1;
    end
end
time = 1:dataSize;
xAcc = zeros(1,dataSize);
yAcc = zeros(1,dataSize);
zAcc = zeros(1,dataSize);
xMag = zeros(1,dataSize);
yMag = zeros(1,dataSize);
zMag = zeros(1,dataSize);
temp1 = zeros(1,dataSize);
temp2 = zeros(1,dataSize);
temp3 = zeros(1,dataSize);
light1a = zeros(1,dataSize);
light1b = zeros(1,dataSize);
light2a = zeros(1,dataSize);
light2b = zeros(1,dataSize);
light3a = zeros(1,dataSize);
light3b = zeros(1,dataSize);

if setTime
    disp(['The unit was on for ',num2str(endDateNum-startDateNum),' days']);
    time = time*(endDateNum-startDateNum)*86400/length(time);
else
    disp(['The unit was on for ',num2str(length(time)/86400),' days']);
end

% Read the files
idx = 0;
for i = 1:(length(files)-lastFileEmpty)
    fid = fopen(files(i).name);
    while (~feof(fid))
        data = fread(fid,inf,'int16');
    end
    fclose(fid);
    xAccTemp(idx+1:idx+files(i).bytes/32) = data(2:16:end);
    yAccTemp(idx+1:idx+files(i).bytes/32) = data(3:16:end);
    zAccTemp(idx+1:idx+files(i).bytes/32) = data(4:16:end);
    xMagTemp(idx+1:idx+files(i).bytes/32) = data(5:16:end);
    yMagTemp(idx+1:idx+files(i).bytes/32) = data(6:16:end);
    zMagTemp(idx+1:idx+files(i).bytes/32) = data(7:16:end);
    temp1(idx+1:idx+files(i).bytes/32) = data(8:16:end);
    temp2(idx+1:idx+files(i).bytes/32) = data(9:16:end);
    temp3(idx+1:idx+files(i).bytes/32) = data(10:16:end);
    light1a(idx+1:idx+files(i).bytes/32) = data(11:16:end);
    light1b(idx+1:idx+files(i).bytes/32) = data(12:16:end);
    light2a(idx+1:idx+files(i).bytes/32) = data(13:16:end);
    light2b(idx+1:idx+files(i).bytes/32) = data(14:16:end);
    light3a(idx+1:idx+files(i).bytes/32) = data(15:16:end);
    light3b(idx+1:idx+files(i).bytes/32) = data(16:16:end);
    
    idx = idx + files(i).bytes/32;
end

%% Re-align axes for plotting Euler Angles
if pointedAxis == 1 % Default, X-axis aligned with long axis of egg
    xAcc = xAccTemp;
    yAcc = yAccTemp;
    zAcc = zAccTemp;
    xMag = xMagTemp;
    yMag = yMagTemp;
    zMag = zMagTemp;
elseif pointedAxis == 2 % Logger Y-axis is aligned with long axis of egg
    xAcc = yAccTemp;
    yAcc = zAccTemp;
    zAcc = xAccTemp;
    xMag = yMagTemp;
    yMag = zMagTemp;
    zMag = xMagTemp;
elseif pointedAxis == 3 % Logger Z-axis is aligned with long axis of egg
    xAcc = zAccTemp;
    yAcc = xAccTemp;
    zAcc = yAccTemp;
    xMag = zMagTemp;
    yMag = xMagTemp;
    zMag = yMagTemp;
end

%% Select data points to use for calibration
% Only look at middle 80% of time and remove acceleration outliers (assumes
% no large accelerometer biases)
ind = find(~(xAcc == 0) & ~(yAcc == 0) & ~(zAcc == 0) & ...
    (sqrt(xAcc.^2 + yAcc.^2 + zAcc.^2) < 1.025*2^14) & ...
    (sqrt(xAcc.^2 + yAcc.^2 + zAcc.^2) > 0.975*2^14) & ...
    (time > 0.1*(time(end)-time(1))) & (time < 0.9*(time(end)-time(1))) );

% Then randomly select nCalibrate from this sample
nCalibrate = min([500,length(ind)]);
ind = ind(1:ceil(length(ind)/nCalibrate):end);

meanXMag = mean(xMag(ind));
meanYMag = mean(yMag(ind));
meanZMag = mean(zMag(ind));

disp('Calibrating Accelerometer ...')
[AAcc,BAcc] = calibrate(xAcc(ind),yAcc(ind),zAcc(ind));
disp('Calibrating Magnetometer ...')
[AMag,BMag] = calibrate(xMag(ind)-meanXMag,yMag(ind)-meanYMag,zMag(ind)-meanZMag);

%% Transform and normalize accelerometer and magnetometer data based on the calibration
newXAcc = AAcc(1,:)*[xAcc;yAcc;zAcc] + BAcc(1);
newYAcc = AAcc(2,:)*[xAcc;yAcc;zAcc] + BAcc(2);
newZAcc = AAcc(3,:)*[xAcc;yAcc;zAcc] + BAcc(3);
newXMag = AMag(1,:)*[xMag-meanXMag;yMag-meanYMag;zMag-meanZMag] + BMag(1);
newYMag = AMag(2,:)*[xMag-meanXMag;yMag-meanYMag;zMag-meanZMag] + BMag(2);
newZMag = AMag(3,:)*[xMag-meanXMag;yMag-meanYMag;zMag-meanZMag] + BMag(3);

accMagnitude = sqrt(newXAcc.*newXAcc + newYAcc.*newYAcc + newZAcc.*newZAcc);
accMean = mean(accMagnitude);
accStd = std(accMagnitude);
magMagnitude = sqrt(newXMag.*newXMag + newYMag.*newYMag + newZMag.*newZMag);
magMean = mean(magMagnitude);
magStd = std(magMagnitude);

% For debugging / checking quality of calibration data
if debug
    fig1 = figure('Color','white');
    subplot(1,2,1)
    plot3(xAcc(ind),yAcc(ind),zAcc(ind),'.')
    title('Raw Accelerometer Data')
    xlabel('x [bits]')
    ylabel('y [bits]')
    zlabel('z [bits]')
    axis equal
    grid on
    view(3)
    
    subplot(1,2,2)
    hold on
    [X,Y,Z] = sphere(20);
    surf(X,Y,Z,'FaceAlpha',0.1,'EdgeAlpha',0.1,'FaceColor','g');
    plot3(newXAcc(ind),newYAcc(ind),newZAcc(ind),'.')
    title('Calibrated Accelerometer Data')
    xlabel('x [g]')
    ylabel('y [g]')
    zlabel('z [g]')
    axis equal
    grid on
    view(3)
    
    fig2 = figure('Color','white');
    subplot(1,2,1)
    plot3(xMag(ind),yMag(ind),zMag(ind),'.')
    title('Raw Magnetometer Data')
    xlabel('x [bits]')
    ylabel('y [bits]')
    zlabel('z [bits]')
    axis equal
    grid on
    view(3)
    
    subplot(1,2,2)
    hold on
    surf(X,Y,Z,'FaceAlpha',0.1,'EdgeAlpha',0.1,'FaceColor','g');
    plot3(newXMag(ind),newYMag(ind),newZMag(ind),'.')
    title('Mag')
    xlabel('x')
    ylabel('y')
    zlabel('z')
    axis equal
    grid on
    view(3)
%     posData = get(gcf,'Position');
%     Frames = getframe(gcf,[0,0,posData(3:4)]);
%     imwrite(Frames.cdata,'MagEx.png'); % Be sure to pad with 0's
    
    figure;
    hist(sqrt(newXAcc.^2 + newYAcc.^2 + newZAcc.^2)-1,200)
    title(['Accelerometer Error: Std = ',num2str(accStd)])
    
    figure;
    hist(sqrt(newXMag.^2 + newYMag.^2 + newZMag.^2)-1,200)
    title(['Magnetometer Error: Std = ',num2str(magStd)])
end


%% Determine vertical magnetic field strength from calibration data

% Normalize measurements to unit vectors
xAcc = newXAcc./accMagnitude;
yAcc = newYAcc./accMagnitude;
zAcc = newZAcc./accMagnitude;
xMag = newXMag./magMagnitude;
yMag = newYMag./magMagnitude;
zMag = newZMag./magMagnitude;

% Compute the angle between the gravity unit vector and magnetic field unit
% vector. Average over all of the calibration samples. Z field is the
% average.
dotProduct = zeros(length(ind),1);
for i = 1:length(ind)
    dotProduct(i) = xAcc(ind(i))*xMag(ind(i))+yAcc(ind(i))*yMag(ind(i))+zAcc(ind(i))*zMag(ind(i));
end
zField = -mean(dotProduct); % - sign since the accelerometer measures with the wrong sign (may change for new loggers)
disp(['Calibration Quality: ',num2str(1/std(dotProduct)),' (Should be 5 or greater)']);
xyz = [sqrt(1-zField^2),0,zField]';
disp(['Accelerometer Error: Mean = ',num2str(accMean),', Std = ',num2str(accStd)])
disp(['Magnetometer Error: Mean = ',num2str(magMean),', Std = ',num2str(magStd)])


%% Smooth raw sensor data after calibration corrections, not really necessary
windowTime = 1; % Number of sensor readings to use for averaging window
if windowTime > 1
    w = 0.54 - 0.46 * cos(2*pi*(0:windowTime-1)/(windowTime-1)); % Weights
    w = w / sum(w);
    xAcc = conv(xAcc,w,'same');
    yAcc = conv(yAcc,w,'same');
    zAcc = conv(zAcc,w,'same');
    xMag = conv(xMag,w,'same');
    yMag = conv(yMag,w,'same');
    zMag = conv(zMag,w,'same');
end

%% Convert temperature data
fid = fopen('CONFIG.TXT');
[~] = fgetl(fid);
[~] = fgetl(fid);
[~] = fgetl(fid);
[line4] = fgetl(fid);
fclose(fid);
if strncmp('No',line4,2)
    temp1 = 25 + temp1/128; % LSM303DLHC temperature sensor
else
    temp1 = temp1/256; % TMP102 sensor
end
temp2 = temp2/256;
temp3 = temp3/256;

% Convert light data
lux1 = zeros(1,dataSize);
lux2 = zeros(1,dataSize);
lux3 = zeros(1,dataSize);
for i = 1:length(light1a)
    % Check for overflow
    if light1a(i) < 0
        light1a(i) = light1a(i) + 2^16;
    end
    if light1b(i) < 0
        light1b(i) = light1b(i) + 2^16;
    end
    if light2a(i) < 0
        light2a(i) = light2a(i) + 2^16;
    end
    if light2b(i) < 0
        light2b(i) = light2b(i) + 2^16;
    end
    if light3a(i) < 0
        light3a(i) = light3a(i) + 2^16;
    end
    if light3b(i) < 0
        light3b(i) = light3b(i) + 2^16;
    end
    
    % Calculate lux levels
    if light1a(i) == 0
        lux1(i) = 0;
    elseif (light1b(i)/light1a(i) < 0.52)
        lux1(i) = (0.0315*light1a(i)) - (0.0593*light1a(i)*(light1b(i)/light1a(i))^1.4);
    elseif (light1b(i)/light1a(i) < 0.65)
        lux1(i) = (0.0229*light1a(i)) - (0.0291*light1b(i));
    elseif (light1b(i)/light1a(i) < 0.8)
        lux1(i) = (0.0157*light1a(i)) - (0.0180*light1b(i));
    elseif (light1b(i)/light1a(i) < 1.3)
        lux1(i) = (0.00338*light1a(i)) - (0.00260*light1b(i));
    else
        lux1(i) = 0;
    end
    
    if light2a(i) == 0
        lux2(i) = 0;
    elseif (light2b(i)/light2a(i) < 0.52)
        lux2(i) = (0.0315*light2a(i)) - (0.0593*light2a(i)*(light2b(i)/light2a(i))^1.4);
    elseif (light2b(i)/light2a(i) < 0.65)
        lux2(i) = (0.0229*light2a(i)) - (0.0291*light2b(i));
    elseif (light2b(i)/light2a(i) < 0.8)
        lux2(i) = (0.0157*light2a(i)) - (0.0180*light2b(i));
    elseif (light2b(i)/light2a(i) < 1.3)
        lux2(i) = (0.00338*light2a(i)) - (0.00260*light2b(i));
    else
        lux2(i) = 0;
    end
    
    if light3a(i) == 0
        lux3(i) = 0;
    elseif (light3b(i)/light3a(i) < 0.52)
        lux3(i) = (0.0315*light3a(i)) - (0.0593*light3a(i)*(light3b(i)/light3a(i))^1.4);
    elseif (light3b(i)/light3a(i) < 0.65)
        lux3(i) = (0.0229*light3a(i)) - (0.0291*light3b(i));
    elseif (light3b(i)/light3a(i) < 0.8)
        lux3(i) = (0.0157*light3a(i)) - (0.0180*light3b(i));
    elseif (light3b(i)/light3a(i) < 1.3)
        lux3(i) = (0.00338*light3a(i)) - (0.00260*light3b(i));
    else
        lux3(i) = 0;
    end
end



%% Run the Kalman filter

% Estimate initial state from first data
theta0 = asin(-xAcc(1)/sqrt(xAcc(1).*xAcc(1)+yAcc(1).*yAcc(1)+zAcc(1).*zAcc(1)));                               % Initial pitch estimate from accelerometer
phi0 = asin(yAcc(1)/sqrt(xAcc(1).*xAcc(1)+yAcc(1).*yAcc(1)+zAcc(1).*zAcc(1))./cos(theta0));                     % Initial roll estimate from accelerometer
psi0 = lsqnonlin(@findHeading,0,-2*pi,2*pi,optimset('Display','none'),theta0,phi0,xMag(1),yMag(1),zMag(1),xyz); % Initial heading guess from magnetometer
qPsi = [cos(psi0/2), 0, 0, sin(psi0/2)];        % Initial yaw quaternions
qTheta = [cos(theta0/2), 0, sin(theta0/2), 0];  % Initial pitch quaterion
qPhi = [cos(phi0/2), sin(phi0/2), 0, 0];        % initial roll quaternion

% Multiply qTheta * qPsi
qThPs = [qTheta(1)*qPsi(1)-qTheta(2)*qPsi(2)-qTheta(3)*qPsi(3)-qTheta(4)*qPsi(4);...
    qTheta(1)*qPsi(2)+qTheta(2)*qPsi(1)+qTheta(3)*qPsi(4)-qTheta(4)*qPsi(3);...
    qTheta(1)*qPsi(3)-qTheta(2)*qPsi(4)+qTheta(3)*qPsi(1)+qTheta(4)*qPsi(2);...
    qTheta(1)*qPsi(4)+qTheta(2)*qPsi(3)-qTheta(3)*qPsi(2)+qTheta(4)*qPsi(1)];
% Multiply the qPhi * (qThPs)
mu = [qPhi(1)*qThPs(1)-qPhi(2)*qThPs(2)-qPhi(3)*qThPs(3)-qPhi(4)*qThPs(4);...
    qPhi(1)*qThPs(2)+qPhi(2)*qThPs(1)+qPhi(3)*qThPs(4)-qPhi(4)*qThPs(3);...
    qPhi(1)*qThPs(3)-qPhi(2)*qThPs(4)+qPhi(3)*qThPs(1)+qPhi(4)*qThPs(2);...
    qPhi(1)*qThPs(4)+qPhi(2)*qThPs(3)-qPhi(3)*qThPs(2)+qPhi(4)*qThPs(1)];

n = length(mu); % State vector length

% Initial Covariance Matrix Estimate
Sigma = diag([0.1^2,0.1^2,0.1^2,0.1^2]);

% Process noise standard deviations
sigmaQuat = 0.1; % quaternion process noise
Rt = diag([sigmaQuat^2,sigmaQuat^2,sigmaQuat^2,sigmaQuat^2]);

% Measurement noise standard deviations
sigmaAx = accStd;     % x-accelerometer measurement noise, g's
sigmaAy = accStd;     % y-accelerometer measurement noise, g's
sigmaAz = accStd;     % z-accelerometer measurement noise, g's
sigmaMx = magStd;     % x-magnetometer measurement noise, fraction of field strength
sigmaMy = magStd;     % y-magnetometer measurement noise, fraction of field strength
sigmaMz = magStd;     % z-magnetometer measurement noise, fraction of field strength
Qt = diag([sigmaAx^2,sigmaAy^2,sigmaAz^2,sigmaMx^2,sigmaMy^2,sigmaMz^2]);

% Initialize variables to save data to and counters
counter = 1;
nSave = length(time)-1;
dataSave = zeros(n+1,nSave);
Roll = zeros(1,nSave);
Pitch = zeros(1,nSave);
Yaw = zeros(1,nSave);
RollSigma = zeros(1,nSave);
PitchSigma = zeros(1,nSave);
YawSigma = zeros(1,nSave);
chi2save = zeros(1,nSave);

%% Start EKF
disp(['Parsing Hour 1 of ',num2str(ceil(length(time)/3600))]);
while (counter < length(time))
    
    if mod(counter,3600) == 0
        disp(['Parsing Hour ',num2str(counter/3600+1),' of ',num2str(ceil(length(time)/3600))]);
    end
    
    % Save current state estimate
    dataSave(:,counter) = [mu;time(counter)];
    
    %% 2. Propogate current state through process model
    %mu = mu;
    
    %% 4. Update Covariance Estimate
    Sigma = Sigma + Rt*(time(counter+1)-time(counter));
    
    % If measurements are valid (~1g accelerometer reading) then run
    % measurement update;
    if abs(1-sqrt(xAcc(counter)^2+yAcc(counter)^2+zAcc(counter)^2)) < 0.025 || ...
            abs(1-sqrt(xMag(counter)^2+yMag(counter)^2+zMag(counter)^2)) < 0.025
        
        %% 5. Compute measurement model jacobian
        zbartTmp = Measure(mu,xyz);
        HtTmp = MeasureJacobian(mu,xyz);
        
        
        if abs(1-sqrt(xAcc(counter)^2+yAcc(counter)^2+zAcc(counter)^2)) < 0.025 && ...
            abs(1-sqrt(xMag(counter)^2+yMag(counter)^2+zMag(counter)^2)) < 0.025 % Both measurements valid
            zbart = zbartTmp;
            Ht = HtTmp;
            zt = [xAcc(counter);yAcc(counter);zAcc(counter);xMag(counter);yMag(counter);zMag(counter)];
        elseif abs(1-sqrt(xAcc(counter)^2+yAcc(counter)^2+zAcc(counter)^2)) < 0.025 % Only accel valid
            zbart = zbartTmp(1:3);
            Ht = HtTmp(1:3,:);
            zt = [xAcc(counter);yAcc(counter);zAcc(counter)];
        elseif abs(1-sqrt(xMag(counter)^2+yMag(counter)^2+zMag(counter)^2)) < 0.025 % only mag valid
            zbart = zbartTmp(4:6);
            Ht = HtTmp(4:6,:);
            zt = [xMag(counter);yMag(counter);zMag(counter)];
        end
        
        %% 6. Calculate Kalman Gain
        B = Sigma*Ht';
        A = Ht*B+Qt;
        Kt = B/A;
        
        %% 7. Calculate the Innovation and Apply Measurement Update
        % Actual measurements
        
        %% 8. Update state and covariance
        chi2save(counter) = (zt - zbart)' * (A \ (zt - zbart));
        mu = mu + Kt*(zt-zbart);
        mu = mu/sqrt(sum(mu.*mu));
        Sigma = (eye(4) - Kt*Ht)*Sigma;
        
    end
    
    % Compute Euler angles and their uncertainties
    e1 = mu(1);
    e2 = mu(2);
    e3 = mu(3);
    e4 = mu(4);
    
    % Euler angles in radians
    Roll(counter) = atan2(2*(e3.*e4+e1.*e2),1-2*(e2.^2+e3.^2));
    Pitch(counter) = asin(-2*(e2.*e4 - e1.*e3));
    Yaw(counter) = atan2(2*(e2.*e3+e1.*e4),1-2*(e3.^2+e4.^2));
    
    % Euler angle partial derivatives with respect to the quaternions
    dRollde1 = 2*e2/(1-2*e2^2-2*e3^2)/(1+(2*e3*e4+2*e1*e2)^2/(1-2*e2^2-2*e3^2)^2);
    dRollde2 = (2*e1/(1-2*e2^2-2*e3^2)+4*(2*e3*e4+2*e1*e2)/(1-2*e2^2-2*e3^2)^2*e2)/(1+(2*e3*e4+2*e1*e2)^2/(1-2*e2^2-2*e3^2)^2);
    dRollde3 = (2*e4/(1-2*e2^2-2*e3^2)+4*(2*e3*e4+2*e1*e2)/(1-2*e2^2-2*e3^2)^2*e3)/(1+(2*e3*e4+2*e1*e2)^2/(1-2*e2^2-2*e3^2)^2);
    dRollde4 = 2*e3/(1-2*e2^2-2*e3^2)/(1+(2*e3*e4+2*e1*e2)^2/(1-2*e2^2-2*e3^2)^2);
    
    dPitchde1 = 2*e3/(1-4*e2^2*e4^2+8*e2*e4*e1*e3-4*e1^2*e3^2)^(1/2);
    dPitchde2 = -2*e4/(1-4*e2^2*e4^2+8*e2*e4*e1*e3-4*e1^2*e3^2)^(1/2);
    dPitchde3 = 2*e1/(1-4*e2^2*e4^2+8*e2*e4*e1*e3-4*e1^2*e3^2)^(1/2);
    dPitchde4 = -2*e2/(1-4*e2^2*e4^2+8*e2*e4*e1*e3-4*e1^2*e3^2)^(1/2);
    
    dYawde1 = 2*e4/(1-2*e3^2-2*e4^2)/(1+(2*e2*e3+2*e1*e4)^2/(1-2*e3^2-2*e4^2)^2);
    dYawde2 = 2*e3/(1-2*e3^2-2*e4^2)/(1+(2*e2*e3+2*e1*e4)^2/(1-2*e3^2-2*e4^2)^2);
    dYawde3 = (2*e2/(1-2*e3^2-2*e4^2)+4*(2*e2*e3+2*e1*e4)/(1-2*e3^2-2*e4^2)^2*e3)/(1+(2*e2*e3+2*e1*e4)^2/(1-2*e3^2-2*e4^2)^2);
    dYawde4 = (2*e1/(1-2*e3^2-2*e4^2)+4*(2*e2*e3+2*e1*e4)/(1-2*e3^2-2*e4^2)^2*e4)/(1+(2*e2*e3+2*e1*e4)^2/(1-2*e3^2-2*e4^2)^2);
    
    % Euler angle uncertainties
    RollSigma(counter) = dRollde1*dRollde1*Sigma(1,1) + dRollde2*dRollde2*Sigma(2,2) + dRollde3*dRollde3*Sigma(3,3) + dRollde4*dRollde4*Sigma(4,4) + ...
        2*dRollde1*dRollde2*Sigma(1,2) + 2*dRollde1*dRollde3*Sigma(1,3) + 2*dRollde1*dRollde4*Sigma(1,4) + ...
        2*dRollde2*dRollde3*Sigma(2,3) + 2*dRollde2*dRollde4*Sigma(2,4) + 2*dRollde3*dRollde4*Sigma(3,4);
    PitchSigma(counter) = dPitchde1*dPitchde1*Sigma(1,1) + dPitchde2*dPitchde2*Sigma(2,2) + dPitchde3*dPitchde3*Sigma(3,3) + dPitchde4*dPitchde4*Sigma(4,4) + ...
        2*dPitchde1*dPitchde2*Sigma(1,2) + 2*dPitchde1*dPitchde3*Sigma(1,3) + 2*dPitchde1*dPitchde4*Sigma(1,4) + ...
        2*dPitchde2*dPitchde3*Sigma(2,3) + 2*dPitchde2*dPitchde4*Sigma(2,4) + 2*dPitchde3*dPitchde4*Sigma(3,4);
    YawSigma(counter) = dYawde1*dYawde1*Sigma(1,1) + dYawde2*dYawde2*Sigma(2,2) + dYawde3*dYawde3*Sigma(3,3) + dYawde4*dYawde4*Sigma(4,4) + ...
        2*dYawde1*dYawde2*Sigma(1,2) + 2*dYawde1*dYawde3*Sigma(1,3) + 2*dYawde1*dYawde4*Sigma(1,4) + ...
        2*dYawde2*dYawde3*Sigma(2,3) + 2*dYawde2*dYawde4*Sigma(2,4) + 2*dYawde3*dYawde4*Sigma(3,4);
    
    counter = counter + 1;
end

%% Optionally plot the results
if plotsOn == 1
    
    % Smooth values before plotting
    % Number of time steps to use for the smoothing window (roughly = seconds)
    windowTime = 61;
    
    % Compute the weights for the Hamming window used for smoothing
    w = 0.54 - 0.46 * cos(2*pi*(0:windowTime-1)/(windowTime-1));
    w = w/sum(w);
    
    RollPlot = conv(Roll,w,'same'); % Should be careful with unwrapping, but it's probably OK for plotting
    RollPlotSigma = conv(RollSigma,w,'same');
    PitchPlot = conv(Pitch,w,'same');
    PitchPlotSigma = conv(PitchSigma,w,'same');
    YawPlot = conv(Yaw,w,'same'); % Should be careful with unwrapping, but it's probably OK for plotting
    YawPlotSigma = conv(YawSigma,w,'same');
    temp1Plot = conv(temp1,w,'same');
    temp2Plot = conv(temp2,w,'same');
    temp3Plot = conv(temp3,w,'same');
    lux1Plot = conv(lux1,w,'same');
    lux2Plot = conv(lux2,w,'same');
    lux3Plot = conv(lux3,w,'same');

    % Plot the mean roll angle estimate +_ 3 standard deviations vs. time
    fig1 = figure('color','white');
    set(fig1,'units','normalized','position',[0.1,0.3,0.8,0.5]);
    hold on;
    if counter < 5000
        fill([dataSave(5,:)/3600,fliplr(dataSave(5,:)/3600)],[(RollPlot+3*sqrt(RollPlotSigma))*180/pi,fliplr(RollPlot-3*sqrt(RollPlotSigma))*180/pi],'r','FaceAlpha',0.3,'EdgeAlpha',0,'EdgeColor','r')
    end
    plot(dataSave(5,:)/3600,RollPlot*180/pi,'k')
    %     for i = 1:length(startInd)
    %         plot([dataSave(5,startInd(i)),dataSave(5,startInd(i))]/3600,[0,max(angleDiffSmooth)],'b')
    %         plot([dataSave(5,endInd(i)),dataSave(5,endInd(i))]/3600,[0,max(angleDiffSmooth)],'r')
    %     end
    xlabel('time [hr]')
    ylabel('Roll Angle [deg]')
    ylim([-180,180])
    grid on
    title('Mean Roll Estimate')
    
    % Plot the mean pitch angle estimate +_ 3 standard deviations vs. time
    fig2 = figure('color','white');
    set(fig2,'units','normalized','position',[0.1,0.3,0.8,0.5]);
    hold on;
    if counter < 5000
        fill([dataSave(5,:)/3600,fliplr(dataSave(5,:)/3600)],[(PitchPlot+3*sqrt(PitchPlotSigma))*180/pi,fliplr(PitchPlot-3*sqrt(PitchPlotSigma))*180/pi],'r','FaceAlpha',0.3,'EdgeAlpha',0,'EdgeColor','r')
    end
    plot(dataSave(5,:)/3600,PitchPlot*180/pi,'k')
    %     for i = 1:length(startInd)
    %         plot([dataSave(5,startInd(i)),dataSave(5,startInd(i))]/3600,[0,max(angleDiffSmooth)],'b')
    %         plot([dataSave(5,endInd(i)),dataSave(5,endInd(i))]/3600,[0,max(angleDiffSmooth)],'r')
    %     end
    ylim([-90,90])
    xlabel('time [hr]')
    ylabel('Pitch Angle [deg]')
    grid on
    title('Mean Pitch Estimate')
    
    % Plot the mean yaw angle estimate +_ 3 standard deviations vs. time
    fig3 = figure('color','white');
    set(fig3,'units','normalized','position',[0.1,0.3,0.8,0.5]);
    hold on;
    if counter < 5000
        fill([dataSave(5,:)/3600,fliplr(dataSave(5,:)/3600)],[(YawPlot+3*sqrt(YawPlotSigma))*180/pi,fliplr(YawPlot-3*sqrt(YawPlotSigma))*180/pi],'r','FaceAlpha',0.3,'EdgeAlpha',0,'EdgeColor','r')
    end
    plot(dataSave(5,:)/3600,YawPlot*180/pi,'k')
    %     for i = 1:length(startInd)
    %         plot([dataSave(5,startInd(i)),dataSave(5,startInd(i))]/3600,[0,max(angleDiffSmooth)],'b')
    %         plot([dataSave(5,endInd(i)),dataSave(5,endInd(i))]/3600,[0,max(angleDiffSmooth)],'r')
    %     end
    xlabel('time [hr]')
    ylabel('Yaw Angle [deg]')
    title('Mean Yaw Estimate (w.r.t. Magnetic North)')
    grid on
    ylim([-180,180])
    
    % Plot the uncertainty in each Euler angle vs. time
    fig5 = figure('color','white');
    set(fig5,'units','normalized','position',[0.1,0.3,0.8,0.5]);
    hold on;
    plot(dataSave(5,:)/3600,sqrt(RollPlotSigma)*180/pi,'k',dataSave(5,:)/3600,sqrt(PitchPlotSigma)*180/pi,'b',dataSave(5,:)/3600,sqrt(YawPlotSigma)*180/pi,'r')
    xlabel('time [hr]')
    ylabel('Euler Angle Uncertainty [deg]')
    title('Euler Angle Uncertainty')
    grid on
    legend('Roll','Pitch','Yaw')
    
    % Plot of how well the measurements agreed with the process model
    % (should be small when filter is converged, the measurements are
    % consistent, and egg is not moving).
    fig6 = figure('color','white');
    set(fig6,'units','normalized','position',[0.1,0.3,0.8,0.5]);
    plot(dataSave(5,:)/3600,chi2save);
    xlabel('Time [sec]');
    ylabel('Chi^2 Value');
    title('Filter Measurement Chi^2 Values')
    grid on
    
    % Plot Temp 1 vs. time
    fig7 = figure('color','white');
    set(fig7,'units','normalized','position',[0.1,0.3,0.8,0.5]);
    hold on;
    plot(time/3600,temp1Plot,'.')
    xlabel('time [hr]')
    ylabel('Temp1 [C]')
    grid on
    title('Temperature Sensor 1')
    
    % Plot Temp 2 vs. time
    fig8 = figure('color','white');
    set(fig8,'units','normalized','position',[0.1,0.3,0.8,0.5]);
    hold on;
    plot(time/3600,temp2Plot,'.')
    xlabel('time [hr]')
    ylabel('Temp2 [C]')
    grid on
    title('Temperature Sensor 2')
    
    % Plot Temp 3 vs. time
    fig9 = figure('color','white');
    set(fig9,'units','normalized','position',[0.1,0.3,0.8,0.5]);
    hold on;
    plot(time/3600,temp3Plot,'.')
    xlabel('time [hr]')
    ylabel('Temp3 [C]')
    grid on
    title('Temperature Sensor 3')
    
    % Plot Lux 1 vs. time
    fig10 = figure('color','white');
    set(fig10,'units','normalized','position',[0.1,0.3,0.8,0.5]);
    hold on;
    plot(time/3600,lux1Plot,'.')
    xlabel('time [hr]')
    ylabel('Lux 1 [lux]')
    grid on
    title('Light Sensor 1')
    
    % Plot Lux 2 vs. time
    fig11 = figure('color','white');
    set(fig11,'units','normalized','position',[0.1,0.3,0.8,0.5]);
    hold on;
    plot(time/3600,lux2Plot,'.')
    xlabel('time [hr]')
    ylabel('Lux 2 [lux]')
    grid on
    title('Light Sensor 2')
    
    % Plot Lux 3 vs. time
    fig12 = figure('color','white');
    set(fig12,'units','normalized','position',[0.1,0.3,0.8,0.5]);
    hold on;
    plot(time/3600,lux3Plot,'.')
    xlabel('time [hr]')
    ylabel('Lux 3 [lux]')
    grid on
    title('Light Sensor 3')
    
end

%%%%%
time = time';
temp1 = temp1'; temp2 = temp2'; temp3 = temp3';
lux1 = lux1'; lux2 = lux2'; lux3 = lux3';
Pitch = Pitch'; PitchSigma = PitchSigma';
Roll = Roll'; RollSigma = RollSigma';
Yaw = Yaw'; YawSigma = YawSigma';

Results = [time(1:end-1,1),Roll,Pitch,Yaw,RollSigma,PitchSigma,YawSigma,lux1(1:end-1,1),lux2(1:end-1,1),lux3(1:end-1,1),...
    temp1(1:end-1,1),temp2(1:end-1,1),temp3(1:end-1,1)];

% NewMat.species = species;
% NewMat.location = location;
% NewMat.loggerno = loggernum;
% NewMat.startDate = datevec(startDateNum);
% NewMat.julSDate = startDateNum;
% NewMat.endDate = datevec(endDateNum);
% NewMat.julEDate = endDateNum;
% NewMat.deployDays = endDateNum - startDateNum;
% NewMat.rawData = Results;
% 
% % Save the results to a matlab file
% % uisave({'Roll','Pitch','Yaw','RollSigma','PitchSigma','YawSigma','lux1','lux2','lux3','temp1','temp2','temp3','time'},'RESULTS.mat');
% save([direc,outfile],'NewMat');



%% Detect changes in orientation and plot statistics

% Calculate the orientation delta's between each time step. 'angleDiff' is
% the shortest rotation angle required to go from the orientation at time i
% to that at time i+1 by Euler's theorem. We don't care about the vector
% about which that rotation occurs here so we discard it.
[angleDiff,~] = rotationAngle(Roll(1:end-1),Pitch(1:end-1),Yaw(1:end-1),Roll(2:end),Pitch(2:end),Yaw(2:end));

% Next, we're going to smooth these angular changes to remove higher
% frequency content and will then look for times where the smoothed
% rotation rate exceeds some user defined threshold. This threshold is
% selected to be somewhere above the noise floor. In practice, something
% around 2e-2 [rad/s] appears to work well.

% Number of time steps to use for the smoothing window (roughly = seconds)
windowTime = 61;

% Compute the weights for the Hamming window used for smoothing
w = 0.54 - 0.46 * cos(2*pi*(0:windowTime-1)/(windowTime-1));
w = w/sum(w);

% Smooth the angular changes with the above window
angleDiffSmooth = conv(angleDiff,w,'same');

% Set the threshold to indicate there is some significant angular change.
thresh = 2e-2;

% Find the time steps at which the angle difference threshold is exceeded,
% we can think of this as there being a significant rotation rate over the
% time window of interest.
ind = find(angleDiffSmooth > thresh);

% Next we look for the delta times between the time steps that exceed the
% threshold. If the difference is 1, then it's an indication that the
% movement is still occuring. If it's greater than one there was some
% "stationary" time between the movement.
timeBetweenOverThresh = diff(ind);

% Find those indices corresponding to when there is time between movement.
endStartTime = find(timeBetweenOverThresh > 1);

% Find the indices right before each movement starts
startInd = ind(endStartTime(1:end-1)+1)-1;

% Find the indices right after each movement ends (need a +2, since we are
% looking at the differences between timesteps).
endInd = ind(endStartTime(2:end))+2;

% Now we can calculate the shortest rotation angle by Euler's theorem from
% right before the movement started until right after it ended. Again,
% we'll discard the vector about which the rotation occurs.
[deltaAngle,~] = rotationAngle(Roll(startInd),Pitch(startInd),Yaw(startInd),Roll(endInd),Pitch(endInd),Yaw(endInd));

% Example of finding rotations greater than some other threshold (say 10
% deg), and the times at which they start/stop
indicesGreaterThanNewThreshold = find(deltaAngle > 10 * pi/180);
startTimes = time(startInd(indicesGreaterThanNewThreshold));
endTimes = time(endInd(indicesGreaterThanNewThreshold));

if plotsOn
    % Here we plot the changes in orientation that occur for each movement vs.
    % the mean time at which they occurred.
    fig13 = figure('color','white');
    set(fig13,'units','normalized','position',[0.1,0.3,0.8,0.5]);
    hold on;
    plot((dataSave(5,startInd)+dataSave(5,endInd))/2/3600, deltaAngle *180/pi,'.')
    xlabel('Time [hr]')
    ylabel('Change in orientation [deg]')
    grid on
    title('Angle Change vs. Time')
    
    % Next, we plot the number of rotations exceeding a given angular threshold
    % normalized by the number of hours for the data set. I.e. rotations per
    % hour of at least a given size.
    dAngle = (0:180)*pi/180; % Angle bins we'll look at.
    numEventsPerHr = zeros(size(dAngle));
    for i = 1:length(dAngle)
        numEventsPerHr(i) = 3600*sum(deltaAngle > dAngle(i))/(max(time)-min(time));
    end
    fig14 = figure('color','white');
    set(fig14,'units','normalized','position',[0.1,0.3,0.8,0.5]);
    hold on;
    plot(dAngle*180/pi,numEventsPerHr)
    xlabel('Angular Change [deg]')
    ylabel('Cumulative Events Per Hour')
    grid on
    title('Cumulative Rotation Events per Hour vs. Anguler Change')
    
    % Lastly, we plot the smoothed angle changes as a function of time. In
    % addition we plot the times at which each rotation event starts with a
    % blue line, and the time it ends with a red line.
    fig15 = figure('color','white');
    set(fig15,'units','normalized','position',[0.1,0.3,0.8,0.5]);
    hold on
    plot(dataSave(5,1:end-1)/3600,angleDiffSmooth,'k')
    for i = 1:length(startInd)
        plot([dataSave(5,startInd(i)),dataSave(5,startInd(i))]/3600,[0,max(angleDiffSmooth)],'b')
        plot([dataSave(5,endInd(i)),dataSave(5,endInd(i))]/3600,[0,max(angleDiffSmooth)],'r')
    end
    xlabel('Time [hr]');
    ylabel('Smoothed angular rate [rad/s]');
    legend('Smoothed Angular Rate','Start Rotation','End Rotation');
    grid on
    title('Smoothed Angular Rate vs. Time')
    
    % Note: Un-commenting the for loops in the roll/pitch/yaw plots will 
    % indicate how well this method is at identifying the times when the 
    % orientation is changing (it looks pretty good!).
end