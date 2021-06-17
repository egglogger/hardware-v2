
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This script is used to plot the data from the egg loggers
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all

%%%% Directories
direc1 = '/Volumes/Booby/Data/EggLoggers/CAAUs/S2012/'; % Souurce of data to analyze
direc2 = '/Volumes/Booby/Data/EggLoggers/CAAUs/MatFiles/';  % Place to save data
direc3 = '/Volumes/Booby/Data/EggLoggers/CAAUs/Figures/';

%%%% Files to load
infile1 = '/20120417/2__20120417/RESULTS_02.mat';

%%%% Load files
load([direc1,infile1]);
time = time';
temp1 = temp1'; temp2 = temp2'; temp3 = temp3';
lux1 = lux1'; lux2 = lux2'; lux3 = lux3';
Pitch = Pitch'; PitchSigma = PitchSigma';
Roll = Roll'; RollSigma = RollSigma';
Yaw = Yaw'; YawSigma = YawSigma';

%%%%% Change from Radians to Degrees
PitchDeg = rad2deg(Pitch);
RollDeg = rad2deg(Roll);
YawDeg = rad2deg(Yaw);

%%%% Change Time
Days = time./86400;
Hours = time./3600;

% %%% Plot TEMPERATURE changes
% figure(1)
% axes('LineWidth',4);
% % plot(time(:,1),temp1(:,1),'-k','LineWidth',2); % time plotted in seconds
% plot(Hours(:,1),temp1(:,1),'-k','LineWidth',2); % time plotted in fractions of a day
% hold on
% set(gca,'LineWidth',4,'FontSize',24); % set y-axis parameters
% xlabel('Time (Hours)','FontSize',30); % in seconds
% % xlabel('Time (Fractions of Days)','FontSize',30); % in days
% ylabel('Temperature (C)','FontSize',30);

%%%% Plot OREIENTATION changes
figure(2)
axes('LineWidth',4);
plot(Hours(1:end-1,1),PitchDeg(:,1),'-k','LineWidth',2); % time(1:end-1,1)
hold on
plot(Hours(1:end-1,1),RollDeg(:,1),'-r','LineWidth',2);
plot(Hours(1:end-1,1),YawDeg(:,1),'-b','LineWidth',2);
set(gca,'LineWidth',4,'FontSize',24); % set y-axis parameters'YLim',[minRange maxRange]
% xlabel('Time (seconds)','FontSize',30);
xlabel('Time (Hours)','FontSize',30); % in days
ylabel('Orientation (degrees)','FontSize',30);
legend('Pitch','Yaw','Roll');

% %%%%% Double Y Plot
% figure(3)
% x = Hours(1:end-1,1);
% y1 = RollDeg(:,1);
% y2 = temp1(1:end-1,1);
% [AX,H1,H2] = plotyy(x,y1,x,y2,'plot');
% set(get(AX(1),'Ylabel'),'String','Roll Angle (deg)','FontSize',30); 
% set(get(AX(2),'Ylabel'),'String','Egg Temperature (^oC)','FontSize',30);
% xlabel('Recording Time (Hours)','FontSize',30);
% set(H1,'Linewidth',2,'Color','b');
% set(H2,'Linewidth',2,'Color','r');

% %%%%% Double plots
% x1 = Hours(1:end-1,1);
% y1 = RollDeg(:,1);
% x2 = Hours(1:end-1,1);
% y2 = temp1(1:end-1,1);
% hl1 = line(x1,y1,'Color','b','linewidth',2);
% ax1 = gca;
% set(ax1,'XColor','k','YColor','k','linewidth',4,'fontsize',24);
% set(get(ax1,'Ylabel'),'String','Roll Angle (deg)','fontsize',30);
% set(get(ax1,'Xlabel'),'String','Time (Hours)','fontsize',30);
% ax2 = axes('Position',get(ax1,'Position'),...
%            'XAxisLocation','top',...
%            'YAxisLocation','right',...
%            'Color','none',...
%            'XColor','k','YColor','k',...
%            'linewidth',4,'fontsize',24);
% hl2 = line(x2,y2,'Color','r','linewidth',2,'Parent',ax2);
% set(get(ax2,'Ylabel'),'String','Egg Temperature (^oC)','fontsize',30);
% % xlimits = get(ax1,'XLim');
% % ylimits = get(ax1,'YLim');
% % xinc = (xlimits(2)-xlimits(1))/10;
% % yinc = (ylimits(2)-ylimits(1))/5;
% % set(ax1,'XTick',[xlimits(1):xinc:xlimits(2)],...
% %         'YTick',[ylimits(1):yinc:ylimits(2)]);
% % xlimits2 = get(ax2,'XLim');
% % ylimits2 = get(ax2,'YLim');
% % xinc2 = (xlimits2(2)-xlimits2(1))/10;
% % yinc2 = (ylimits2(2)-ylimits2(1))/5;
% % set(ax2,'XTick',[xlimits2(1):xinc2:xlimits2(2)],...
% %         'YTick',[ylimits2(1):yinc2:ylimits2(2)]);

% %%%% Plot SIGMA OREIENTATION changes
% figure(3)
% axes('LineWidth',4);
% plot(time(1:end-1,1),PitchSigma(:,1),'-k','LineWidth',2);
% hold on
% plot(time(1:end-1,1),RollSigma(:,1),'-b','LineWidth',2);
% plot(time(1:end-1,1),YawSigma(:,1),'-r','LineWidth',2);
% set(gca,'LineWidth',4,'FontSize',24); % set y-axis parameters'YLim',[minRange maxRange]
% xlabel('Time (seconds)','FontSize',30);
% ylabel('Sigma Orientation','FontSize',30);
% legend('sPitch','sRoll','sYaw');





%% End
