% Flood simulation based on water level rise - South Hilo District Region, Hawaii
% Sep 6, 2018
%  Copyright 2018 Payam Ghassemi
%  Author: Payam Ghassemi, payamgha@buffalo.edu

clc; clear all; close all;


tryDate = strcat(num2str(date()),'_',num2str(randi(100)));

logName = strcat('FloodSimulationLog_',...
                   tryDate,'.log');
diary(logName);

%% HAWAII (Big Island) - BIL format

%% Load Dataset
filename ='hawaii.bil';

% X and Y are measured in Meters (10m res.);
% Z (elevation) is measured in Foot;
% From the .hdr file, I obtained the following arguments for the MUTLIBANDREAD function.
sz =      [ 16698 13798 1];      % [ NROWS NCOLS NBANDS]
precision = 'int16';                  % from NBITS and PIXEL TYPE = int
offset = 0;                              % since the header is not included in this file
interleave = 'bil';                     % LAYOUT
byteorder = 'ieee-le';              % BYTEORDER = I (refers to Intel order or little endian format)

X_bil = multibandread(filename, sz, precision, offset, interleave, byteorder);
% Multiply by 0.3048 to convert foot to meter
X_bil = X_bil*0.3048;

stp = 5;
lstp = 10/1000; % 0.01km = 10 m
strIdx = 9000; %floor(sz(2)*0.6)+1;
endIdx = 12000; %floor(sz(2)*1);
strIdy = 7000; %floor(sz(1)*0.5)+1;
endIdy = 9000; %floor(sz(1)*1);
%Z = X_bil(1:stp:end, 1:stp:end);
Z = X_bil(strIdy:stp:endIdy,strIdx:stp:endIdx)/1000;
[ly,lx] = size(Z);
%x = [strIdx:endIdx]*stp*lstp;
%y = [endIdy:-1:strIdy]*stp*lstp;
x = [0:lx-1]*stp*lstp;
y = [ly-1:-1:0]*stp*lstp;
[X, Y] = meshgrid(x,y);
Z(Z==0) = NaN;
Z_all = X_bil(1:stp:end,1:stp:end)/1000;
Z_all(Z_all==0) = NaN;

%% Plot regions and all possible sample points
figure;
mesh(X,Y,Z,'FaceColor','flat');
%xlabel('[m]');
%ylabel('[m]');
zlabel('Elevation Above Sea [km]');
%demcmap(Z);
colorbar();
axis equal
%colormap('hsv');
% demcmap(Z_all);

az = 0;
el = 90;
view(az, el);
grid off
hold on;
region_1 = [0, 25; 12, 20; 1, 1]; %x;y;z
region_2 = [20, 30; 0, 12; 1, 1]; %x;y;z
region_3 = [0, 20; 0, 12; 1, 1]; %x;y;z
region_portion = [0.7, 0.2, 0.1];

region = region_1;
plot3( [region(1,1) region(1,2) region(1,2) region(1,1) region(1,1)],...
       [region(2,1) region(2,1) region(2,2) region(2,2) region(2,1)],...
       [1 1 1 1 1], 'k' )
region = region_2;
plot3( [region(1,1) region(1,2) region(1,2) region(1,1) region(1,1)],...
       [region(2,1) region(2,1) region(2,2) region(2,2) region(2,1)],...
       [1 1 1 1 1], 'k' )
region = region_3;
plot3( [region(1,1) region(1,2) region(1,2) region(1,1) region(1,1)],...
       [region(2,1) region(2,1) region(2,2) region(2,2) region(2,1)],...
       [1 1 1 1 1], 'k' )
   
[Xp1, Yp1] = plotGridPoints(region_1, 1);
[Xp2, Yp2] = plotGridPoints(region_2, 1);
[Xp3, Yp3] = plotGridPoints(region_3, 2);


Zp1 = interp2(X,Y,Z, Xp1, Yp1);
Zp2 = interp2(X,Y,Z, Xp2, Yp2);
Zp3 = interp2(X,Y,Z, Xp3, Yp3);
elevTh = 0;%0.001; % in Km; Threshold on land above elveTh above sea level (1 meter in this example);
Zp1(Zp1<elevTh) = NaN; % Only keep points above 1 meter of sea level
Zp2(Zp2<elevTh) = NaN; % Only keep points above 1 meter of sea level
Zp3(Zp3<elevTh) = NaN; % Only keep points above 1 meter of sea level

scatter3(Xp1(:),Yp1(:), Zp1(:), '.',...
        'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'k');
scatter3(Xp2(:),Yp2(:), Zp2(:), '.',...
        'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'k');
scatter3(Xp3(:),Yp3(:), Zp3(:), '.',...
        'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'k');
    
savefig(strcat('FloodSimulationResults_',...
        tryDate,'_DoE.fig'));
    
%% Show the whole Big Island (Hawaii) Map
figure;
imagesc(X_bil)               % Display the image file using IMAGESC
% demcmap(X_bil)                        % Changing the colormap 
axis equal
    
%% Simulation
figure;
mesh(X,Y,Z,'FaceColor','flat');
%xlabel('[m]');
%ylabel('[m]');
zlabel('Elevation Above Sea [km]');
colorbar();
%colormap('hsv');
% demcmap(Z_all);
axis equal
az = 0;
el = 90;
view(az, el);
zlabel('Elevation Above Sea [km]');
colorbar();
% Two regions of Water Level
px = 0.3;
py = 0.3;
Xs1 = [X(1,1), X(1,floor(lx*px)); 
       X(floor(lx*px),1), X(floor(lx*px),floor(lx*px))];
Ys1 = [Y(1,1), Y(1,floor(ly*py)); 
      Y(floor(ly*py),1), Y(floor(ly*py),floor(ly*py));];
Xs2 = [X(floor(lx*px),floor(lx*px)), X(floor(lx*px),end); 
      X(end,floor(lx*px)), X(end,end)];
Ys2 = [Y(1,1), Y(1,end); 
      Y(end,1), Y(end,end)];
FloodZ1 = [0.72, 0.18;
           0.72, 0.18];
FloodZ2 = [0, 0; 
           0, 0];

hold on;
scatter3(Xp1(:),Yp1(:), Zp1(:), '.',...
        'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'k');
scatter3(Xp2(:),Yp2(:), Zp2(:), '.',...
        'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'k');
scatter3(Xp3(:),Yp3(:), Zp3(:), '.',...
        'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'k');
Xs = Xs1; Ys = Ys1; FloodZ = FloodZ1;
mesh(Xs,Ys,FloodZ,'FaceColor',[93, 173, 226]/256);
Xs = Xs2; Ys = Ys2; FloodZ = FloodZ2;
mesh(Xs,Ys,FloodZ,'FaceColor',[93, 173, 226]/256);

savefig(strcat('FloodSimulationResults_',...
        tryDate,'_SimulationStart.fig'));

pause(1);

tStep = 1; %5; %5 min
floodRate = 0.004/60; %m per min;
endTime = 5*60;%5*60; % 5 hours = 300 min
floodDepth = 0.0005; % 0.5 meter above flooded water

dataRegion_1 = []; % x [km], y [km], z[km], time [min]
dataRegion_2 = []; % x [km], y [km], z[km], time [min]
dataRegion_3 = []; % x [km], y [km], z[km], time [min]
n1 = 0; n2 = 0; n3 = 0;
fprintf('t, R1, R2, R3\n');
enablePlot = 0;
for t = 0:tStep:endTime
% Check it's under flooded water or not?
% Evaluate surface's function @ point's X & Y
%px = 25; py = 5;
%spz = interp2(X,Y,Z,px,py);
%sz = interp2(Xs2,Ys2,FloodZ2,px,py);
%plot3(px,py,spz,'o','MarkerFaceColor','r');
%if spz < sz
%    disp('Flooded!');
%end

% Region 1
sz11 = interp2(Xs1,Ys1,FloodZ1,Xp1,Yp1);
% Detect & remove drown points and record time.
mask = (sz11+floodDepth) > Zp1;
if ~isempty(mask)
xDummy = Xp1(mask);
yDummy = Yp1(mask);
zDummy = Zp1(mask);
tDummy = t*ones(size(xDummy));
dataRegion_1 = [dataRegion_1; 
                xDummy(:), yDummy(:), zDummy(:), tDummy(:)];
end
Xp1(mask) = [];
Yp1(mask) = [];
Zp1(mask) = [];
n1 = length(xDummy) + n1;
sz12 = interp2(Xs2,Ys2,FloodZ2,Xp1,Yp1);
mask = (sz12+floodDepth) > Zp1;
if ~isempty(mask)
xDummy = Xp1(mask);
yDummy = Yp1(mask);
zDummy = Zp1(mask);
tDummy = t*ones(size(xDummy));
dataRegion_1 = [dataRegion_1; 
                xDummy(:), yDummy(:), zDummy(:), tDummy(:)];
end
Xp1(mask) = [];
Yp1(mask) = [];
Zp1(mask) = [];
n1 = length(xDummy) + n1;
% Region 2
sz22 = interp2(Xs2,Ys2,FloodZ2,Xp2,Yp2);
mask = sz22+floodDepth > Zp2;
if ~isempty(mask)
xDummy = Xp2(mask);
yDummy = Yp2(mask);
zDummy = Zp2(mask);
tDummy = t*ones(size(xDummy));
dataRegion_2 = [dataRegion_2; 
                xDummy(:), yDummy(:), zDummy(:), tDummy(:)];
end
Xp2(mask) = [];
Yp2(mask) = [];
Zp2(mask) = [];
n2 = length(xDummy) + n2;

% Region 3
sz32 = interp2(Xs2,Ys2,FloodZ2,Xp3,Yp3);
mask = sz32+floodDepth > Zp3;
if ~isempty(mask)
xDummy = Xp3(mask);
yDummy = Yp3(mask);
zDummy = Zp3(mask);
tDummy = t*ones(size(xDummy));
dataRegion_3 = [dataRegion_3; 
                xDummy(:), yDummy(:), zDummy(:), tDummy(:)];
end
Xp3(mask) = [];
Yp3(mask) = [];
Zp3(mask) = [];
n3 = length(xDummy) + n3;

%disp(t);
fprintf('%d, %d, %d, %d\n', t, n1, n2, n3);
if (enablePlot == 1)
pause(0.1);
children = get(gca, 'children');
delete(children(1));
children = get(gca, 'children');
delete(children(1));
end
FloodZ1(1,1) = FloodZ1(1,1) + tStep*floodRate*6;
FloodZ1(2,1) = FloodZ1(2,1) + tStep*floodRate;
FloodZ1(1,1) = FloodZ1(1,1) + tStep*floodRate*3.5;
FloodZ1(1,2) = FloodZ1(1,2) + tStep*floodRate*3;
Xs = Xs1; Ys = Ys1; FloodZ = FloodZ1;
if (enablePlot == 1)
mesh(Xs,Ys,FloodZ,'FaceColor',[93, 173, 226]/256);
end
FloodZ2 = FloodZ2 + tStep*floodRate;%*0.3;
Xs = Xs2; Ys = Ys2; FloodZ = FloodZ2;
if (enablePlot == 1)
mesh(Xs,Ys,FloodZ,'FaceColor',[93, 173, 226]/256);
end
end

% Region 1: No flooded points
mask = Zp1 > 0;
if ~isempty(mask)
xDummy = Xp1(mask);
yDummy = Yp1(mask);
zDummy = Zp1(mask);
tDummy = t*ones(size(xDummy));
dataRegion_1 = [dataRegion_1; 
                xDummy(:), yDummy(:), zDummy(:), tDummy(:)];
end
% Region 2: No flooded points
mask = Zp2 > 0;
if ~isempty(mask)
    xDummy = Xp2(mask);
yDummy = Yp2(mask);
zDummy = Zp2(mask);
tDummy = t*ones(size(xDummy));
dataRegion_2 = [dataRegion_2; 
                xDummy(:), yDummy(:), zDummy(:), tDummy(:)];
end
% Region 3: No flooded points
mask = Zp3 > 0;
if ~isempty(mask)
    xDummy = Xp3(mask);
yDummy = Yp3(mask);
zDummy = Zp3(mask);
tDummy = t*ones(size(xDummy));
dataRegion_3 = [dataRegion_3; 
                xDummy(:), yDummy(:), zDummy(:), tDummy(:)];
end
savefig(strcat('FloodSimulationResults_',...
        tryDate,'_SimulationEnd.fig'));
    
fileName = strcat('FloodSimulationResults_',...
                   tryDate,'.mat');
save(fileName);
diary off;

function [Xp, Yp] = plotGridPoints(region, stp)
strX = region(1,1)+stp/2;
endX = region(1,2)-stp/2;
xp = strX:stp:endX;
strY = region(2,1)+stp/2;
endY = region(2,2)-stp/2;
yp = strY:stp:endY;
[Xp, Yp] = meshgrid(xp,yp);
scatter3(Xp(:),Yp(:), ones(size(Xp(:))),...
        'MarkerFaceColor', 'None', 'MarkerEdgeColor', 'y');
end
% 
% %% Hawaii and South Hilo District
% figure;
% stp = 5;
% lstp = 10/1000; % 0.01km = 10 m
% strIdx = 9000; %floor(sz(2)*0.6)+1;
% endIdx = 12000; %floor(sz(2)*1);
% strIdy = 7000; %floor(sz(1)*0.5)+1;
% endIdy = 9000; %floor(sz(1)*1);
% Z_all = X_bil(1:stp:end,1:stp:end)/1000;
% [ly,lx] = size(Z_all);
% x = [0:lx-1]*stp*lstp;
% y = [ly-1:-1:0]*stp*lstp;
% [X_all, Y_all] = meshgrid(x,y);
% Z_all(Z_all==0) = NaN;
% mesh(X_all,Y_all,Z_all,'FaceColor','flat');
% axis equal;
% grid off;
% az = 0;
% el = 90;
% view(az, el);
% demcmap(Z_all);
% 
% Xs = [9000, 12000; 9000, 12000]/100;
% Ys = [7800, 7800; 9800, 9800]/100;
% FloodZ = zeros(2)+10;
% hold on;
% mesh(Xs,Ys,FloodZ,'FaceColor','None');
% 
% %set(gca,'xtick',[],'ytick',[]);
% set(gca,'XColor','none','YColor','none','TickDir','out');
% 
% figure;
% mesh(X,Y,Z,'FaceColor','flat');
% %xlabel('[m]');
% %ylabel('[m]');
% zlabel('Elevation Above Sea [km]');
% demcmap(Z_all);
% colorbar();
% axis equal
% 
% az = 0;
% el = 90;
% view(az, el);
% grid off
