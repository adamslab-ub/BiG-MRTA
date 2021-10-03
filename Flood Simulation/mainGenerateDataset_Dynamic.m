% Generate Random Dataset based on Flood simulation results
%- South Hilo District Region, Hawaii
% Sep 6, 2018
%  Copyright 2018 Payam Ghassemi
%  Author: Payam Ghassemi, payamgha@buffalo.edu

clc; clear all; close all;


tryDate = strcat(num2str(date()),'_',num2str(randi(100)));

logName = strcat('FloodSimulationLog_',...
    tryDate,'.log');
%diary(logName);

%% Load data
fileName = 'FloodSimulationResults_06-Sep-2018_49.mat';
load(fileName);

depotLocation = [10, 14]; % x,y
caseStudies = [10, 100]; % Number of tasks
runsPerCase = 1;

regionPortion = [0.7, 0.15, 0.15];

minThreshold = 10; % min

dataRegion = dataRegion_1;
dataRegion_c1 = dataRegion(dataRegion(:,4)>minThreshold,:);
dataRegion = dataRegion_2;
dataRegion_c2 = dataRegion(dataRegion(:,4)>minThreshold,:);
dataRegion = dataRegion_3;
dataRegion_c3 = dataRegion(dataRegion(:,4)>minThreshold,:);

%% Dynamic Case Studies
n = 150;
nFinal = 150;
for j = 1:runsPerCase
    taskData = [];
    for k = 1:3
        if k == 3
            dataRegion = dataRegion_c3;
            nS = n - size(taskData,1);
        elseif k == 2
            dataRegion = dataRegion_c2;
            nS = floor(n*regionPortion(k));
        elseif k == 1
            dataRegion = dataRegion_c1;
            nS = floor(n*regionPortion(k));            
        end
    N = size(dataRegion,1);
        idx = randperm(N,nS);
        dataDummy = dataRegion(idx,:);
        taskData = [taskData; dataDummy];
    end
    taskData = sortrows(taskData,4);
    save(strcat('FloodSim_DataSet_n',num2str(n),'_run_',num2str(j)), 'taskData', 'depotLocation');
end

return

%% Static Case Studies
for i = 1:length(caseStudies)
    N = size(dataRegion,1);
    n = caseStudies(i);
    for j = 1:runsPerCase
        taskData = [];
        for k = 1:3
            if k == 3
                nS = n - size(taskData,1);
            else
                nS = floor(n*regionPortion(k));
            end
            idx = randperm(N,nS);
            dataDummy = dataRegion(idx,:);
            taskData = [taskData; dataDummy];
        end
        save(strcat('FloodSim_DataSet_n',num2str(n),'_run_',num2str(j)), 'taskData', 'depotLocation');
    end
end
