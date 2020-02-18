


% compute the RMS error and log Likelihood between simulations and data
% this code gives me the best fit to the mass simulations


% close all
clear all
clc


global sho_x sho_y

sho_x = 0;
sho_y = -0.4;

%% RMS error

% Simulations_dF = load('Simulations_direct_dF_+100%');
Simulations_dF = load('Simulations_direct_dF_+50%');

% datadir = 'C:\GoogleDrive\Synced Folder\Experimental Data\NoForce4Targets\Averages';
datadir = 'C:\Users\Yeki\Google Drive\Synced Folder\Experimental Data\NoForce4Targets\Averages';
% % % S = load([datadir,'/LimbFeedback_Averages2']);
% S = load([datadir,'/LimbFeedback_Averages_40']);
S = load([datadir,'/LimbFeedback_Averages_40b']);

% datadir = 'C:\Users\Yeki\Google Drive\Synced Folder\Experimental Data\NoForce4Targets_Original\NoForce4Target Experiments\NoForce4Targets\Averages';
% S = load([datadir,'/NoForceLimbPos_Block&Bin_AVGS.mat']);

ID = [];
Error_dF = [];

for counter = 1:4:length(Simulations_dF.ParamTrack)
    
    error_temp_dF = sum(RMS_v2_fun(Simulations_dF.ParamTrack,S, counter));

    Error_dF = [Error_dF, error_temp_dF];
    
    counter
    
end

ID = 1:4:length(Simulations_dF.ParamTrack);

%
[a,b] = min(Error_dF);
best_ID_dF = ID(b);
display(['RMS error for best fit, min dF: ', num2str(a)])
%
figure
hold on
for i = 1:4
    plot(Simulations_dF.ParamTrack{best_ID_dF+i-1}.Sol.X(:,1),Simulations_dF.ParamTrack{best_ID_dF+i-1}.Sol.X(:,2))
    plot(S.BlockData.X_avg{i}-sho_x, S.BlockData.Y_avg{i}-sho_y,'b--')
%     plot(S.BlockData.XrAvg{i}-sho_x, S.BlockData.YrAvg{i}-sho_y,'b--')
end
%
axis equal
title('best fit for RMS and min dF')

%% save the best fit

% % best_ID_dF = 645  % for the RMS error v2
for i = 1:4
    ParamTrack{i} = Simulations_dF.ParamTrack{best_ID_dF+i-1};
end

save('best_fit_dF_+50%','ParamTrack')

%% load the best fit for RMS and analyze it

clear all
clc

load('best_fit_dF_+100%')
% Data = load('SOLINIT_dF_30');

% how long does it take to run? 
% take 716 seconds to run 4 reaches ~ 12 minutes
tic

for i = 1:4
    
    Params = ParamTrack{i}.Parameters; 
    %%%%%%%%%%%%%%%% Obtimal Model    
    [Sol{i}, flag, algorithm, fval, Numpt, Numcon] = getOpt2LinkMuscleFFDirectMethod_RateOfForce_ineq_YZ_2(Params);

end

toc

% 
figure
hold on

for i = 1:4
    plot(Sol{i}.X(:,1),Sol{i}.X(:,2))
    plot(ParamTrack{i}.Sol.X(:,1),ParamTrack{i}.Sol.X(:,2),'--')
end

% 
figure
hold on

for i = 1:4
    subplot(2,2,i)
%     plot(ParamTrack{i}.Sol.U)
    plot(Sol{i}.Q(:,5:10))
end

%% Log Likelihood error

Simulations_dF = load('Simulations_direct_dF_+100%');

% datadir = 'C:\GoogleDrive\Synced Folder\Experimental Data\NoForce4Targets\Averages';
% S = load([datadir,'/LimbFeedback_Averages3']);

ID = [];
Error_dF = [];

for counter = 1:4:length(Simulations_dF.ParamTrack)
    
    error_temp_dF = sum(logL_fun(Simulations_dF.ParamTrack,S, counter));

    Error_dF = [Error_dF, error_temp_dF];
    
end

ID = 1:4:length(Simulations_dF.ParamTrack);

%
[a,b] = max(Error_dF);
best_ID_dF = ID(b);
display(['Log Likelihood error for best fit, min dF: ', num2str(a)])
%
figure
hold on
for i = 1:4
    plot(Simulations_dF.ParamTrack{best_ID_dF+i-1}.Sol.X(:,1),Simulations_dF.ParamTrack{best_ID_dF+i-1}.Sol.X(:,2))
    plot(S.BlockData.X_avg{i}-sho_x, S.BlockData.Y_avg{i}-sho_y,'b--')
end
%
axis equal
title('best fit for logL and min dF')



%% THE END

%% scratch
% clear all
% clc
% 
% global sho_x sho_y
% 
% sho_x = 0;
% sho_y = -0.4;
% 
% datadir = 'C:\Users\Yeki\Google Drive\Synced Folder\Experimental Data\NoForce4Targets\Averages';
% 
% S1 = load([datadir,'/LimbFeedback_Averages2']);
% S2 = load([datadir,'/LimbFeedback_Averages_40']);
% 
% figure
% hold on
% for i = 4
%       plot(S1.BlockData.X_avg{i}-sho_x)
%       plot(S2.BlockData.X_avg{i}-sho_x,'b--')
% end






