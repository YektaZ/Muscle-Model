% across experiment comparisons with models


%% load the data
% close all
clear all
clc

global sho_x sho_y

sho_x = 0;
sho_y = -0.4;

% Group = 'LimbFeedback';
% datadir = 'C:\GoogleDrive\Synced Folder\Experimental Data\NoForce4Targets\Averages';
% % datadir = 'C:\Users\Yeki\Google Drive\Synced Folder\Experimental Data\NoForce4Targets\Averages';
% S = load([datadir,'/LimbFeedback_Averages_40b']);

% Group = 'CursorFeedback';
% % datadir = 'C:\Users\Yeki\Google Drive\Synced Folder\Experimental Data\NoForce4Targets_control\Averages';
% datadir = 'C:\GoogleDrive\Synced Folder\Experimental Data\NoForce4Targets_control\Averages';
% S = load([datadir,'/CursorFeedback_Averages_40b']);
% 
% Group = 'MixFeedback';
% % datadir = 'C:\Users\Yeki\Google Drive\Synced Folder\Experimental Data\NoForce4Target_mix\Averages';
% datadir = 'C:\GoogleDrive\Synced Folder\Experimental Data\NoForce4Target_mix\Averages';
% S = load([datadir,'/MixFeedback_Averages_40b']);

addpath('C:\GoogleDrive\Synced Folder\Matlab\Simple Two-Link Arm')
% addpath('C:\Users\Yeki\Google Drive\Synced Folder\Matlab\Simple Two-Link Arm');

% datadir = 'C:\Users\Yeki\Google Drive\Synced Folder\Experimental Data\NoForce4Targets_Original\NoForce4Target Experiments\NoForce4Targets\Averages';
% S = load([datadir,'/NoForceLimbPos_Block&Bin_AVGS.mat']);

% Load the nominal solutions
% Nominal_dF = load('nominal_solution_dF.mat'); % 0.75s
% Nominal_dT = load('nominal_solution_dT.mat'); % 0.75s
Nominal_dF = load('nominal_solution_dF_1sec.mat'); % 1s
Nominal_dT = load('nominal_solution_dT_1sec.mat'); % 1s
% Fit_100 = load('best_fit_dF_v3_+100%.mat'); %0.75 s
% Fit_50 = load('best_fit_dF_+50%.mat'); % 0.75 s
dF_avg20 = load('FR_AVG_20subs.mat');
dT_avg20 = load('TR_AVG_20subs.mat');
MJJ_avg20 = load('MJJ_AVG_20subs.mat');

%% Generate min jerk solution
% Generate min jerk solution in joint space

% addpath('C:\Users\Yeki\Google Drive\Synced Folder\Matlab\Minimum Jerk Functions');
addpath('C:\GoogleDrive\Synced Folder\Matlab\Minimum Jerk Functions');

N = length(Nominal_dF.ParamTrack{1}.Sol.X);
t = Nominal_dF.ParamTrack{1}.Sol.t;

for counter = 1:4
    a1 = Nominal_dF.ParamTrack{counter}.Parameters.a1;
    a2 = Nominal_dF.ParamTrack{counter}.Parameters.a2;
    %
    Xo = Nominal_dF.ParamTrack{counter}.Parameters.xstart(1:2);
    Xf = Nominal_dF.ParamTrack{counter}.Parameters.xfinish(1:2);
    %
    Xo_joint = inv_Position_2(Xo, a1, a2);
    Xf_joint = inv_Position_2(Xf, a1, a2);
    for i = 1:N
        X_mj{counter}(:,i) = min_Jerk_Position( t(i), t(1), t(end), Xo, Xf );
        dX_mj{counter}(:,i) = min_Jerk_Velocity( t(i), t(1), t(end), Xo, Xf );
        temp_joint = min_Jerk_Position( t(i), t(1), t(end), Xo_joint, Xf_joint);
        temp_vel_joint = min_Jerk_Velocity( t(i), t(1), t(end), Xo_joint, Xf_joint); %?
        X_joint{counter}(:,i) = forward_Position_2(temp_joint, a1, a2);
        dX_joint{counter}(:,i) = forward_Velocity( temp_joint, temp_vel_joint, a1, a2 );
    end
    
    MJ{counter}.Sol.X = [X_mj{counter}; dX_mj{counter}]';
    MJ_joint{counter}.Sol.X = [X_joint{counter}; dX_joint{counter}]';
    MJ{counter}.Sol.t = t;
    MJ_joint{counter}.Sol.t = t;
end

%% Get the min torque, two-link arm solution

for i = 1:4
    
    Parameters = Nominal_dF.ParamTrack{i}.Parameters;
    Parameters.sho_x = sho_x;
    Parameters.sho_y = sho_y;
    Parameters.R = eye(2); %two inputs to this system
    Parameters.Q = zeros(6);
    Parameters.Phi = zeros(6);
    
    [X,u,x_pos,y_pos, vel_pos] = TwoLinkArm_min_dT(Parameters);
    MT{i}.Sol.X = [x_pos;y_pos;vel_pos]';
    
    MT{i}.Sol.t = Nominal_dF.ParamTrack{i}.Sol.t';
    i
end

%% compute the RMS error between the nominal solution and averages

ID = [];
Error_mus_dF = [];
Error_mus_dT = [];
Error_dT = [];
% Error_dF_fit_100 = [];
% Error_dF_fit_50 = [];
Error_mj = [];
Error_joint = [];

for counter = 1:4:length(Nominal_dF.ParamTrack)
    
    error_temp_Mus_dF = sum(RMS_v2_fun(Nominal_dF.ParamTrack,S, counter));
    error_temp_Mus_dT = sum(RMS_v2_fun(Nominal_dT.ParamTrack,S, counter));
    error_temp_dT = sum(RMS_v2_fun(MT,S, counter));
%     error_temp_dF_fit_100 = sum(RMS_v2_fun(Fit_100.ParamTrack,S, counter));
%     error_temp_dF_fit_50 = sum(RMS_v2_fun(Fit_50.ParamTrack,S, counter));
    error_temp_mj = sum(RMS_v2_fun(MJ,S, counter)); %different numbers, but add to the same number? How?
    error_temp_joint = sum(RMS_v2_fun(MJ_joint,S, counter));
    
    Error_mus_dF = [Error_mus_dF, error_temp_Mus_dF];
    Error_mus_dT = [Error_mus_dT, error_temp_Mus_dT];
    Error_dT = [Error_dT, error_temp_dT];
%     Error_dF_fit_100 = [Error_dF_fit_100, error_temp_dF_fit_100];
%     Error_dF_fit_50 = [Error_dF_fit_50, error_temp_dF_fit_50];
    Error_mj = [Error_mj, error_temp_mj];
    Error_joint = [Error_joint, error_temp_joint];
    
    counter
    
end

%% plot error bars

figure
hold on
labels = {'nom min dF', 'min dT','min jerk','joint jerk'}; %,'best fit +100% dF','best fit +50% dF'};
Errors = [Error_mus_dF, Error_dT, Error_mj, Error_joint]; %, Error_dF_fit_100,Error_dF_fit_50];
Colors = ['r','y','g','b']; %,'m','k'];
for i = 1:length(Errors)
    h = bar(i,Errors(i));
    set(h, 'FaceColor', Colors(i))
end
xticks(1:length(Errors))
xticklabels(labels)
title('RMS error with avg data')

figure
hold on

for i = 1:4
    plot(Nominal_dF.ParamTrack{i}.Sol.X(:,1), Nominal_dF.ParamTrack{i}.Sol.X(:,2),'r')
%     plot(Nominal_dT.ParamTrack{i}.Sol.X(:,1), Nominal_dT.ParamTrack{i}.Sol.X(:,2),'m')
    plot(MT{i}.Sol.X(:,1), MT{i}.Sol.X(:,2),'y')
    plot(MJ{i}.Sol.X(:,1),MJ{i}.Sol.X(:,2),'g')
    plot(MJ_joint{i}.Sol.X(:,1),MJ_joint{i}.Sol.X(:,2),'b')
%     plot(Fit_100.ParamTrack{i}.Sol.X(:,1), Fit_100.ParamTrack{i}.Sol.X(:,2),'m')
%     plot(Fit_50.ParamTrack{i}.Sol.X(:,1), Fit_50.ParamTrack{i}.Sol.X(:,2),'k')
    id = S.BlockData.X_Cnt{i}>9;
    %    plot(S.BlockData.XrAvg{i}(id)-sho_x, S.BlockData.YrAvg{i}(id)-sho_y,'k--')
    plot(S.BlockData.X_avg{i}(id)-sho_x, S.BlockData.Y_avg{i}(id)-sho_y,'k--')
end
axis equal
axis([-0.15,0.15,0.25,0.55])
legend('nom min dF','nom min dT','min dT','mj','joint'); %,'best fit +100%','best fit +50%','data')

%% Compute and plot bar plot for enclosed area

% compute the enclosed area (gap) between the data and the simulation

% close all

Targets0 = [-.1 -.1;
    0.1 -0.1;
    0.1 0.1;
    -0.1 0.1];

TargetsF = [Targets0(2:end,:);
    Targets0(1,:)];

labels = {'min jerk','joint jerk', 'min dT','nom min dF'}; %,'best fit +100% dF','best fit +50% dF'};
Colors = ['g','b','y','r',]; %,'m','k'];
figure

EArea_nominal_dF = GapArea_fun(S,Nominal_dF.ParamTrack, Targets0, TargetsF);
% EArea_nominal_dT = GapArea_fun(S,Nominal_dT.ParamTrack, Targets0, TargetsF);
EArea_MT = GapArea_fun(S,MT, Targets0, TargetsF);
EArea_MJ = GapArea_fun(S,MJ, Targets0, TargetsF);
EArea_MJ_joint = GapArea_fun(S,MJ_joint, Targets0, TargetsF);
% EArea_Fit_100 = GapArea_fun(S,Fit_100.ParamTrack, Targets0, TargetsF);
% EArea_Fit_50 = GapArea_fun(S,Fit_50.ParamTrack, Targets0, TargetsF);

for i = 1:4
    
    gap = [ EArea_MJ(i), EArea_MJ_joint(i), EArea_MT(i), EArea_nominal_dF(i)]; %, EArea_Fit_100(i), EArea_Fit_50(i)];
    
    subplot(2,2,i)
    hold on
    
    for bar_num = 1:length(labels)
        h = bar( bar_num, gap(bar_num));
        set(h, 'FaceColor', Colors(bar_num))
    end
    
    title('Enclosed Area ')
    i
end

legend(labels)

for i = 1:4
    subplot(2,2,i)
    axis([0,length(labels)+1, 0, 1.5*max( EArea_MT) ])
end

%%%%%%%%%%%%%%%%%%%%%%%% one plot for all reaches

% MT and nominal _min_dT look suspisious

figure
hold on
gap = [ sum(EArea_MJ), sum(EArea_MJ_joint),sum(EArea_MT),sum(EArea_nominal_dF)];

for bar_num = 1:length(labels)
    h = bar( bar_num, gap(bar_num));
    set(h, 'FaceColor', Colors(bar_num))
end

axis([0,length(labels)+1, 0, 1.5*max( gap) ])
title('Enclosed Area with Data - sum 4 reaches')

%% Compute and plot the bar plots for Normalized path length

NL_nominal_dF = NPathLength_fun(Nominal_dF.ParamTrack);
% NL_nominal_dT = NPathLength_fun(Nominal_dT.ParamTrack);
NL_MT = NPathLength_fun(MT);
NL_MJ = NPathLength_fun(MJ);
NL_MJ_joint = NPathLength_fun(MJ_joint);
%
NL_Data = NPathLength_data_fun(S); 

figure

labels = {'min jerk','joint jerk', 'min dT','nom min dF'}; %,'best fit +100% dF','best fit +50% dF'};
Colors = ['g','b','y','r']; 

for i = 1:4
    
    NL = abs( NL_Data(i)- ...
        [NL_MJ(i), NL_MJ_joint(i), NL_MT(i), NL_nominal_dF(i)]); 
    
    subplot(2,2,i)
    hold on
    
    for bar_num = 1:length(labels)
        h = bar( bar_num, NL(bar_num));
        set(h, 'FaceColor', Colors(bar_num))
    end
    
    title('Normalized path length difference with data')
    axis([0,length(labels)+1, 0, 1.2*0.0879 ])
    i
end

legend(labels)

%%%%%%%%%%%%%%%%%%%%%%%% one plot for all reaches

figure
hold on
NL = [ sum(NL_MJ), sum(NL_MJ_joint), sum(NL_MT), sum(NL_nominal_dF)];

for bar_num = 1:length(labels)
    abs(sum(NL_Data) - NL(bar_num))
    h = bar( bar_num, abs(sum(NL_Data) - NL(bar_num)));
    set(h, 'FaceColor', Colors(bar_num))
end

% axis([0,length(labels)+1, 0, 0.1 ])
title('normalized path length difference with data - sum 4 reaches')

%% Perpendicular Error

PE_nominal_dF = PerpError_fun(Nominal_dF.ParamTrack);
% PE_nominal_dT = PerpError_fun(Nominal_dT.ParamTrack);
PE_MT = PerpError_fun(MT);
PE_MJ = PerpError_fun(MJ);
PE_MJ_joint = PerpError_fun(MJ_joint);
%
PE_Data = PerpError_data_fun(S); 
%
PE_sim = [PE_MJ; PE_MJ_joint; PE_MT; PE_nominal_dF];

temp_PE = abs(PE_Data - PE_sim); % distance table with perp errors

figure

labels = {'min jerk','joint jerk', 'min dT','nom min dF'}; 
Colors = ['g','b','y','r']; 

for i = 1:4
    
    PE = temp_PE(:,i); 
    
    subplot(2,2,i)
    hold on
    
    for bar_num = 1:length(labels)
        h = bar( bar_num, PE(bar_num));
        set(h, 'FaceColor', Colors(bar_num))
    end
    
    title('perp error difference with data')
    axis([0,length(labels)+1, 0, 1.2*0.0879 ])
    i
end

legend(labels)

%%%%%%%%%%%%%%%%%%%%%%%% one plot for all reaches

%the sign matters

figure
hold on

for bar_num = 1:length(labels)
    sum(temp_PE(bar_num,:))
    h = bar( bar_num, sum(temp_PE(bar_num,:)) );
    set(h, 'FaceColor', Colors(bar_num))
end

title('perp error difference with data - sum 4 reaches')

%% Log Likelihood

logL_nominal_dF = logL_v2_fun(Nominal_dF.ParamTrack,S);
logL_nominal_dT = logL_v2_fun(Nominal_dT.ParamTrack,S);
logL_MT = logL_v2_fun(MT,S);
logL_MJ = logL_v2_fun(MJ,S);
logL_MJ_joint = logL_v2_fun(MJ_joint,S);
%
% logL_Data = PerpError_data_fun(S); 
%
logL_sim = [logL_nominal_dF; logL_nominal_dT; logL_MT; logL_MJ; logL_MJ_joint];

% temp = abs(logL_Data - PE_sim); % distance table with perp errors

figure

labels = {'nom min dF','nom min dT', 'min dT','min jerk','joint jerk'}; 
Colors = ['r','m','y','g','b']; 

for i = 1:4
    
    logL = logL_sim(:,i); 
    
    subplot(2,2,i)
    hold on
    
    for bar_num = 1:length(labels)
        h = bar( bar_num, logL(bar_num));
        set(h, 'FaceColor', Colors(bar_num))
    end
    
    title('Log Likelihood')
%     axis([0,length(labels)+1, 0, 1.2*0.0879 ])
    i
end

legend(labels)

%%%%%%%%%%%%%%%%%%%%%%%% one plot for all reaches

%the sign matters

figure
hold on

for bar_num = 1:length(labels)
    h = bar( bar_num, sum(logL(bar_num,:)) );
    set(h, 'FaceColor', Colors(bar_num))
end

title('Log Likelihood - sum 4 reaches')
legend(labels)

%% goodness of fit with Wolpert's method

% !!!!!!!!!!!!! check the shoulder location!
% Nominal_dF
for i = 1:4
    
    N = sum(S.BlockData.X_Cnt{i}>9);
    %%%%%%%%%%%%%%%%%     interpolate
    
    Speed_sim = sqrt(Nominal_dF.ParamTrack{i}.Sol.X(:,3).^2 + Nominal_dF.ParamTrack{i}.Sol.X(:,4).^2);
    
    T = Nominal_dF.ParamTrack{i}.Sol.t;
    TT = S.BlockData.Time{i}(51:N);
    dt = TT(2)-TT(1); 
    
    count = 1;
    for j = 0.001:dt:T(end)
        Speed_temp(count) = interp1(T,Speed_sim,j);
        X_temp(count) = interp1(T,Nominal_dF.ParamTrack{i}.Sol.X(:,1),j);
        Y_temp(count) = interp1(T,Nominal_dF.ParamTrack{i}.Sol.X(:,2),j);
        count = count + 1;
    end
    
    id0_data = find(S.BlockData.Time{i}>0, 1);
    id0_sim = find(Speed_temp>0.05, 1);
    
    numpt = length(Speed_temp); % numpt for simulations
    
    id_data = floor(linspace(id0_data, N, 700)); % only consider 100 data points
    id_sim = floor(linspace(id0_sim,numpt, 700));
   
    X_data = S.BlockData.X_avg{i}(id_data);
    Y_data = S.BlockData.Y_avg{i}(id_data);
    
   R2_Nominal_dF(i) = 1- (var( X_data' - X_temp(id_sim)) + var(Y_data' - Y_temp(id_sim)))/(var(X_data) + var(Y_data));
    
end


% MJ
for i = 1:4
    
    N = sum(S.BlockData.X_Cnt{i}>9);
    %%%%%%%%%%%%%%%%%     interpolate
    
    Speed_sim = sqrt(MJ{i}.Sol.X(:,3).^2 + MJ{i}.Sol.X(:,4).^2);
    
    T = MJ{i}.Sol.t;
    TT = S.BlockData.Time{i}(51:N);
    dt = TT(2)-TT(1); 
    
    count = 1;
    for j = 0.001:dt:T(end)
        Speed_temp(count) = interp1(T,Speed_sim,j);
        X_temp(count) = interp1(T,MJ{i}.Sol.X(:,1),j);
        Y_temp(count) = interp1(T,MJ{i}.Sol.X(:,2),j);
        count = count + 1;
    end
    
    id0_data = find(S.BlockData.Time{i}>0, 1);
    id0_sim = find(Speed_temp>0.05, 1);
    
    numpt = length(Speed_temp); % numpt for simulations
    
    id_data = floor(linspace(id0_data, N, 700)); % only consider 100 data points
    id_sim = floor(linspace(id0_sim,numpt, 700));
   
    X_data = S.BlockData.X_avg{i}(id_data);
    Y_data = S.BlockData.Y_avg{i}(id_data);
    
   R2_MJ(i) = 1- (var( X_data' - X_temp(id_sim)) + var(Y_data' - Y_temp(id_sim)))/(var(X_data) + var(Y_data));
    
end

%% New metric for model comparison without time

% close all

figure
hold on

for i = 1:4
    plot(Nominal_dF.ParamTrack{i}.Sol.X(:,1), Nominal_dF.ParamTrack{i}.Sol.X(:,2),'r')
    plot(MT{i}.Sol.X(:,1), MT{i}.Sol.X(:,2),'y')
    plot(MJ{i}.Sol.X(:,1),MJ{i}.Sol.X(:,2),'g')
    plot(MJ_joint{i}.Sol.X(:,1),MJ_joint{i}.Sol.X(:,2),'b')
    id = (S.BlockData.X_Cnt{i}>=S.BlockData.X_Cnt{i}(1));
    plot(S.BlockData.X_avg{i}(id)-sho_x, S.BlockData.Y_avg{i}(id)-sho_y,'k--')
end
legend('nom min dF','min dT','mj','joint'); 

Error_dF = TimeLessError_fun(Nominal_dF.ParamTrack, S);
Error_MT = TimeLessError_fun(MT, S);
Error_MJJ = TimeLessError_fun(MJ_joint, S);
Error_MJ = TimeLessError_fun(MJ, S);

Errors_rms = [Error_MJ; Error_MJJ; Error_MT; Error_dF];

figure

labels = {'min jerk','joint jerk', 'min dT','nom min dF'}; 
Colors = ['g','b','y','r']; 

for i = 1:4
    
    e = Errors_rms(:,i); %first reach for all models
    
    subplot(2,2,i)
    hold on
    
    for bar_num = 1:length(labels)
        h = bar( bar_num, e(bar_num));
        set(h, 'FaceColor', Colors(bar_num))
    end
    
    title('Time Less Error')
%     axis([0,length(labels)+1, 0, 1.2*0.0879 ])
    i
end

legend(labels)

%%%%%%%%%%%%%%%%%%%%%%%% one plot for all reaches

figure
hold on

for bar_num = 1:length(labels)
    h = bar( bar_num, sum(Errors_rms(bar_num,:)) );
    set(h, 'FaceColor', Colors(bar_num))
end

title('Time Less Error - sum 4 reaches')
legend(labels)

%% model comparison for individual subjects with std's - time less error - --> looks horrible!

% clear all
% close all
clc

% Define the Subjects

% Group = 'LimbFeedback'; 
% datadir = 'C:\GoogleDrive\Synced Folder\Experimental Data\NoForce4Targets'; 
% % datadir = 'C:\Users\Yeki\Google Drive\Synced Folder\Experimental Data\NoForce4Targets';
% Subjects = {'NF04', 'NF05', 'NF06', 'NF07', 'NF08', 'NF15', 'NF16', 'NF30', 'NF31', 'NF32'}; 

% Group = 'CursorFeedback';
% % datadir = 'C:\Users\Yeki\Google Drive\Synced Folder\Experimental Data\NoForce4Targets_control';
% datadir = 'C:\GoogleDrive\Synced Folder\Experimental Data\NoForce4Targets_control'; 
% % % % Subjects = {'NF20', 'NF21', 'NF22', 'NF23', 'NF24', 'NF25', 'NF26', 'NF27', 'NF33', 'NF34'};
% Subjects = {'NF20', 'NF21', 'NF22', 'NF23', 'NF24', 'NF25', 'NF26', 'NF33', 'NF34'};

Group = 'MixFeedback';
% datadir = 'C:\Users\Yeki\Google Drive\Synced Folder\Experimental Data\NoForce4Target_mix';
datadir = 'C:\GoogleDrive\Synced Folder\Experimental Data\NoForce4Target_mix';
Subjects = {'NF42', 'NF43', 'NF44', 'NF45', 'NF46', 'NF47', 'NF48', 'NF49', 'NF50', 'NF51'}; 

num_subs = length(Subjects);

Error_dF = [];
Error_MT = [];
Error_MJJ = [];
Error_MJ = [];

for sub = 1:num_subs

    S = load([datadir,'/',Subjects{sub},'/',Subjects{sub},'_AVG_Data_40']);
    
    Error_dF = [Error_dF; TimeLessError_indv_fun(Nominal_dF.ParamTrack, S)];
    Error_MT = [Error_MT; TimeLessError_indv_fun(MT, S)];
    Error_MJJ = [Error_MJJ; TimeLessError_indv_fun(MJ_joint,S)];
    Error_MJ = [Error_MJ; TimeLessError_indv_fun(MJ, S)];
    
    sub
end

Errors = [sum(Error_dF,2), sum(Error_MT,2), sum(Error_MJ,2), sum(Error_MJJ,2)];

% [p,tbl,stats] = anova1(Errors);
% multcompare(stats)

labels = {'nom min dF', 'min dT','min jerk','joint jerk'}; 
Colors = ['r','y','g','b']; 

figure
hold on

for bar_num = 1:length(labels)
    h = bar( bar_num, mean(Errors(:,bar_num)) );
    set(h, 'FaceColor', Colors(bar_num))
    errorbar( bar_num, mean(Errors(:,bar_num)), std(Errors(:,bar_num)) );
end

title([Group, ' Time Less Error - sum 4 reaches'])
% legend(labels)


% make plots for different reaches seperately

figure
for i = 1:4
    subplot(2,2,i)
    hold on
    h = bar([mean(Error_dF(:,i)), mean(Error_MT(:,i)), mean(Error_MJ(:,i)), mean(Error_MJJ(:,i)) ]);
    errorbar([mean(Error_dF(:,i)), mean(Error_MT(:,i)),  mean(Error_MJ(:,i)), mean(Error_MJJ(:,i))],...
        [std(Error_dF(:,i)), std(Error_MT(:,i)), std(Error_MJ(:,i)), std(Error_MJJ(:,i))]./sqrt(num_subs) );
    xticks([1:4])
    xticklabels(labels)
    axis([0,5,0,0.025])
    grid on
    title('timeless RMS')
end


for i = 1:4
    
   anova1([Error_dF(:,i), Error_MT(:,i), Error_MJ(:,i), Error_MJJ(:,i)]) 
   
end
%% model comparison for individual subjects with std's - RMS - --> looks horrible!

% clear all
% close all
clc

% Define the Subjects

% Group = 'LimbFeedback'; 
% datadir = 'C:\GoogleDrive\Synced Folder\Experimental Data\NoForce4Targets'; 
% % datadir = 'C:\Users\Yeki\Google Drive\Synced Folder\Experimental Data\NoForce4Targets';
% Subjects = {'NF04', 'NF05', 'NF06', 'NF07', 'NF08', 'NF15', 'NF16', 'NF30', 'NF31', 'NF32'}; 

% Group = 'CursorFeedback';
% % datadir = 'C:\Users\Yeki\Google Drive\Synced Folder\Experimental Data\NoForce4Targets_control';
% datadir = 'C:\GoogleDrive\Synced Folder\Experimental Data\NoForce4Targets_control'; 
% % % % Subjects = {'NF20', 'NF21', 'NF22', 'NF23', 'NF24', 'NF25', 'NF26', 'NF27', 'NF33', 'NF34'};
% Subjects = {'NF20', 'NF21', 'NF22', 'NF23', 'NF24', 'NF25', 'NF26', 'NF33', 'NF34'};

Group = 'MixFeedback';
% datadir = 'C:\Users\Yeki\Google Drive\Synced Folder\Experimental Data\NoForce4Target_mix';
datadir = 'C:\GoogleDrive\Synced Folder\Experimental Data\NoForce4Target_mix';
Subjects = {'NF42', 'NF43', 'NF44', 'NF45', 'NF46', 'NF47', 'NF48', 'NF49', 'NF50', 'NF51'}; 

num_subs = length(Subjects);

Error_dF = [];
Error_MT = [];
Error_MJJ = [];
Error_MJ = [];

for sub = 1:num_subs
    
    S = load([datadir,'/',Subjects{sub},'/',Subjects{sub},'_AVG_Data_40']);
    
    Error_dF = [Error_dF; RMS_indv_fun(Nominal_dF.ParamTrack, S)];
    Error_MT = [Error_MT; RMS_indv_fun(MT, S)];
    Error_MJJ = [Error_MJJ; RMS_indv_fun(MJ_joint,S)];
    Error_MJ = [Error_MJ; RMS_indv_fun(MJ, S)];
    
    sub
end

Errors = [sum(Error_dF,2), sum(Error_MT,2), sum(Error_MJ,2), sum(Error_MJJ,2)];

labels = {'nom min dF', 'min dT','min jerk','joint jerk'}; 
Colors = ['r','y','g','b']; 

figure
hold on

for bar_num = 1:length(labels)
    h = bar( bar_num, mean(Errors(:,bar_num)) );
    set(h, 'FaceColor', Colors(bar_num))
    errorbar( bar_num, mean(Errors(:,bar_num)), std(Errors(:,bar_num)) );
end

title('RMS Error - sum 4 reaches')
% legend(labels)

%% Compute and plot bar plot for enclosed area with std

% compute the enclosed area (gap) between the data and the simulation

% close all

Targets0 = [-.1 -.1;
    0.1 -0.1;
    0.1 0.1;
    -0.1 0.1];

TargetsF = [Targets0(2:end,:);
    Targets0(1,:)];

clc

% Define the Subjects

% Group = 'LimbFeedback'; 
% datadir = 'C:\GoogleDrive\Synced Folder\Experimental Data\NoForce4Targets'; 
% % datadir = 'C:\Users\Yeki\Google Drive\Synced Folder\Experimental Data\NoForce4Targets';
% Subjects = {'NF04', 'NF05', 'NF06', 'NF07', 'NF08', 'NF15', 'NF16', 'NF30', 'NF31', 'NF32'}; 

% Group = 'CursorFeedback';
% % datadir = 'C:\Users\Yeki\Google Drive\Synced Folder\Experimental Data\NoForce4Targets_control';
% datadir = 'C:\GoogleDrive\Synced Folder\Experimental Data\NoForce4Targets_control'; 
% % % % Subjects = {'NF20', 'NF21', 'NF22', 'NF23', 'NF24', 'NF25', 'NF26', 'NF27', 'NF33', 'NF34'};
% Subjects = {'NF20', 'NF21', 'NF22', 'NF23', 'NF24', 'NF25', 'NF26', 'NF33', 'NF34'};

Group = 'MixFeedback';
% datadir = 'C:\Users\Yeki\Google Drive\Synced Folder\Experimental Data\NoForce4Target_mix';
datadir = 'C:\GoogleDrive\Synced Folder\Experimental Data\NoForce4Target_mix';
Subjects = {'NF42', 'NF43', 'NF44', 'NF45', 'NF46', 'NF47', 'NF48', 'NF49', 'NF50', 'NF51'}; 

num_subs = length(Subjects);

Error_dF = [];
Error_MT = [];
Error_MJJ = [];
Error_MJ = [];

for sub = 1:num_subs
    
    S = load([datadir,'/',Subjects{sub},'/',Subjects{sub},'_AVG_Data_40']);
    
    Error_dF = [Error_dF; GapArea_indv_fun(S,Nominal_dF.ParamTrack, Targets0, TargetsF)];
    Error_MT = [Error_MT; GapArea_indv_fun(S,MT, Targets0, TargetsF)];
    Error_MJJ = [Error_MJJ; GapArea_indv_fun(S,MJ_joint, Targets0, TargetsF)];
    Error_MJ = [Error_MJ; GapArea_indv_fun(S,MJ, Targets0, TargetsF)];
    
    sub
end

Errors = [sum(Error_dF,2), sum(Error_MT,2), sum(Error_MJ,2), sum(Error_MJJ,2)];

labels = {'nom min dF', 'min dT','min jerk','joint jerk'}; 
Colors = ['r','y','g','b']; 

figure
hold on

for bar_num = 1:length(labels)
    h = bar( bar_num, mean(Errors(:,bar_num)) );
    set(h, 'FaceColor', Colors(bar_num))
    errorbar( bar_num, mean(Errors(:,bar_num)), std(Errors(:,bar_num)) );
end

title([ Group, 'Enclosed Area with Data - sum 4 reaches'])

% [p,tbl,stats] = anova1(Errors);
% multcompare(stats)

for i = 1:4
    
   anova1([Error_dF(:,i), Error_MT(:,i), Error_MJ(:,i), Error_MJJ(:,i)]) 
   
end

%% Compute and plot bar plot for PE with std

clc

% Define the Subjects

% Group = 'LimbFeedback'; 
% datadir = 'C:\GoogleDrive\Synced Folder\Experimental Data\NoForce4Targets'; 
% % datadir = 'C:\Users\Yeki\Google Drive\Synced Folder\Experimental Data\NoForce4Targets';
% Subjects = {'NF04', 'NF05', 'NF06', 'NF07', 'NF08', 'NF15', 'NF16', 'NF30', 'NF31', 'NF32'}; 

% Group = 'CursorFeedback';
% % datadir = 'C:\Users\Yeki\Google Drive\Synced Folder\Experimental Data\NoForce4Targets_control';
% datadir = 'C:\GoogleDrive\Synced Folder\Experimental Data\NoForce4Targets_control'; 
% % % % Subjects = {'NF20', 'NF21', 'NF22', 'NF23', 'NF24', 'NF25', 'NF26', 'NF27', 'NF33', 'NF34'};
% Subjects = {'NF20', 'NF21', 'NF22', 'NF23', 'NF24', 'NF25', 'NF26', 'NF33', 'NF34'};

Group = 'MixFeedback';
% datadir = 'C:\Users\Yeki\Google Drive\Synced Folder\Experimental Data\NoForce4Target_mix';
datadir = 'C:\GoogleDrive\Synced Folder\Experimental Data\NoForce4Target_mix';
Subjects = {'NF42', 'NF43', 'NF44', 'NF45', 'NF46', 'NF47', 'NF48', 'NF49', 'NF50', 'NF51'}; 

num_subs = length(Subjects);

Error_dF = [];
Error_MT = [];
Error_MJJ = [];
Error_MJ = [];
Error_S = [];

for sub = 1:num_subs
    
    S = load([datadir,'/',Subjects{sub},'/',Subjects{sub},'_AVG_Data_40']);
    
    Error_S = [Error_S; PerpError_data_indv_fun(S)];
    
    Error_dF = [Error_dF; abs(PerpError_fun(Nominal_dF.ParamTrack)-PerpError_data_indv_fun(S))];
    Error_MT = [Error_MT; abs(PerpError_fun(MT)-PerpError_data_indv_fun(S))];
    Error_MJJ = [Error_MJJ; abs(PerpError_fun(MJ_joint)-PerpError_data_indv_fun(S))];
    Error_MJ = [Error_MJ; abs(PerpError_fun(MJ)-PerpError_data_indv_fun(S))];
    
    sub
end

Errors = [sum(Error_dF,2), sum(Error_MT,2), sum(Error_MJ,2), sum(Error_MJJ,2)];

labels = {'nom min dF', 'min dT','min jerk','joint jerk'}; 
Colors = ['r','y','g','b']; 

figure
hold on

for bar_num = 1:length(labels)
    h = bar( bar_num, mean(Errors(:,bar_num)) );
    set(h, 'FaceColor', Colors(bar_num))
    errorbar( bar_num, mean(Errors(:,bar_num)), std(Errors(:,bar_num)) );
end

title([ Group, ' PE - sum 4 reaches'])

% [p,tbl,stats] = anova1(Errors);
% multcompare(stats)

for i = 1:4
    
   anova1([Error_dF(:,i), Error_MT(:,i), Error_MJ(:,i), Error_MJJ(:,i)]) 
   
end


%% Compute and plot bar plot for Normalized Path Length with std

clc

% Define the Subjects
% 
% Group = 'LimbFeedback'; 
% datadir = 'C:\GoogleDrive\Synced Folder\Experimental Data\NoForce4Targets'; 
% datadir = 'C:\Users\Yeki\Google Drive\Synced Folder\Experimental Data\NoForce4Targets';
% Subjects = {'NF04', 'NF05', 'NF06', 'NF07', 'NF08', 'NF15', 'NF16', 'NF30', 'NF31', 'NF32'}; 

% Group = 'CursorFeedback';
% % datadir = 'C:\Users\Yeki\Google Drive\Synced Folder\Experimental Data\NoForce4Targets_control';
% datadir = 'C:\GoogleDrive\Synced Folder\Experimental Data\NoForce4Targets_control'; 
% % % % Subjects = {'NF20', 'NF21', 'NF22', 'NF23', 'NF24', 'NF25', 'NF26', 'NF27', 'NF33', 'NF34'};
% Subjects = {'NF20', 'NF21', 'NF22', 'NF23', 'NF24', 'NF25', 'NF26', 'NF33', 'NF34'};

Group = 'MixFeedback';
% datadir = 'C:\Users\Yeki\Google Drive\Synced Folder\Experimental Data\NoForce4Target_mix';
datadir = 'C:\GoogleDrive\Synced Folder\Experimental Data\NoForce4Target_mix';
Subjects = {'NF42', 'NF43', 'NF44', 'NF45', 'NF46', 'NF47', 'NF48', 'NF49', 'NF50', 'NF51'}; 

num_subs = length(Subjects);

Error_dF = [];
Error_MT = [];
Error_MJJ = [];
Error_MJ = [];
Error_S = [];

for sub = 1:num_subs
    
    S = load([datadir,'/',Subjects{sub},'/',Subjects{sub},'_AVG_Data_40']);
    
    Error_S = [Error_S; NPathLength_indv_fun(S)];
    
    Error_dF = [Error_dF; abs(NPathLength_fun(Nominal_dF.ParamTrack)-NPathLength_indv_fun(S))];
    Error_MT = [Error_MT; abs(NPathLength_fun(MT)-NPathLength_indv_fun(S))];
    Error_MJJ = [Error_MJJ; abs(NPathLength_fun(MJ_joint)-NPathLength_indv_fun(S))];
    Error_MJ = [Error_MJ; abs(NPathLength_fun(MJ)-NPathLength_indv_fun(S))];
    
    sub
end

Errors = [sum(Error_dF,2), sum(Error_MT,2), sum(Error_MJ,2), sum(Error_MJJ,2)];

labels = {'nom min dF', 'min dT','min jerk','joint jerk'}; 
Colors = ['r','y','g','b']; 

figure
hold on

for bar_num = 1:length(labels)
    h = bar( bar_num, mean(Errors(:,bar_num)) );
    set(h, 'FaceColor', Colors(bar_num))
    errorbar( bar_num, mean(Errors(:,bar_num)), std(Errors(:,bar_num)) );
end

title([ Group, ' PL - sum 4 reaches'])

[p,tbl,stats] = anova1(Errors);
multcompare(stats)

for i = 1:4
    
   anova1([Error_dF(:,i), Error_MT(:,i), Error_MJ(:,i), Error_MJJ(:,i)]) 
   
end

%% make a plot of all 4 metrics together

figure

subplot(2,2,1)
hold on
for bar_num = 1:length(labels)
    h = bar( bar_num, sum(Errors_rms(bar_num,:)) );
    set(h, 'FaceColor', Colors(bar_num))
end
title('RMS path')
axis([0,5,0,5])
%
%
subplot(2,2,2)
hold on
for bar_num = 1:length(labels)
    sum(temp_PE(bar_num,:));
    h = bar( bar_num, sum(temp_PE(bar_num,:)) );
    set(h, 'FaceColor', Colors(bar_num))
end
title('PE')
axis([0,5,0,0.1])
%
%
subplot(2,2,3)
hold on
for bar_num = 1:length(labels)
    abs(sum(NL_Data) - NL(bar_num));
    h = bar( bar_num, abs(sum(NL_Data) - NL(bar_num)));
    set(h, 'FaceColor', Colors(bar_num))
end
title('NL')
axis([0,5,0,0.15])
%
%
subplot(2,2,4)
hold on
for bar_num = 1:length(labels)
    h = bar( bar_num, gap(bar_num));
    set(h, 'FaceColor', Colors(bar_num))
end
axis([0,length(labels)+1, 0, 1.5*max( gap) ])
title('EA')
axis([0,5,0,0.03])

%% compute the best fit for each subject

% clc

% Define the Subjects

% Group = 'LimbFeedback'; 
% datadir = 'C:\GoogleDrive\Synced Folder\Experimental Data\NoForce4Targets'; 
% % datadir = 'C:\Users\Yeki\Google Drive\Synced Folder\Experimental Data\NoForce4Targets';
% Subjects = {'NF04', 'NF05', 'NF06', 'NF07', 'NF08', 'NF15', 'NF16', 'NF30', 'NF31', 'NF32'}; 

Group = 'CursorFeedback';
% datadir = 'C:\Users\Yeki\Google Drive\Synced Folder\Experimental Data\NoForce4Targets_control';
datadir = 'C:\GoogleDrive\Synced Folder\Experimental Data\NoForce4Targets_control'; 
% % % Subjects = {'NF20', 'NF21', 'NF22', 'NF23', 'NF24', 'NF25', 'NF26', 'NF27', 'NF33', 'NF34'};
Subjects = {'NF20', 'NF21', 'NF22', 'NF23', 'NF24', 'NF25', 'NF26', 'NF33', 'NF34'};

% Group = 'MixFeedback';
% % datadir = 'C:\Users\Yeki\Google Drive\Synced Folder\Experimental Data\NoForce4Target_mix';
% datadir = 'C:\GoogleDrive\Synced Folder\Experimental Data\NoForce4Target_mix';
% Subjects = {'NF42', 'NF43', 'NF44', 'NF45', 'NF46', 'NF47', 'NF48', 'NF49', 'NF50', 'NF51'}; 

num_subs = length(Subjects);

Error_dF = [];
Error_MT = [];
Error_MJJ = [];
Error_MJ = [];

% %%%%%%%%% TIME LESS RMS
% for sub = 1:num_subs
%     
%     S = load([datadir,'/',Subjects{sub},'/',Subjects{sub},'_AVG_Data_40']);
%     
%     Error_dF = [Error_dF; TimeLessError_indv_fun(Nominal_dF.ParamTrack, S)];
%     Error_MT = [Error_MT; TimeLessError_indv_fun(MT, S)];
%     Error_MJJ = [Error_MJJ; TimeLessError_indv_fun(MJ_joint,S)];
%     Error_MJ = [Error_MJ; TimeLessError_indv_fun(MJ, S)];
%     
%     sub
% end

% %%%%%%%% SLIDING RMS
% for sub = 1:num_subs
%     
%     S = load([datadir,'/',Subjects{sub},'/',Subjects{sub},'_AVG_Data_40']);
%     
%     Error_dF = [Error_dF; RMS_Sliding_indv_fun(Nominal_dF.ParamTrack, S)];
%     Error_MT = [Error_MT; RMS_Sliding_indv_fun(MT, S)];
%     Error_MJJ = [Error_MJJ; RMS_Sliding_indv_fun(MJ_joint,S)];
%     Error_MJ = [Error_MJ; RMS_Sliding_indv_fun(MJ, S)];
%     
%     sub
% end

% %%%%%%%% PERP ERROR
% for sub = 1:num_subs
%     
%     S = load([datadir,'/',Subjects{sub},'/',Subjects{sub},'_AVG_Data_40']);
%     
%     Error_dF = [Error_dF; abs(PerpError_fun(Nominal_dF.ParamTrack)-PerpError_data_indv_fun(S))];
%     Error_MT = [Error_MT; abs(PerpError_fun(MT)-PerpError_data_indv_fun(S))];
%     Error_MJJ = [Error_MJJ; abs(PerpError_fun(MJ_joint)-PerpError_data_indv_fun(S))];
%     Error_MJ = [Error_MJ; abs(PerpError_fun(MJ)-PerpError_data_indv_fun(S))];
%     
%     sub
% end

% %%%%%%%% EA
% for sub = 1:num_subs
%     
%     S = load([datadir,'/',Subjects{sub},'/',Subjects{sub},'_AVG_Data_40']);
%     
%     Error_dF = [Error_dF; abs(PerpError_fun(Nominal_dF.ParamTrack)-PerpError_data_indv_fun(S))];
%     Error_MT = [Error_MT; abs(PerpError_fun(MT)-PerpError_data_indv_fun(S))];
%     Error_MJJ = [Error_MJJ; abs(PerpError_fun(MJ_joint)-PerpError_data_indv_fun(S))];
%     Error_MJ = [Error_MJ; abs(PerpError_fun(MJ)-PerpError_data_indv_fun(S))];
%     
%     sub
% end

%%%%%%%% PL
for sub = 1:num_subs
    
    S = load([datadir,'/',Subjects{sub},'/',Subjects{sub},'_AVG_Data_40']);
    
    Error_dF = [Error_dF; abs(NPathLength_fun(Nominal_dF.ParamTrack)-NPathLength_indv_fun(S))];
    Error_MT = [Error_MT; abs(NPathLength_fun(MT)-NPathLength_indv_fun(S))];
    Error_MJJ = [Error_MJJ; abs(NPathLength_fun(MJ_joint)-NPathLength_indv_fun(S))];
    Error_MJ = [Error_MJ; abs(NPathLength_fun(MJ)-NPathLength_indv_fun(S))];
    
    sub
end

Errors = [sum(Error_dF,2), sum(Error_MT,2), sum(Error_MJ,2), sum(Error_MJJ,2)];

[a,b] = min(Errors');

display(b)

% Errors_reach1 = [Error_dF(:,1), Error_MT(:,1), Error_MJ(:,1), Error_MJJ(:,1)];
% Errors_reach2 = [Error_dF(:,2), Error_MT(:,2), Error_MJ(:,2), Error_MJJ(:,2)];
% Errors_reach3 = [Error_dF(:,3), Error_MT(:,3), Error_MJ(:,3), Error_MJJ(:,3)];
% Errors_reach4 = [Error_dF(:,4), Error_MT(:,4), Error_MJ(:,4), Error_MJJ(:,4)];
% 
% [a,b1] = min(Errors_reach1');
% [a,b2] = min(Errors_reach2');
% [a,b3] = min(Errors_reach3');
% [a,b4] = min(Errors_reach4');
% 
% display(b1)
% display(b2)
% display(b3)
% display(b4)

%% slide the data and the simulations to find the least RMS error (Sliding RMS)


% clear all
% close all
% clc

% Define the Subjects

% Group = 'LimbFeedback'; 
% datadir = 'C:\GoogleDrive\Synced Folder\Experimental Data\NoForce4Targets'; 
% % datadir = 'C:\Users\Yeki\Google Drive\Synced Folder\Experimental Data\NoForce4Targets';
% Subjects = {'NF04', 'NF05', 'NF06', 'NF07', 'NF08', 'NF15', 'NF16', 'NF30', 'NF31', 'NF32'}; 

% Group = 'CursorFeedback';
% % datadir = 'C:\Users\Yeki\Google Drive\Synced Folder\Experimental Data\NoForce4Targets_control';
% datadir = 'C:\GoogleDrive\Synced Folder\Experimental Data\NoForce4Targets_control'; 
% % % % Subjects = {'NF20', 'NF21', 'NF22', 'NF23', 'NF24', 'NF25', 'NF26', 'NF27', 'NF33', 'NF34'};
% Subjects = {'NF20', 'NF21', 'NF22', 'NF23', 'NF24', 'NF25', 'NF26', 'NF33', 'NF34'};

Group = 'MixFeedback';
% datadir = 'C:\Users\Yeki\Google Drive\Synced Folder\Experimental Data\NoForce4Target_mix';
datadir = 'C:\GoogleDrive\Synced Folder\Experimental Data\NoForce4Target_mix';
Subjects = {'NF42', 'NF43', 'NF44', 'NF45', 'NF46', 'NF47', 'NF48', 'NF49', 'NF50', 'NF51'}; 

num_subs = length(Subjects);

Error_dF = [];
Error_MT = [];
Error_MJJ = [];
Error_MJ = [];

figure
hold on

for sub = 1:num_subs
    
    S = load([datadir,'/',Subjects{sub},'/',Subjects{sub},'_AVG_Data_40']);
    
    Error_dF = [Error_dF; RMS_Sliding_indv_fun(Nominal_dF.ParamTrack, S)];
    Error_MT = [Error_MT; RMS_Sliding_indv_fun(MT, S)];
    Error_MJJ = [Error_MJJ; RMS_Sliding_indv_fun(MJ_joint,S)];
    Error_MJ = [Error_MJ; RMS_Sliding_indv_fun(MJ, S)];
    
    sub
end

Errors = [sum(Error_dF,2), sum(Error_MT,2), sum(Error_MJ,2), sum(Error_MJJ,2)];

labels = {'nom min dF', 'min dT','min jerk','joint jerk'}; 
Colors = ['r','y','g','b']; 

figure
hold on

for bar_num = 1:length(labels)
    h = bar( bar_num, mean(Errors(:,bar_num)) );
    set(h, 'FaceColor', Colors(bar_num))
    errorbar( bar_num, mean(Errors(:,bar_num)), std(Errors(:,bar_num)) );
end

title([ Group, ' Sliding RMS Error - sum 4 reaches'])
axis([0, 5, 0, 0.12])

% make plots for different reaches seperately

figure
for i = 1:4
    subplot(2,2,i)
    hold on
    h = bar([mean(Error_dF(:,i)), mean(Error_MT(:,i)), mean(Error_MJ(:,i)), mean(Error_MJJ(:,i)) ]);
    errorbar([mean(Error_dF(:,i)), mean(Error_MT(:,i)),  mean(Error_MJ(:,i)), mean(Error_MJJ(:,i))],...
        [std(Error_dF(:,i)), std(Error_MT(:,i)), std(Error_MJ(:,i)), std(Error_MJJ(:,i))]./sqrt(num_subs) );
    xticks([1:4])
    xticklabels(labels)
    axis([0,5,0,0.035])
    grid on
    title('sliding RMS error')
end

%% Sliding RMS for average trajectories


Error_dF = RMS_Sliding_fun(Nominal_dF.ParamTrack, S, 1);
Error_MT = RMS_Sliding_fun(MT, S, 1);
Error_MJJ = RMS_Sliding_fun(MJ_joint, S, 1);
Error_MJ = RMS_Sliding_fun(MJ, S, 1);

Errors_rms = [Error_MJ; Error_MJJ; Error_MT; Error_dF];

figure

labels = {'min jerk','joint jerk', 'min dT','nom min dF'}; 
Colors = ['g','b','y','r']; 

for i = 1:4
    
    e = Errors_rms(:,i); %first reach for all models
    
    subplot(2,2,i)
    hold on
    
    for bar_num = 1:length(labels)
        h = bar( bar_num, e(bar_num));
        set(h, 'FaceColor', Colors(bar_num))
    end
    
    title('Time Less Error')
%     axis([0,length(labels)+1, 0, 1.2*0.0879 ])
    i
end

legend(labels)

%%%%%%%%%%%%%%%%%%%%%%%% one plot for all reaches

figure
hold on

for bar_num = 1:length(labels)
    h = bar( bar_num, sum(Errors_rms(bar_num,:)) );
    set(h, 'FaceColor', Colors(bar_num))
end

title([ Group, ' Sliding RMS - sum 4 reaches'])
legend(labels)

%% Sliding RMS for vel profiles (Vel_RMS)

% Define the Subjects

Group = 'LimbFeedback'; 
datadir = 'C:\GoogleDrive\Synced Folder\Experimental Data\NoForce4Targets'; 
% datadir = 'C:\Users\Yeki\Google Drive\Synced Folder\Experimental Data\NoForce4Targets';
Subjects = {'NF04', 'NF05', 'NF06', 'NF07', 'NF08', 'NF15', 'NF16', 'NF30', 'NF31', 'NF32'}; 

% Group = 'CursorFeedback';
% % datadir = 'C:\Users\Yeki\Google Drive\Synced Folder\Experimental Data\NoForce4Targets_control';
% datadir = 'C:\GoogleDrive\Synced Folder\Experimental Data\NoForce4Targets_control'; 
% % % % Subjects = {'NF20', 'NF21', 'NF22', 'NF23', 'NF24', 'NF25', 'NF26', 'NF27', 'NF33', 'NF34'};
% Subjects = {'NF20', 'NF21', 'NF22', 'NF23', 'NF24', 'NF25', 'NF26', 'NF33', 'NF34'};

% Group = 'MixFeedback';
% % datadir = 'C:\Users\Yeki\Google Drive\Synced Folder\Experimental Data\NoForce4Target_mix';
% datadir = 'C:\GoogleDrive\Synced Folder\Experimental Data\NoForce4Target_mix';
% Subjects = {'NF42', 'NF43', 'NF44', 'NF45', 'NF46', 'NF47', 'NF48', 'NF49', 'NF50', 'NF51'}; 

num_subs = length(Subjects);

Error_dF = [];
Error_MT = [];
Error_MJJ = [];
Error_MJ = [];

figure
hold on

for sub = 1:num_subs
    
    S = load([datadir,'/',Subjects{sub},'/',Subjects{sub},'_AVG_Vel']); % change to vel average
    
    Error_dF = [Error_dF; RMS_Vel_Sliding_indv_fun(Nominal_dF.ParamTrack, S)];
    Error_MT = [Error_MT; RMS_Vel_Sliding_indv_fun(MT, S)];
    Error_MJJ = [Error_MJJ; RMS_Vel_Sliding_indv_fun(MJ_joint,S)];
    Error_MJ = [Error_MJ; RMS_Vel_Sliding_indv_fun(MJ, S)];
    
    sub
end

Errors = [sum(Error_dF,2), sum(Error_MT,2), sum(Error_MJ,2), sum(Error_MJJ,2)];

labels = {'nom min dF', 'min dT','min jerk','joint jerk'}; 
Colors = ['r','y','g','b']; 

figure
hold on

for bar_num = 1:length(labels)
    h = bar( bar_num, mean(Errors(:,bar_num)) );
    set(h, 'FaceColor', Colors(bar_num))
    errorbar( bar_num, mean(Errors(:,bar_num)), std(Errors(:,bar_num)) );
end

title([ Group, ' Sliding Vel RMS - sum 4 reaches'])
axis([0, 5, 0, 0.45])

% make plots for different reaches seperately

figure
for i = 1:4
    subplot(2,2,i)
    hold on
    h = bar([mean(Error_dF(:,i)), mean(Error_MT(:,i)), mean(Error_MJ(:,i)), mean(Error_MJJ(:,i)) ]);
    errorbar([mean(Error_dF(:,i)), mean(Error_MT(:,i)),  mean(Error_MJ(:,i)), mean(Error_MJJ(:,i))],...
        [std(Error_dF(:,i)), std(Error_MT(:,i)), std(Error_MJ(:,i)), std(Error_MJJ(:,i))]./sqrt(num_subs) );
    xticks([1:4])
    xticklabels(labels)
    axis([0,5,0,0.14])
    grid on
    title('sliding Vel RMS error')
end

%% THE END


