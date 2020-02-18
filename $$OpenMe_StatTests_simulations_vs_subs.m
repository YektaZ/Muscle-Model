

% in this script I compute the error metric between each subject and it's
% own model prediction. 
% each subject's model prediction means the prediction that is computed
% based on the parameters from that subject. 

% I run a one way anova test to see which model wins.
% I repeat that for all 4 metrics.

% for cursor feedback group see Stattests_avg_simulations

clear all
clc

global sho_x sho_y

sho_x = 0;
sho_y = -0.4;

% addpath('C:\GoogleDrive\Synced Folder\Matlab\Simple Two-Link Arm')
% addpath('C:\Users\Yeki\Google Drive\Synced Folder\Matlab\Simple Two-Link Arm');
addpath('/Users/yektazahed/Google Drive/Synced Folder/Matlab/Simple Two-Link Arm')


dF_20 = load('ParamTrack_20subs.mat');
dT_20 = load('TR_20subs.mat');
MJJ_20 = load('MJJ_20subs.mat');


%% Generate min jerk solution
% Generate min jerk solution in joint space

% addpath('C:\Users\Yeki\Google Drive\Synced Folder\Matlab\Minimum Jerk Functions');
% addpath('C:\GoogleDrive\Synced Folder\Matlab\Minimum Jerk Functions');
addpath('/Users/yektazahed/Google Drive/Synced Folder/Matlab/Minimum Jerk Functions');

N = length(dT_20.MT{1,1}.Sol.X);
t = dT_20.MT{1,1}.Sol.t;

for counter = 1:4
    %
    Xo = dT_20.MT{counter,1}.Sol.X(1,1:2);
    Xf = dT_20.MT{counter,1}.Sol.X(end,1:2);
    %
    for i = 1:N
        X_mj{counter}(:,i) = min_Jerk_Position( t(i), t(1), t(end), Xo, Xf );
        dX_mj{counter}(:,i) = min_Jerk_Velocity( t(i), t(1), t(end), Xo, Xf );
    end
    
    MJ{counter}.Sol.X = [X_mj{counter}; dX_mj{counter}]';
    MJ{counter}.Sol.t = t;
end

%% Timeless errors for individual subjects

clc

% Define the Subjects

% Group = 'LimbFeedback'; 
% % datadir = 'C:\GoogleDrive\Synced Folder\Experimental Data\NoForce4Targets'; 
% datadir = '/Users/yektazahed/Google Drive/Synced Folder/Experimental Data/NoForce4Targets';
% % datadir = 'C:\Users\Yeki\Google Drive\Synced Folder\Experimental Data\NoForce4Targets';
% Subjects = {'NF04', 'NF05', 'NF06', 'NF07', 'NF08', 'NF15', 'NF16', 'NF30', 'NF31', 'NF32'};

Group = 'MixedFeedback'; 
% datadir = 'C:\GoogleDrive\Synced Folder\Experimental Data\NoForce4Target_mix';
datadir = '/Users/yektazahed/Google Drive/Synced Folder/Experimental Data/NoForce4Target_mix';
Subjects =  {'NF42', 'NF43', 'NF44', 'NF45', 'NF46', 'NF47', 'NF48', 'NF49', 'NF50', 'NF51'}; 

num_subs = length(Subjects);

Error_dF = [];
Error_MT = [];
Error_MJJ = [];
Error_MJ = [];

for sub = 11:(20)
% for sub = 1:(10)
    
    FR = {dF_20.ParamTrack{sub}, dF_20.ParamTrack{2*num_subs+sub},...
        dF_20.ParamTrack{4*num_subs+sub}, dF_20.ParamTrack{6*num_subs+sub}}; % this should be change for the group
    TR = {dT_20.MT{:,sub}};
    MJJ = {MJJ_20.MJ_joint{sub,:}};
    
    S = load([datadir,'/',Subjects{sub-10},'/',Subjects{sub-10},'_AVG_Data_40']);
    
    Error_dF = [Error_dF; TimeLessError_indv_fun(FR, S)];
    Error_MT = [Error_MT; TimeLessError_indv_fun(TR, S)];
    Error_MJJ = [Error_MJJ; TimeLessError_indv_fun(MJJ,S)];
    Error_MJ = [Error_MJ; TimeLessError_indv_fun(MJ, S)];
    
    sub
end

Errors = [sum(Error_dF,2), sum(Error_MT,2), sum(Error_MJ,2), sum(Error_MJJ,2)];

[p,tbl,stats] = anova1(Errors);
multcompare(stats)

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


%% EA --> Compute and plot bar plot for enclosed area with std

% compute the enclosed area (gap) between the data and the simulation

% close all
clc

% datadir = 'C:\GoogleDrive\Synced Folder\Experimental Data\NoForce4Targets'; 

Targets0 = [-.1 -.1;
    0.1 -0.1;
    0.1 0.1;
    -0.1 0.1];

TargetsF = [Targets0(2:end,:);
    Targets0(1,:)];

Error_dF = [];
Error_MT = [];
Error_MJJ = [];
Error_MJ = [];

% for sub = 1:num_subs
for sub = 11:(num_subs+10)
    
%     if sub>10
%         datadir = datadir2;
%     end
    
    FR = {dF_20.ParamTrack{sub}, dF_20.ParamTrack{2*num_subs+sub},...
        dF_20.ParamTrack{4*num_subs+sub}, dF_20.ParamTrack{6*num_subs+sub}};
    TR = {dT_20.MT{:,sub}};
    MJJ = {MJJ_20.MJ_joint{sub,:}};
    
    S = load([datadir,'/',Subjects{sub-10},'/',Subjects{sub-10},'_AVG_Data_40']);
    
    Error_dF = [Error_dF; GapArea_indv_fun(S,FR,Targets0,TargetsF)];
    Error_MT = [Error_MT; GapArea_indv_fun(S,TR,Targets0,TargetsF)];
    Error_MJJ = [Error_MJJ; GapArea_indv_fun(S,MJJ,Targets0,TargetsF)];
    Error_MJ = [Error_MJ; GapArea_indv_fun(S,MJ,Targets0,TargetsF)];
    
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

[p,tbl,stats] = anova1(Errors);
multcompare(stats)


%% PE

clc

% datadir = 'C:\GoogleDrive\Synced Folder\Experimental Data\NoForce4Targets'; 

num_subs = length(Subjects);

Error_dF = [];
Error_MT = [];
Error_MJJ = [];
Error_MJ = [];
Error_S = [];

% for sub = 1:num_subs
for sub = 11:(num_subs+10)
    
%     if sub>10
%         datadir = datadir2;
%     end
    
    FR = {dF_20.ParamTrack{sub}, dF_20.ParamTrack{2*num_subs+sub},...
        dF_20.ParamTrack{4*num_subs+sub}, dF_20.ParamTrack{6*num_subs+sub}};
    TR = {dT_20.MT{:,sub}};
    MJJ = {MJJ_20.MJ_joint{sub,:}};
    
    S = load([datadir,'/',Subjects{sub-10},'/',Subjects{sub-10},'_AVG_Data_40']);
    
    Error_S = [Error_S; PerpError_data_indv_fun(S)];
    
    Error_dF = [Error_dF; abs(PerpError_fun(FR)-PerpError_data_indv_fun(S))];
    Error_MT = [Error_MT; abs(PerpError_fun(TR)-PerpError_data_indv_fun(S))];
    Error_MJJ = [Error_MJJ; abs(PerpError_fun(MJJ)-PerpError_data_indv_fun(S))];
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

[p,tbl,stats] = anova1(Errors);
multcompare(stats)


%% PL

clc

% datadir = 'C:\GoogleDrive\Synced Folder\Experimental Data\NoForce4Targets'; 

num_subs = length(Subjects);

Error_dF = [];
Error_MT = [];
Error_MJJ = [];
Error_MJ = [];
Error_S = [];

% for sub = 1:num_subs
for sub = 11:(num_subs+10)
    
%      if sub>10
%         datadir = datadir2;
%     end
    
    FR = {dF_20.ParamTrack{sub}, dF_20.ParamTrack{2*num_subs+sub},...
        dF_20.ParamTrack{4*num_subs+sub}, dF_20.ParamTrack{6*num_subs+sub}};
    TR = {dT_20.MT{:,sub}};
    MJJ = {MJJ_20.MJ_joint{sub,:}};
    
    S = load([datadir,'/',Subjects{sub-10},'/',Subjects{sub-10},'_AVG_Data_40']);
    
    Error_S = [Error_S; NPathLength_indv_fun(S)];
    
    Error_dF = [Error_dF; abs(NPathLength_fun(FR)-NPathLength_indv_fun(S))];
    Error_MT = [Error_MT; abs(NPathLength_fun(TR)-NPathLength_indv_fun(S))];
    Error_MJJ = [Error_MJJ; abs(NPathLength_fun(MJJ)-NPathLength_indv_fun(S))];
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


%%


% THE END

