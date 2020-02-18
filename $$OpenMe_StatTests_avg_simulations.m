
% for the cursor FB group since I don't have any measurements, I will
% compare them with the average of all other groups. 

clear all
clc

global sho_x sho_y

sho_x = 0;
sho_y = -0.4;

% addpath('C:\GoogleDrive\Synced Folder\Matlab\Simple Two-Link Arm')
% addpath('C:\Users\Yeki\Google Drive\Synced Folder\Matlab\Simple Two-Link Arm');
addpath('/Users/yektazahed/Google Drive/Synced Folder/Matlab/Simple Two-Link Arm')


dF_avg20 = load('FR_AVG_20subs.mat');
dT_avg20 = load('TR_AVG_20subs.mat');
MJJ_avg20 = load('MJJ_AVG_20subs.mat');


%% Generate min jerk solution
% Generate min jerk solution in joint space

% addpath('C:\Users\Yeki\Google Drive\Synced Folder\Matlab\Minimum Jerk Functions');
% addpath('C:\GoogleDrive\Synced Folder\Matlab\Minimum Jerk Functions');
addpath('/Users/yektazahed/Google Drive/Synced Folder/Matlab/Minimum Jerk Functions');

N = length(dF_avg20.FR_AVG{1}.Sol.X);
t = dF_avg20.FR_AVG{1}.Sol.t;

for counter = 1:4
    %
    Xo = dF_avg20.FR_AVG{counter}.Sol.X(1,1:2);
    Xf = dF_avg20.FR_AVG{counter}.Sol.X(end,1:2);
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

Group = 'CursorFeedback';
% datadir = 'C:\Users\Yeki\Google Drive\Synced Folder\Experimental Data\NoForce4Targets_control';
% datadir = 'C:\GoogleDrive\Synced Folder\Experimental Data\NoForce4Targets_control'; 
datadir = '/Users/yektazahed/Google Drive/Synced Folder/Experimental Data/NoForce4Targets_control';
% % % Subjects = {'NF20', 'NF21', 'NF22', 'NF23', 'NF24', 'NF25', 'NF26', 'NF27', 'NF33', 'NF34'};
Subjects = {'NF20', 'NF21', 'NF22', 'NF23', 'NF24', 'NF25', 'NF26', 'NF33', 'NF34'};


num_subs = length(Subjects);

Error_dF = [];
Error_MT = [];
Error_MJJ = [];
Error_MJ = [];

for sub = 1:num_subs
    
    S = load([datadir,'/',Subjects{sub},'/',Subjects{sub},'_AVG_Data_40']);
    
    Error_dF = [Error_dF; TimeLessError_indv_fun(dF_avg20.FR_AVG, S)];
    Error_MT = [Error_MT; TimeLessError_indv_fun(dT_avg20.TR_AVG, S)];
    Error_MJJ = [Error_MJJ; TimeLessError_indv_fun(MJJ_avg20.MJJ_AVG,S)];
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

Group = 'CursorFeedback';
% datadir = 'C:\Users\Yeki\Google Drive\Synced Folder\Experimental Data\NoForce4Targets_control';
datadir = 'C:\GoogleDrive\Synced Folder\Experimental Data\NoForce4Targets_control'; 
% % % Subjects = {'NF20', 'NF21', 'NF22', 'NF23', 'NF24', 'NF25', 'NF26', 'NF27', 'NF33', 'NF34'};
Subjects = {'NF20', 'NF21', 'NF22', 'NF23', 'NF24', 'NF25', 'NF26', 'NF33', 'NF34'};

% Group = 'MixFeedback';
% datadir = 'C:\Users\Yeki\Google Drive\Synced Folder\Experimental Data\NoForce4Target_mix';
% datadir = 'C:\GoogleDrive\Synced Folder\Experimental Data\NoForce4Target_mix';
% Subjects = {'NF42', 'NF43', 'NF44', 'NF45', 'NF46', 'NF47', 'NF48', 'NF49', 'NF50', 'NF51'}; 

num_subs = length(Subjects);

Error_dF = [];
Error_MT = [];
Error_MJJ = [];
Error_MJ = [];

for sub = 1:num_subs
    
    S = load([datadir,'/',Subjects{sub},'/',Subjects{sub},'_AVG_Data_40']);
    
    Error_dF = [Error_dF; GapArea_indv_fun(S,dF_avg20.FR_AVG,Targets0,TargetsF)];
    Error_MT = [Error_MT; GapArea_indv_fun(S,dT_avg20.TR_AVG,Targets0,TargetsF)];
    Error_MJJ = [Error_MJJ; GapArea_indv_fun(S,MJJ_avg20.MJJ_AVG,Targets0,TargetsF)];
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

%% PE for individuals
clc

% Define the Subjects
% 
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
Error_S = [];

for sub = 1:num_subs
    
    S = load([datadir,'/',Subjects{sub},'/',Subjects{sub},'_AVG_Data_40']);
    
    Error_S = [Error_S; PerpError_data_indv_fun(S)];
    
    Error_dF = [Error_dF; abs(PerpError_fun(dF_avg20.FR_AVG)-PerpError_data_indv_fun(S))];
    Error_MT = [Error_MT; abs(PerpError_fun(dT_avg20.TR_AVG)-PerpError_data_indv_fun(S))];
    Error_MJJ = [Error_MJJ; abs(PerpError_fun(MJJ_avg20.MJJ_AVG)-PerpError_data_indv_fun(S))];
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

%% PL for individual subjects
clc

% Define the Subjects
% 
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
Error_S = [];

for sub = 1:num_subs
    
    S = load([datadir,'/',Subjects{sub},'/',Subjects{sub},'_AVG_Data_40']);
    
    Error_S = [Error_S; NPathLength_indv_fun(S)];
    
    Error_dF = [Error_dF; abs(NPathLength_fun(dF_avg20.FR_AVG)-NPathLength_indv_fun(S))];
    Error_MT = [Error_MT; abs(NPathLength_fun(dT_avg20.TR_AVG)-NPathLength_indv_fun(S))];
    Error_MJJ = [Error_MJJ; abs(NPathLength_fun(MJJ_avg20.MJJ_AVG)-NPathLength_indv_fun(S))];
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

%% Sliding RMS individual subjects

clc

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

figure
hold on

for sub = 1:num_subs
    
    S = load([datadir,'/',Subjects{sub},'/',Subjects{sub},'_AVG_Data_40']);
    
    Error_dF = [Error_dF; RMS_Sliding_indv_fun(dF_avg20.FR_AVG, S)];
    Error_MT = [Error_MT; RMS_Sliding_indv_fun(dT_avg20.TR_AVG, S)];
    Error_MJJ = [Error_MJJ; RMS_Sliding_indv_fun(MJJ_avg20.MJJ_AVG,S)];
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

[p,tbl,stats] = anova1(Errors);
multcompare(stats)


%% EA correlation coefficient
% results: no correlations were significant

dF_20 = load('ParamTrack_20subs.mat');
dT_20 = load('TR_20subs.mat');
MJJ_20 = load('MJJ_20subs.mat');

datadir = 'C:\GoogleDrive\Synced Folder\Experimental Data\NoForce4Targets'; 
% datadir = 'C:\Users\Yeki\Google Drive\Synced Folder\Experimental Data\NoForce4Targets';
datadir2 = 'C:\GoogleDrive\Synced Folder\Experimental Data\NoForce4Target_mix';
Subjects = {'NF04', 'NF05', 'NF06', 'NF07', 'NF08', 'NF15', 'NF16', 'NF30', 'NF31', 'NF32',...
    'NF42', 'NF43', 'NF44', 'NF45', 'NF46', 'NF47', 'NF48', 'NF49', 'NF50', 'NF51'}; 

num_subs = length(Subjects);

Targets0 = [-.1 -.1;
    0.1 -0.1;
    0.1 0.1;
    -0.1 0.1];

TargetsF = [Targets0(2:end,:);
    Targets0(1,:)];

EArea_dF_sim = zeros(4,num_subs);
EArea_dT_sim = zeros(4,num_subs);
EArea_MJJ_sim = zeros(4,num_subs);
EArea_data = zeros(4,num_subs);


for  sub = 1:num_subs
    
    if sub>10
        datadir = datadir2;
    end
    
    S = load([datadir,'/',Subjects{sub},'/',Subjects{sub},'_AVG_Data_40']);
    
    dF_Sim = {dF_20.ParamTrack{sub}, dF_20.ParamTrack{num_subs + sub}, ...
        dF_20.ParamTrack{2*num_subs + sub}, dF_20.ParamTrack{3*num_subs + sub}};
    
    [~, EArea_dF_sim(:,sub), EArea_data(:,sub)] = GapArea_indv_fun(S,dF_Sim, Targets0,TargetsF);
    [~, EArea_dT_sim(:,sub), ~] = GapArea_indv_fun(S,{dT_20.MT{:,sub}}, Targets0,TargetsF);
    [~, EArea_MJJ_sim(:,sub), ~] = GapArea_indv_fun(S,{MJJ_20.MJ_joint{sub,:}}, Targets0,TargetsF);
    
end

figure
for i = 1:4
    
    [r_dF, p_dF] = corrcoef(EArea_dF_sim(i,:), EArea_data(i,:));
    [r_dT, p_dT] = corrcoef(EArea_dT_sim(i,:), EArea_data(i,:));
    [r_MJJ, p_MJJ] = corrcoef(EArea_MJJ_sim(i,:), EArea_data(i,:));
    
%     display(r_dF)
%     display(r_dT)
%     display(r_MJJ)
    display(p_dF)
    display(p_dT)
    display(p_MJJ)
    
    subplot(2,2,i)
    hold on
    plot(EArea_dF_sim(i,:), EArea_data(i,:),'bo')
    plot(EArea_dT_sim(i,:), EArea_data(i,:),'ro')
    plot(EArea_MJJ_sim(i,:), EArea_data(i,:),'go')
%     plot(EArea_data(i,:), EArea_data(i,:),'k*')
    legend('FR','TR','JJ') %,'data')
    title('EA for subs')
end

%% PE correlation
% none of the correletation were significant

dF_20 = load('ParamTrack_20subs.mat');
dT_20 = load('TR_20subs.mat');
MJJ_20 = load('MJJ_20subs.mat');

datadir = 'C:\GoogleDrive\Synced Folder\Experimental Data\NoForce4Targets'; 
% datadir = 'C:\Users\Yeki\Google Drive\Synced Folder\Experimental Data\NoForce4Targets';
datadir2 = 'C:\GoogleDrive\Synced Folder\Experimental Data\NoForce4Target_mix';
Subjects = {'NF04', 'NF05', 'NF06', 'NF07', 'NF08', 'NF15', 'NF16', 'NF30', 'NF31', 'NF32',...
    'NF42', 'NF43', 'NF44', 'NF45', 'NF46', 'NF47', 'NF48', 'NF49', 'NF50', 'NF51'}; 

num_subs = length(Subjects);

PE_dF_sim = zeros(4,num_subs);
PE_dT_sim = zeros(4,num_subs);
PE_MJJ_sim = zeros(4,num_subs);
PE_data = zeros(4,num_subs);


for  sub = 1:num_subs
    
    if sub>10
        datadir = datadir2;
    end
    
    S = load([datadir,'/',Subjects{sub},'/',Subjects{sub},'_AVG_Data_40']);
    
    dF_Sim = {dF_20.ParamTrack{sub}, dF_20.ParamTrack{num_subs + sub}, ...
        dF_20.ParamTrack{2*num_subs + sub}, dF_20.ParamTrack{3*num_subs + sub}};
    
    PE_data(:,sub) = PerpError_data_indv_fun(S);
    PE_dF_sim(:,sub) = PerpError_fun(dF_Sim);
    PE_dT_sim(:,sub) = PerpError_fun({dT_20.MT{:,sub}});
    PE_MJJ_sim(:,sub) = PerpError_fun({MJJ_20.MJ_joint{sub,:}});
    
end

figure
for i = 1:4
    
    [r_dF, p_dF] = corrcoef(PE_dF_sim(i,:), PE_data(i,:));
    [r_dT, p_dT] = corrcoef(PE_dT_sim(i,:), PE_data(i,:));
    [r_MJJ, p_MJJ] = corrcoef(PE_MJJ_sim(i,:), PE_data(i,:));
    
    display(p_dF)
    display(p_dT)
    display(p_MJJ)
    
    subplot(2,2,i)
    hold on
    plot(PE_dF_sim(i,:),PE_data(i,:), 'bo')
    plot(PE_dT_sim(i,:),PE_data(i,:), 'ro')
    plot(PE_MJJ_sim(i,:),PE_data(i,:), 'go')
%     plot(PE_data(i,:),'k*')
    legend('FR','TR','JJ')
    title('PE for subs')
end


%% PL correlation
% none of the correlations are significant

dF_20 = load('ParamTrack_20subs.mat');
dT_20 = load('TR_20subs.mat');
MJJ_20 = load('MJJ_20subs.mat');

datadir = 'C:\GoogleDrive\Synced Folder\Experimental Data\NoForce4Targets'; 
% datadir = 'C:\Users\Yeki\Google Drive\Synced Folder\Experimental Data\NoForce4Targets';
datadir2 = 'C:\GoogleDrive\Synced Folder\Experimental Data\NoForce4Target_mix';
Subjects = {'NF04', 'NF05', 'NF06', 'NF07', 'NF08', 'NF15', 'NF16', 'NF30', 'NF31', 'NF32',...
    'NF42', 'NF43', 'NF44', 'NF45', 'NF46', 'NF47', 'NF48', 'NF49', 'NF50', 'NF51'}; 

num_subs = length(Subjects);

PL_dF_sim = zeros(4,num_subs);
PL_dT_sim = zeros(4,num_subs);
PL_MJJ_sim = zeros(4,num_subs);
PL_data = zeros(4,num_subs);


for  sub = 1:num_subs
    
    if sub>10
        datadir = datadir2;
    end
    
    S = load([datadir,'/',Subjects{sub},'/',Subjects{sub},'_AVG_Data_40']);
    
    dF_Sim = {dF_20.ParamTrack{sub}, dF_20.ParamTrack{num_subs + sub}, ...
        dF_20.ParamTrack{2*num_subs + sub}, dF_20.ParamTrack{3*num_subs + sub}};
    
    PL_data(:,sub) = NPathLength_indv_fun(S);
    PL_dF_sim(:,sub) = NPathLength_fun(dF_Sim);
    PL_dT_sim(:,sub) = NPathLength_fun({dT_20.MT{:,sub}});
    PL_MJJ_sim(:,sub) = NPathLength_fun({MJJ_20.MJ_joint{sub,:}});
    
end

figure
for i = 1:4
    
    [r_dF, p_dF] = corrcoef(PL_dF_sim(i,:), PL_data(i,:));
    [r_dT, p_dT] = corrcoef(PL_dT_sim(i,:), PL_data(i,:));
    [r_MJJ, p_MJJ] = corrcoef(PL_MJJ_sim(i,:), PL_data(i,:));
    
    display(p_dF)
    display(p_dT)
    display(p_MJJ)
    
    subplot(2,2,i)
    hold on
    plot(PL_dF_sim(i,:),PL_data(i,:),'bo')
    plot(PL_dT_sim(i,:),PL_data(i,:), 'ro')
    plot(PL_MJJ_sim(i,:),PL_data(i,:), 'go')
%     plot(PL_data(i,:),'k*')
    legend('FR','TR','JJ')
    title('PL for subs')
end

%% t-test for multiple simulations and multiple subjects

clc

% all reaches

% [h,p] = ttest2(EArea_data(:), EArea_dF_sim(:)) %N
% [h,p] = ttest2(EArea_data(:), EArea_dT_sim(:)) %Y
% [h,p] = ttest2(EArea_data(:), EArea_MJJ_sim(:)) %N

% [h,p] = ttest2(PE_data(:), PE_dF_sim(:)) %Y
% [h,p] = ttest2(PE_data(:), PE_dT_sim(:)) %Y
% [h,p] = ttest2(PE_data(:), PE_MJJ_sim(:)) %N

% [h,p] = ttest2(PL_data(:), PL_dF_sim(:)) %Y
% [h,p] = ttest2(PL_data(:), PL_dT_sim(:)) %N
% [h,p] = ttest2(PL_data(:), PL_MJJ_sim(:)) %Y

% reach 1

% [h,p] = ttest2(EArea_data(1,:), EArea_dF_sim(1,:)) %Y
% [h,p] = ttest2(EArea_data(1,:), EArea_dT_sim(1,:)) %N
% [h,p] = ttest2(EArea_data(1,:), EArea_MJJ_sim(1,:)) %N
% 
% [h,p] = ttest2(PE_data(1,:), PE_dF_sim(1,:)) %Y
% [h,p] = ttest2(PE_data(1,:), PE_dT_sim(1,:)) %Y
% [h,p] = ttest2(PE_data(1,:), PE_MJJ_sim(1,:)) %Y
% 
% [h,p] = ttest2(PL_data(1,:), PL_dF_sim(1,:)) %Y
% [h,p] = ttest2(PL_data(1,:), PL_dT_sim(1,:)) %Y
% [h,p] = ttest2(PL_data(1,:), PL_MJJ_sim(1,:)) %Y

% reach 2

% [h,p] = ttest2(EArea_data(2,:), EArea_dF_sim(2,:)) %Y
% [h,p] = ttest2(EArea_data(2,:), EArea_dT_sim(2,:)) %Y
% [h,p] = ttest2(EArea_data(2,:), EArea_MJJ_sim(2,:)) %N
% % 
% [h,p] = ttest2(PE_data(2,:), PE_dF_sim(2,:)) %Y
% [h,p] = ttest2(PE_data(2,:), PE_dT_sim(2,:)) %Y
% [h,p] = ttest2(PE_data(2,:), PE_MJJ_sim(2,:)) %Y
% 
% [h,p] = ttest2(PL_data(2,:), PL_dF_sim(2,:)) %N
% [h,p] = ttest2(PL_data(2,:), PL_dT_sim(2,:)) %N
% [h,p] = ttest2(PL_data(2,:), PL_MJJ_sim(2,:)) %Y

% reach 3

% [h,p] = ttest2(EArea_data(3,:), EArea_dF_sim(3,:)) %Y
% [h,p] = ttest2(EArea_data(3,:), EArea_dT_sim(3,:)) %Y
% [h,p] = ttest2(EArea_data(3,:), EArea_MJJ_sim(3,:)) %Y
% 
% [h,p] = ttest2(PE_data(3,:), PE_dF_sim(3,:)) %N
% [h,p] = ttest2(PE_data(3,:), PE_dT_sim(3,:)) %Y
% [h,p] = ttest2(PE_data(3,:), PE_MJJ_sim(3,:)) %N
% 
% [h,p] = ttest2(PL_data(3,:), PL_dF_sim(3,:)) %Y
% [h,p] = ttest2(PL_data(3,:), PL_dT_sim(3,:)) %N
% [h,p] = ttest2(PL_data(3,:), PL_MJJ_sim(3,:)) %Y

% reach 4

% [h,p] = ttest2(EArea_data(4,:), EArea_dF_sim(4,:)) %Y
% [h,p] = ttest2(EArea_data(4,:), EArea_dT_sim(4,:)) %Y
% [h,p] = ttest2(EArea_data(4,:), EArea_MJJ_sim(4,:)) %N
% 
% [h,p] = ttest2(PE_data(4,:), PE_dF_sim(4,:)) %Y
% [h,p] = ttest2(PE_data(4,:), PE_dT_sim(4,:)) %Y
% [h,p] = ttest2(PE_data(4,:), PE_MJJ_sim(4,:)) %Y

% [h,p] = ttest2(PL_data(4,:), PL_dF_sim(4,:)) %N
% [h,p] = ttest2(PL_data(4,:), PL_dT_sim(4,:)) %N
% [h,p] = ttest2(PL_data(4,:), PL_MJJ_sim(4,:)) %Y

%% THE END


