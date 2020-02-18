

% do mass simulations for direct method, minimizing rate of force or rate
% of torque

%%

close all
clear all
clc

Targets0 = [-.1 -.1;
    0.1 -0.1;
    0.1 0.1;
    -0.1 0.1];

TargetsF = [Targets0(2:end,:);
    Targets0(1,:)];

% limb parameters
Parameters.I_1 = .0264;
Parameters.I_2 = .0472;
Parameters.m1 = 1.85;
Parameters.m2 = 1.69;
Parameters.a1 = 0.295;
Parameters.a2 = 0.3;
Parameters.a1_cm = 0.47*Parameters.a1;
Parameters.a2_cm = 0.56*Parameters.a2;

comm_dim = 6;
state_dim = 10;

% Parameters.M_max = [54 -43 13 -19 0 0;
%     0 0 14 -12 38 -29]; % Nijhof and Kouwenhoven

Parameters.M_max = [54 -43 7 -11 0 0;
    0 0 14 -12 38 -29]; % Nijhof and Kouwenhoven, verified

Parameters.M_arm = [5.9, -3.3, 3.2, -3.2, 0, 0;
    0,0 3.4, -2, 2.7, -1.6]*0.01;
Parameters.f_max = diag([905, 1805, 322, 469, 1548, 1620]); %Nijhof's paper

Parameters.T = 1; % .75;
Parameters.comm_dim = comm_dim;
%
Parameters.numpts = 30; %30; %75;
%
Parameters.Q = zeros(state_dim); 
Parameters.Phi = 0*eye(state_dim);
Parameters.PLOT = false;
Parameters.Curl = zeros(2);
%
Parameters.LB = 0;
Parameters.UB = 1;
%
xo = 0; %.2;
yo = -0.4;
Parameters.sho_x = 0;
Parameters.sho_y = -0.4;

% Data = load('SOLINIT_dT_30');
% datadir = '/Users/Yekta/Documents/MATLAB/Motor Behavior Lab/Modeling/Muscle Model with EL method/';
% InitSol = load([datadir,'Simulations_MinRateTorque_2917.mat']);

%% nominal solutions
% minimizing rate of force

R_nominal = 1e-5*Parameters.f_max'*Parameters.f_max; 
% R_nominal = 1e-3*Parameters.M_max'*Parameters.M_max; % The gain doesn't effect the results. It only helps the simulations go faster

Data = load('SOLINIT_dF_30');

counter = 1;

Parameters.R = R_nominal;


for i = 1:length(Targets0)
    
    Targ0 = Targets0(i,:);
    Targf = TargetsF(i,:);
    
    Parameters.xstart = [Targ0 - [xo, yo], 0, 0]';
    Parameters.xfinish = [Targf- [xo, yo], 0, 0]';
    
    X_temp = Data.X{i}';
    u_temp = Data.u{i};
    u_temp = u_temp(:,1:end-1);
    
    Parameters.initSol = [X_temp(:); u_temp(:)];
    
    %%%%%%%%%%%%%%%% Obtimal Model (min torque)
    
    %[X,u,x_pos,y_pos] = TwoLinkArm_OptimalController(Parameters,0);
    %[X,u,x_pos,y_pos] = TwoLinkArm_OptimalRateTorque_ConstrainedInput(Parameters,0);
%      [Sol, flag, algorithm, fval, Numpt, Numcon] = getOpt2LinkMuscleFFDirectMethod_RateOfTorque_ineq_YZ(Parameters);
    [Sol, flag, algorithm, fval, Numpt, Numcon] = getOpt2LinkMuscleFFDirectMethod_RateOfForce_ineq_YZ_2(Parameters);
    
    ParamTrack{counter}.Sol = Sol;
    ParamTrack{counter}.R = Parameters.R;
    ParamTrack{counter}.flag = flag;
    ParamTrack{counter}.algorithm = algorithm;
    ParamTrack{counter}.fval = fval;
    ParamTrack{counter}.Numpt = Numpt;
    ParamTrack{counter}.Numcon = Numcon;
    ParamTrack{counter}.Parameters = Parameters;
    
    
    
    counter = counter +1
    %
end


fprintf('\n')
disp(' ...Done!... ')
fprintf('\n')

%% saving

save('nominal_solution_dF_1sec','ParamTrack') % 1 second
% save('nominal_solution_dT_1sec_gain-3','ParamTrack') % 1 second
% save('nominal_solution_dT_temp','ParamTrack')

%% plot

figure
hold on

for counter = 1:length(ParamTrack)
    plot(ParamTrack{counter}.Sol.X(:,1),ParamTrack{counter}.Sol.X(:,2))
end
plot(0,0,'rx')
axis equal

%% load the nominal simulations if needed
load('nominal_solution_dF.mat')

%% RMS Error for nominal solutions (this RMS error is not synced! no good!)
global sho_x sho_y

sho_x = 0;
sho_y = -0.4;

% datadir = 'C:\GoogleDrive\Synced Folder\Experimental Data\NoForce4Targets\Averages';
% datadir = 'C:\Users\Yeki\Google Drive\Synced Folder\Experimental Data\NoForce4Targets\Averages';
datadir = '/Users/yektazahed/Google Drive/Synced Folder/Experimental Data/NoForce4Targets/Averages';
S = load([datadir,'/LimbFeedback_Averages4']);

%%%%%%%%%%%%%%%%%%%%%%%% Original averages from before I lost my mac
% datadir = 'C:\Users\Yeki\Desktop\NoForce4Targets\Averages\';
% S = load ([datadir, 'NoForceLimbPos_Block_Bin_AVGS']);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for counter = 1% only 4 trials 
    
    Error_dF = sum(RMS_fun(ParamTrack,S, counter));

end

display(['RMS error: ', num2str(Error_dF)])
%
figure
hold on
for i = 1:4
    plot(ParamTrack{i}.Sol.X(:,1),ParamTrack{i}.Sol.X(:,2))
    plot(S.BlockData.X_avg{i}-sho_x, S.BlockData.Y_avg{i}-sho_y,'b--')
%     plot(S.BlockData.XrAvg{i}-sho_x, S.BlockData.YrAvg{i}-sho_y,'b--')
end
%
axis equal
title('avg data & nominal min dF')

%% THE END


