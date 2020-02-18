

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

Parameters.M_max = [54 -43 13 -19 0 0;
    0 0 14 -12 38 -29]; % Nijhof and Kouwenhoven

Parameters.M_arm = [5.9, -3.3, 3.2, -3.2, 0, 0;
    0,0 3.4, -2, 2.7, -1.6]*0.01;
Parameters.f_max = diag([905, 1805, 322, 469, 1548, 1620]); %Nijhof's paper


Parameters.T = .75;
Parameters.comm_dim = comm_dim;
%
Parameters.numpts = 30; %30; %75;
%
Parameters.Q = zeros(state_dim);
R_nominal = 1e-5*Parameters.f_max'*Parameters.f_max; %1e-1*params.M_max'*params.M_max; %1e-2*eye(comm_dim); %10*eye(comm_dim);
Parameters.Phi = 0*eye(state_dim);
Parameters.PLOT = false;
Parameters.Curl = zeros(2);
%
Parameters.LB = 0;
Parameters.UB = 1;
%
xo = 0; %.2;
yo = -0.4;

% Data = load('SOLINIT_dT_30');
% datadir = '/Users/Yekta/Documents/MATLAB/Motor Behavior Lab/Modeling/Muscle Model with EL method/';
% InitSol = load([datadir,'Simulations_MinRateTorque_2917.mat']);

%% minimizing rate of force

Data = load('SOLINIT_dF_30');

counter = 1;

for j1 = [0.5, 1, 1.5]; %[0.01, 1, 2]; %[0.5, 1, 1.5]; %100*[1,10,20] %[0.01, 0.5, 1]
    R(1,1) = j1*R_nominal(1,1);
    
    for j2 = [0.5, 1, 1.5]; %[0.01, 1, 2]; %[1,10,20]
        R(2,2) = j2*R_nominal(2,2);
        
        for j3 = [0.5, 1, 1.5]; %[0.01, 1, 2]; %[1,10,20]
            R(3,3) = j3*R_nominal(3,3);
            
            for j4 = [0.5, 1, 1.5]; %[0.01, 1, 2]; %[1,10,20]
                R(4,4) = j4*R_nominal(4,4);
                
                for j5 = [0.5, 1, 1.5]; %[0.01, 1, 2]; %[1,10,20]
                    R(5,5) = j5*R_nominal(5,5);
                    
                    for j6 = [0.5, 1, 1.5]; %[0.01, 1, 2]; %[1,10,20]
                        R(6,6) = j6*R_nominal(6,6);
                        
                        Parameters.R = R;
                        
                        
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
                            
%                             try
                                %[X,u,x_pos,y_pos] = TwoLinkArm_OptimalController(Parameters,0);
                                %[X,u,x_pos,y_pos] = TwoLinkArm_OptimalRateTorque_ConstrainedInput(Parameters,0);
%                                 [Sol, flag, algorithm, fval, Numpt, Numcon] = getOpt2LinkMuscleFFDirectMethod_RateOfTorque_ineq_YZ(Parameters);
                                [Sol, flag, algorithm, fval, Numpt, Numcon] = getOpt2LinkMuscleFFDirectMethod_RateOfForce_ineq_YZ_2(Parameters);
                                
                                ParamTrack{counter}.Sol = Sol;
                                ParamTrack{counter}.R = R;
                                ParamTrack{counter}.flag = flag;
                                ParamTrack{counter}.algorithm = algorithm;
                                ParamTrack{counter}.fval = fval;
                                ParamTrack{counter}.Numpt = Numpt;
                                ParamTrack{counter}.Numcon = Numcon;
                                ParamTrack{counter}.Parameters = Parameters;
                                
                                
%                             catch
%                                 ParamTrack{counter}.R = R;
%                                 ParamTrack{counter}.Parameters = Parameters;
%                                 ParamTrack{counter}.fail = 1;
%                                 display('failed')
%                             end
                            
                            counter = counter +1
                            %
                        end
                        
                    end
                end
            end
            savefile = ['temp_simulation', num2str(counter)];
            save(savefile,'ParamTrack');
        end
    end
end

fprintf('\n')
disp(' ...Done!... ')
fprintf('\n')

%% plot

figure
hold on

for counter = 1:length(ParamTrack)
    plot(ParamTrack{counter}.Sol.X(:,1),ParamTrack{counter}.Sol.X(:,2))
end
plot(0,0,'rx')
axis equal


%% THE END


