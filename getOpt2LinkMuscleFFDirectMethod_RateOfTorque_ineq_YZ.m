
%
% based off of getOpt2linkDirectMethod, this one includes a curl force
% field!
%

%% 

function [Sol, flag, algorithm, fval, N, M] = getOpt2LinkMuscleFFDirectMethod_RateOfTorque_ineq_YZ(params)

a1 = params.a1;
a2 = params.a2;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

T = params.T;
numpts = params.numpts;
time = linspace(0,T,numpts);
dt = time(2)-time(1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% state-specific info
state_dim = 10;
comm_dim = params.comm_dim;

UB = params.UB;
LB = params.LB;

xstart = params.xstart;
xfinish = params.xfinish;
qo = inv_Position_2(xstart(1:2),a1, a2);
qd = inv_Position_2(xfinish(1:2),a1, a2);
qod = inv_Velocity(xstart(3:4),qo, a1, a2);
qdd = inv_Velocity(xfinish(3:4),qd, a1, a2);
IC = [qo; qod; zeros(6,1)];
FDC = [qd; qdd; zeros(6,1)];


Q = params.Q;
R = params.R;
Phi = params.Phi;

N = (state_dim+comm_dim)*numpts-comm_dim;
% equality constraints
M = state_dim*(numpts+1);
fprintf('\n')
disp([' Starting ', num2str(N),' dimensional NLP variable with ',num2str(M),' constraints'])

state_index = 1:state_dim:numpts*state_dim;
comm_index = state_dim*numpts+1:comm_dim:N;

num_command_pts = comm_dim*numpts - comm_dim;

% initial guess, NLP variables are state, 2N, and command, N-1.
if isfield(params,'initSol')
    Xo = params.initSol;
%     fprintf('\n')
%     disp(' ...using passed initial guess at solution!... ')
%     fprintf('\n')
else
    Xo = zeros(N,1);
    Xo(1:state_dim) = IC;
end

% plot?
if isfield(params,'PLOT')
    PLOT = params.PLOT;
else
    PLOT = false;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% linear inequalities, linear equalities
A = [];
b = [];
Aeq = [];
beq = [];
% upper and lower bounds
lb = [];
ub = [];

%%%%%%%%%%%%%%%%%% Constraints
% disp(' using linear inequality ')

M_hat = 6*numpts; %number of states with constrains

A = zeros(2*M_hat,N); %one half for ub/ one half for lb
b = zeros(2*M_hat,1);
% these loops place ones to grab commands X(:,5), X(:,6), etc.
loop = 1;
for j = 1:6
    for i = 1:length(state_index)
        rowsf = (state_index(i)+4):(state_index(i)+4+comm_dim-1);
        A(loop,rowsf(j)) = 1;
        loop = loop + 1;
    end
end

% now we just add the same thing to the matrix, but swap sign
A(loop:end,:) = -A(1:loop-1,:);
b(1:loop-1) = UB;
b(loop:end) = -LB;

%%%%%%%%%%%%%%%%%%

% for nonlinear constraint
Anonlin = eye(M,N);
for i = 2:numpts
    rowindex = 1+(i-1)*state_dim:i*state_dim;
    Anonlin(rowindex,rowindex-state_dim) = -eye(state_dim);
end
Anonlin(301:310,291:300)=eye(10);
Anonlin(301:310,301:310)=zeros(10);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
parameters.N = N;
parameters.M = M;
parameters.dt = dt;
parameters.FDC = FDC;
parameters.state_index = state_index;
parameters.state_dim = state_dim;
parameters.comm_index = comm_index;
parameters.comm_dim = comm_dim;
parameters.Phi = Phi;
parameters.Q = Q;
parameters.R = R;
parameters.IC = IC;
parameters.Anonlin = Anonlin;
% parameters.M_arm = params.M_arm;
% parameters.f_max = params.f_max;
parameters.M_max = params.M_max;
parameters.m1 = params.m1;
parameters.m2 = params.m2;
parameters.a1 = params.a1;
parameters.a2 = params.a2;
parameters.a1_cm = params.a1_cm;
parameters.a2_cm = params.a2_cm;
parameters.I_1 = params.I_1;
parameters.I_2 = params.I_2;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  options = optimset('MaxFunEval',2E5,'MaxIter',1E5,'TolFun',1E-14,...
%      'TolX',1E-14,'TolCon',1E-14,'LargeScale','off',...
%      'Algorithm','sqp');
% options = optimset('MaxFunEval',2E5,'MaxIter',1E5,'TolFun',1E-14,...
%    'TolX',1E-14,'TolCon',1E-14,'LargeScale','off',...
%    'Algorithm','sqp','GradConstr','on','GradObj','on');
%
% previously i had algorithm set to sqp, but strangely when i pass it the '
% gradident of the objective funciton it slows down to the point it halts
% because it reaches iterlimit! i removed largscale off (since it barked
% about that) and changed to active-set, which is what it switched to when
% i didn't specify an algorithm

% options = optimset('MaxFunEval',2E5,'MaxIter',1E5,'TolFun',1E-14,...
%    'TolX',1E-14,'TolCon',1E-14,'Algorithm','active-set',...
%    'GradConstr','on','GradObj','on', 'Display','iter');
%  'Algorithm','sqp'
algorithm = 'active-set';
options = optimset('MaxFunEval',2E5,'TolFun',1E-14,...
   'TolX',1E-14,'TolCon',1E-14,'Algorithm',algorithm,...
   'GradObj','on', 'GradConstr','on','Display', 'Iter');
% tic
[X, fval, flag, output, lambda, grad] = ...
    fmincon(@(X)Objective(X,parameters),Xo,A,b,Aeq,beq,lb,ub,@(X)nonlinConstraint(X,parameters),options);
% toc

% display('flag = ')
% display(flag)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for i = 1:state_dim
    q(:,i) = X(state_index+(i-1));
end
for i = 1:comm_dim
    U(:,i) = X(comm_index+(i-1));
end
x = zeros(numpts,state_dim-6);
for i = 1:numpts
    x(i,:) = forward_Map(q(i,1:4), a1, a2);
end

Sol.Q = q;
Sol.X = x;
Sol.U = U;
Sol.t = time;
Sol.Objective = fval;
Sol.NLPx = X;

% fprintf('\n')
% disp(' ...Done!... ')
% fprintf('\n')
    
if PLOT
    
    figure
    subplot(3,1,1)
    plot(time,q(:,1:2),'b',time,q(:,1:2),'.')
    grid
    title(['objective: ',num2str(fval)])
    ylabel('q')
    subplot(3,1,2)
    plot(time,q(:,3:4),'b',time,q(:,3:4),'.')
    grid
    ylabel('qd')
    subplot(3,1,3)
    plot(time(1:numpts-1),U,'b',time(1:numpts-1),U,'.')
    grid
    ylabel('tau')
    
end


%% objective function to be minimized
function [F, g] = Objective(X,params)

N = params.N;
dt = params.dt;
FDC = params.FDC;
state_index = params.state_index;
state_dim = params.state_dim;
comm_index = params.comm_index;
comm_dim = params.comm_dim;
Phi = params.Phi;
Q = params.Q;
R = params.R;

cost = 0;
for i = 1:length(comm_index)
    rowsq = state_index(i):state_index(i)+state_dim-1;
    rowsu = comm_index(i):comm_index(i)+comm_dim-1;
    cost = cost + .5*(X(rowsq)-FDC)'*Q*(X(rowsq)-FDC) ...
        + .5*X(rowsu)'*R*X(rowsu);
end

%F = .5*(q(end,:)'-FDC)'*Phi*(q(end,:)'-FDC) + dt*cost;
rows = state_index(end):state_index(end)+state_dim-1;
F = .5*(X(rows)-FDC)'*Phi*(X(rows)-FDC) + dt*cost;

if nargout > 1
    % compute gradient of objective
    g = zeros(N,1);
    for i = 1:length(state_index)-1
        rows = state_index(i):state_index(i)+state_dim-1;
        g(rows) = dt*Q*(X(rows)-FDC);
        %
        rows = comm_index(i):comm_index(i)+comm_dim-1;
        g(rows) = dt*R*X(rows);
    end
    rows = state_index(end):state_index(end)+state_dim-1;
    g(state_index(end):state_index(end)+state_dim-1) = ...
        Phi*(X(rows)-FDC);
    
end


%% constraints (dynamics) % what was G and Geq? I took them out
function [C, Ceq, G, Geq] = nonlinConstraint(X,params)

N = params.N;
M = params.M;
dt = params.dt;
IC = params.IC;
Anonlin = params.Anonlin;
FDC = params.FDC;
state_index = params.state_index;
state_dim = params.state_dim;
comm_dim = params.comm_dim;
comm_index = params.comm_index;

% inequalities?
C = [];

% equalities
B = zeros(M,1);
B(1:state_dim) = -IC;
B(M-state_dim+1:M) = -FDC;
for i = 2:length(state_index)
    
    state = X([state_index(i-1):state_index(i-1)+state_dim-1]);
    comm = X([comm_index(i-1):comm_index(i-1)+comm_dim-1]);
    B(1+(i-1)*state_dim:i*state_dim) = ...
        -dt*dynamics(state, comm,params);
end

Ceq = Anonlin*X + B;
% Ceq(state_index(end):M) = ...

if nargout > 2
    % compute derivatives
    G = [];
    Geq = zeros(M,N);
    % finite difference this thing
    epsilon = 1E-5;
    deltaX = epsilon*eye(state_dim);
    deltaU = epsilon*eye(comm_dim);
    for i = 1:length(state_index)-1
        
        rows = state_index(i+1):state_index(i+1)+state_dim-1;
        colsx = state_index(i):state_index(i)+state_dim-1;
        colsu = comm_index(i):comm_index(i)+comm_dim-1;
        F = B(rows); 
        baseX = X(colsx);
        baseU = X(colsu);
        %
        F_x = zeros(state_dim);
        for j = 1:state_dim
            xd = baseX + deltaX(:,j);
            Fd = -dt*dynamics(xd, baseU,params);
            F_x(:,j) = (Fd-F)/epsilon;
        end
        % place partials in their proper locaiton
        Geq(rows,colsx) = F_x;
        %
        F_u = zeros(state_dim,comm_dim);
        for j = 1:comm_dim
            ud = baseU + deltaU(:,j);
            Fd = -dt*dynamics(baseX, ud,params);
            F_u(:,j) = (Fd-F)/epsilon;
        end
        % place partials in their proper locaiton
        Geq(rows,colsu) = F_u;
        
    end
     
    % for some reason, matlab wants the transpose?
    % according to matlab, gradC_ij = partialC(j)/partial X(i) which is the
    % transpose of how i am computing it
    Geq = Anonlin' + Geq';
%     Geq(1:M, state_index(end):M) = -ones(M,state_dim);
%     Geq(M-state_dim+1:M, M-2*state_dim+1:M-state_dim) = eye(state_dim);
    
end

%% dynamics
function dXdt = dynamics(X,u,Param)

% M_arm = Param.M_arm;
% f_max = Param.f_max;
M_max = Param.M_max;
m = [Param.m1, Param.m2];
l = [Param.a1, Param.a2];
r = [Param.a1_cm, Param.a2_cm];
II = [Param.I_1, Param.I_2];
%
gama = m(1)*r(1)^2 + m(2)*r(2)^2 + m(2)*l(1)^2 + sum(II);
alpha = 2*m(2)*r(2)*l(1);

I = [ gama+ alpha*cos(X(2)), m(2)*r(2)^2+ II(2)+ 1/2*alpha*cos(X(2));
    m(2)*r(2)^2+ II(2)+ 1/2*alpha*cos(X(2)), m(2)*r(2)^2+ II(2)];


sigma = -l(1)^2*m(2)^2*r(2)^2*cos(X(2))^2 +...
l(1)^2*m(2)^2*r(2)^2+...
II(2)*l(1)^2*m(2)+ ...
m(1)*m(2)*r(1)^2*r(2)^2+...
II(1)*m(2)*r(2)^2+...
II(2)*m(1)*r(1)^2+...
II(1)*II(2);

I_inv = 1/sigma*[m(2)*r(2)^2+ II(2), -(m(2)*r(2)^2+ II(2)+ 1/2*alpha*cos(X(2)));
    -(m(2)*r(2)^2+ II(2)+ 1/2*alpha*cos(X(2))), gama+ alpha*cos(X(2))];

% Compute dI_inv_dx2

sigma2 = -l(1)^2*m(2)^2*r(2)^2*cos(X(2))^2 +...
    l(1)^2*m(2)^2*r(2)^2+...
    II(2)*l(1)^2*m(2)+ ...
    m(1)*m(2)*r(1)^2*r(2)^2+...
    II(1)*m(2)*r(2)^2+...
    II(2)*m(1)*r(1)^2+...
    II(1)*II(2);

sigma1 = 1/sigma2*(l(1)*m(2)*r(2)*sin(X(2))) +...
    1/(sigma2^2)*( 2*l(1)^2*m(2)^2*r(2)^2*cos(X(2))*sin(X(2))*( m(2)*r(2)^2 + l(1)*m(2)*r(2)*cos(X(2))+ II(2)) );


dI_inv_dx2 = [-1/sigma2^2*(l(1)^2*m(2)^2*r(2)^2)*cos(X(2))*sin(X(2))*2*(m(2)*r(2)^2 + II(2)),...
    sigma1;
    sigma1,...
    -1/sigma2*(2*l(1)*m(2)*r(2)*sin(X(2)))-1/(sigma2)^2*( l(1)^2*m(2)^2*r(2)^2*cos(X(2))*sin(X(2))*2*(m(2)*l(1)^2+2*m(2)*cos(X(2))*l(1)*r(2)+ m(1)*r(1)^2+ m(2)*r(2)^2+ II(1)+II(2)) )];

% partials of M

J = [-l(1)*sin(X(1))-l(2)*sin(X(1)+X(2)) -l(2)*sin(X(1)+X(2));
    l(1)*cos(X(1))+l(2)*cos(X(1)+X(2)) l(2)*cos(X(1)+X(2))];

h = -m(2)*l(1)*r(2)*sin(X(2));

C = [h*X(4) h*(X(3)+X(4));
    -h*X(3) 0];

M = X(5:6) -C*[X(3);X(4)];

%partials of M

dh_dx2 = -m(2)*l(1)*r(2)*cos(X(2));

dC_dx2 = [dh_dx2*X(4), dh_dx2*(X(4)+X(3));
    -dh_dx2*X(3), 0];

dC_dx3 = [0, h;
    -h, 0];

dC_dx4 = [h, h;
    0, 0];

%
dM_dx1 = [0;0]; 

dM_dx2 = - dC_dx2*[X(3);X(4)];

dM_dx3 = - dC_dx3*[X(3);X(4)] - C*[1; 0];

dM_dx4 = - dC_dx4*[X(3);X(4)] - C*[0; 1]; 

dM_dx5 = M_max*[1;0;0;0;0;0];

dM_dx6 = M_max*[0;1;0;0;0;0];

dM_dx7 = M_max*[0;0;1;0;0;0];

dM_dx8 = M_max*[0;0;0;1;0;0];

dM_dx9 = M_max*[0;0;0;0;1;0];

dM_dx10 = M_max*[0;0;0;0;0;1];


Si = [inv(I)*dM_dx1, dI_inv_dx2*M+inv(I)*dM_dx2, inv(I)*dM_dx3, inv(I)*dM_dx4, inv(I)*dM_dx5, inv(I)*dM_dx6, inv(I)*dM_dx7, inv(I)*dM_dx8, inv(I)*dM_dx9, inv(I)*dM_dx10];

% Compute the partials
% df_du = [zeros(4,rr);
%     eye(rr)];
% 
% df_dX = [0 0 1 0 0 0 0 0 0 0;
%     0 0 0 1 0 0 0 0 0 0
%     Si
%     zeros(n-4,n)];

dXdt = [X(3);X(4);
    inv(I)*(M_max*X(5:10) - C*X(3:4));
    u];

%% end code
