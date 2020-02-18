%
% compute the log likelihood error between Data and 4 simulated reaches

function logL = logL_v2_fun(Simulations,Data)

global sho_x sho_y

% numpt = length(Simulations{1}.Sol.X);
logL = zeros(1,4);

for i = 1:4
    
    
    N = sum(Data.BlockData.X_Cnt{i}>9);
    %%%%%%%%%%%%%%%%%     interpolate
    
    Speed_sim = sqrt(Simulations{i}.Sol.X(:,3).^2 + Simulations{i}.Sol.X(:,4).^2);
    
    T = Simulations{i}.Sol.t;
    TT = Data.BlockData.Time{i}(51:N);
    dt = TT(2)-TT(1); 
    
    count = 1;
    for j = 0.001:dt:T(end)
        Speed_temp(count) = interp1(T,Speed_sim,j);
        X_temp(count) = interp1(T,Simulations{i}.Sol.X(:,1),j);
        Y_temp(count) = interp1(T,Simulations{i}.Sol.X(:,2),j);
        count = count + 1;
    end
    
    id0_data = find(Data.BlockData.Time{i}>0, 1);
    id0_sim = find(Speed_temp>0.05, 1);
    
    numpt = length(Speed_temp); % numpt for simulations
    
    id_data = floor(linspace(id0_data, N, 700)); % only consider 100 data points
    id_sim = floor(linspace(id0_sim,numpt, 700));
    
    %%%%%%%%%%%%%%%%%
    
    logL_temp = zeros(1, length(id_data));
    likelihood_temp = zeros(1, length(id_data));
    
    for pt = 1:length(id_data) %numpt
        
        X = [Data.BlockData.X_avg{i}(id_data(pt)) - sho_x,  Data.BlockData.Y_avg{i}(id_data(pt)) - sho_y];
        Miu = [X_temp(id_sim(pt)), Y_temp(id_sim(pt))];
        
        % The SEM is squared and multiplied by the number of subjects to get Varience
        % update: use the SEM itself
        Sigma = [Data.BlockData.X_SEM{i}(id_data(pt)).^2, 0;
            0, Data.BlockData.Y_SEM{i}(id_data(pt)).^2]; %covariance matrix
        Q = inv(Sigma); %precision matrix
        
        likelihood_temp(pt) = 1/(2*pi)*1/sqrt(det(Sigma))*exp(-1/2*(X-Miu)*Q*(X-Miu)');
        
        logL_temp(pt) = -log(2*pi)...
            - 1/2*log(det(Sigma))...
            -1/2*(X-Miu)*Q*(X-Miu)';
        
        
    end
    
    logL(i) = sum(logL_temp);
    
end

end