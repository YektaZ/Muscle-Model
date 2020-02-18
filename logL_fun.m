%
% compute the log likelihood error between Data and 4 simulated reaches

function logL = logL_fun(Simulations,Data, counter)

global sho_x sho_y

% numpt = length(Simulations{1}.Sol.X);
logL = zeros(1,4);

for i = 1:4
    
    
    N = sum(Data.BlockData.X_Cnt{i}>9);
        %%%%%%%%%%%%%%%%%     interpolate
    
    Speed_sim = sqrt(Simulations{counter+i-1}.Sol.X(:,3).^2 + Simulations{counter+i-1}.Sol.X(:,4).^2);
    
    T = Simulations{i}.Sol.t;
    TT = Data.BlockData.Time{i}(51:N);
    dt = TT(2)-TT(1); 
    
    count = 1;
    for j = 0.001:dt:T(end)
        Speed_temp(count) = interp1(T,Speed_sim,j);
        X_temp(count) = interp1(T,Simulations{counter+i-1}.Sol.X(:,1),j);
        Y_temp(count) = interp1(T,Simulations{counter+i-1}.Sol.X(:,2),j);
        count = count + 1;
    end
    
    id0_data = find(Data.BlockData.Time{i}>0, 1);
    id0_sim = find(Speed_temp>0.05, 1);
%     idf_data = find(Data.BlockData.Speed_avg{i}>0.05, 1,'last');
%     idf_sim = find(Speed_temp>0.05, 1,'last');
    
%     N = min(N,idf_data);
    
    numpt = length(Speed_temp(id0_sim:end));
    id = floor(linspace(id0_data, N, numpt));
    
    %%%%%%%%%%%%%%%%%
    
    
    
    
    
%     N = length(Data.BlockData.X_avg{i});
%     id = floor(linspace(1, N, numpt));
    
    logL_temp = zeros(1, numpt);
    
    for pt = 1:numpt
        
        X = [Data.BlockData.X_avg{i}(id(pt)) - sho_x,  Data.BlockData.Y_avg{i}(id(pt)) - sho_y];
%         Miu = [Simulations{counter+i-1}.Sol.X(pt,1), Simulations{counter+i-1}.Sol.X(pt,2)];
        Miu = [X_temp(pt), Y_temp(pt)];
        
        % The SEM is squared and multiplied by the number of subjects to get
        % Varience
        Sigma = 10*[Data.BlockData.X_SEM{i}(id(pt)).^2, 0;
            0, Data.BlockData.Y_SEM{i}(id(pt)).^2]; %covariance matrix
        Q = inv(Sigma); %precision matrix
        
        logL_temp(pt) = -log(2*pi)...
            - 1/2*log(det(Sigma))...
            -1/2*(X-Miu)*Q*(X-Miu)';
        
    end
    
    logL(i) = sum(logL_temp);
    
end

end