%
% compute the RMS error between Data and 4 simulated reaches

% sync data before comparison

% The begining of the reach is synced
% Then end of the reach is not, because I don't have the speed profile for
% the data.

function RMS = RMS_v2_fun(Simulations,Data, counter)

global sho_x sho_y

RMS = zeros(1,4);

% figure

for i = 1:4
    
    N = sum(Data.BlockData.X_Cnt{i}>=Data.BlockData.X_Cnt{i}(1));
    
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
    
    numpt = length(Speed_temp(id0_sim:end));
    id = floor(linspace(id0_data, N, numpt));
    
    %%%%%%%%%%%%%%%%%
    
    diff_x = mean((Data.BlockData.X_avg{i}(id) - sho_x - X_temp(id0_sim:end)').^2);
    diff_y = mean((Data.BlockData.Y_avg{i}(id) - sho_y - Y_temp(id0_sim:end)').^2);
    
    RMS(i) = sqrt( diff_x + diff_y );
    
%     
end

end