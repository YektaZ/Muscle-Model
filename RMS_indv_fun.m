%
% compute the RMS error between Data and 4 simulated reaches

% sync data before comparison

% The begining of the reach is synced
% Then end of the reach is not, because I don't have the speed profile for
% the data.

function RMS = RMS_indv_fun(Simulations,Data)

global sho_x sho_y

RMS = zeros(1,4);

for i = 1:4
    
    N = length(Data.AVG_Data.X{i});
    
    %%%%%%%%%%%%%%%%%     interpolate
    
    Speed_sim = sqrt(Simulations{i}.Sol.X(:,3).^2 + Simulations{i}.Sol.X(:,4).^2);
    
    T = Simulations{i}.Sol.t;
    TT = Data.AVG_Data.Time{i}(51:N);
    dt = TT(2)-TT(1);
    
    count = 1;
    for j = 0.001:dt:T(end)
        Speed_temp(count) = interp1(T,Speed_sim,j);
        X_temp(count) = interp1(T,Simulations{i}.Sol.X(:,1),j);
        Y_temp(count) = interp1(T,Simulations{i}.Sol.X(:,2),j);
        count = count + 1;
    end
    
    id0_data = find(Data.AVG_Data.Time{i}>0, 1);
    id0_sim = find(Speed_temp>0.05, 1);
    
    numpt = length(Speed_temp(id0_sim:end));
    id = floor(linspace(id0_data, N, numpt));
    
    %%%%%%%%%%%%%%%%%
    
    diff_x = mean((Data.AVG_Data.X{i}(id) - sho_x - X_temp(id0_sim:end)').^2);
    diff_y = mean((Data.AVG_Data.Y{i}(id) - sho_y - Y_temp(id0_sim:end)').^2);
    
    RMS(i) = sqrt( diff_x + diff_y );
    
    %
end

end