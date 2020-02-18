%
% compute the RMS error between Data and 4 simulated reaches

function RMS = RMS_fun_2(Simulations,Data, counter)

global sho_x sho_y

numpt = length(Simulations{1});

RMS = zeros(1,4);

for i = 1:4
    
%     N = sum(Data.BlockData.XrCnt{i}>9);
%     N = length(Data.BlockData.X_avg{i});
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
%     idf_sim = find(Speed_temp>0.05, 1,'last');
    
    numpt = length(Speed_temp(id0_sim:end));
    id = floor(linspace(id0_data, N, numpt));
    
    
    
    
    
    
%     id = floor(linspace(1, N, numpt));
    
%     diff_x = mean((Data.BlockData.XrAvg{i}(id) - sho_x - Simulations{counter+i-1}(:,1)).^2);
%     diff_y = mean((Data.BlockData.YrAvg{i}(id) - sho_y - Simulations{counter+i-1}(:,2)).^2);
%     
    diff_x = mean((Data.BlockData.X_avg{i}(id) - sho_x - Simulations{counter+i-1}(:,1)).^2);
    diff_y = mean((Data.BlockData.Y_avg{i}(id) - sho_y - Simulations{counter+i-1}(:,2)).^2);
    
    RMS(i) = sqrt( diff_x + diff_y );

end

end