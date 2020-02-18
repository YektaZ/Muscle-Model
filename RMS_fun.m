%
% compute the RMS error between Data and 4 simulated reaches

% sync data before comparison

function RMS = RMS_fun(Simulations,Data, counter)

global sho_x sho_y

numpt = length(Simulations{1}.Sol.X);

RMS = zeros(1,4);

% for i = 1:4
%     
% %     Speed_sim = sqrt(Simulations{counter+i-1}.Sol.X(:,3).^2 + Simulations{counter+i-1}.Sol.X(:,4).^2);
% %     id0_data = find(Data.BlockData.Speed_avg{i}>0.05, 1);
% %     id0_sim = find(Speed_sim>0.05, 1);
%     
%     N = length(Data.BlockData.X_avg{i});
% %     id = floor(linspace(id0_data, N, numpt-id0_sim+1));
%     id = floor(linspace(1, N, numpt));
%     
%     diff_x = mean((Data.BlockData.X_avg{i}(id) - sho_x - Simulations{counter+i-1}.Sol.X(:,1)).^2);
%     diff_y = mean((Data.BlockData.Y_avg{i}(id) - sho_y - Simulations{counter+i-1}.Sol.X(:,2)).^2);
%     
%     RMS(i) = sqrt( diff_x + diff_y );
% 
% end

for i = 1:4
    
    N = sum(Data.BlockData.XrCnt{i}>9);
    id = floor(linspace(1, N, numpt));

    %%%%%%%%%%%%%%%%%
    
    diff_x = mean((Data.BlockData.XrAvg{i}(id) - sho_x - Simulations{counter+i-1}.Sol.X(:,1)).^2);
    diff_y = mean((Data.BlockData.YrAvg{i}(id) - sho_y - Simulations{counter+i-1}.Sol.X(:,2)).^2);
    
    RMS(i) = sqrt( diff_x + diff_y );

end

end