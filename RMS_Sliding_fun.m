%
% compute the RMS error between Data and 4 simulated reaches

function RMS = RMS_Sliding_fun(Simulations,Data, counter)

global sho_x sho_y

numpt = length(Simulations{1});

RMS = zeros(1,4);

for i = 1:4
    
    N = sum(Data.BlockData.X_Cnt{i}>9);
    
    
    %%%%%%%%%%%%%%%%%     interpolate
    
    %     Speed_sim = sqrt(Simulations{counter+i-1}.Sol.X(:,3).^2 + Simulations{counter+i-1}.Sol.X(:,4).^2);
    
    T = Simulations{i}.Sol.t;
    TT = Data.BlockData.Time{i}(51:N);
    dt = TT(2)-TT(1);
    
    count = 1;
    for j = 0.001:dt:T(end)
        %         Speed_temp(count) = interp1(T,Speed_sim,j);
        X_temp(count) = interp1(T,Simulations{counter+i-1}.Sol.X(:,1),j);
        Y_temp(count) = interp1(T,Simulations{counter+i-1}.Sol.X(:,2),j);
        count = count + 1;
    end
    
    
    id0_data = [101:20:400]; % 300*0.001 = 0.3 ms time window
    id0_sim =  [1:20:300];% Keep it this way
    
    numpt = length(X_temp);
    
    error = 10000;
    
    for k = id0_data
        for m = id0_sim
            
            id_data = [k:N];
            id_sim = [m:numpt];
            
            l = min(length(id_data), length(id_sim));
            id_data = [k:k+l-1];
            id_sim = [m:m+l-1];
            
            
            %
            diff_x = mean((Data.BlockData.X_avg{i}(id_data) - sho_x - X_temp(id_sim)').^2);
            diff_y = mean((Data.BlockData.Y_avg{i}(id_data) - sho_y - Y_temp(id_sim)').^2);
            
            error_temp = sqrt( diff_x + diff_y );
            
            if error_temp<error
                error = error_temp;
            end
            
        end
    end
    
    RMS(i) = error;
end

end