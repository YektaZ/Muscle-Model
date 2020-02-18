
% This metric computes multiple errors between two paths to find the best
% one

% The starting time of the reach is moved for a certain time window.

function Error = RMS_Sliding_indv_fun(Simulations, Data)

global sho_x sho_y

Error = zeros(1,4);

for i = 1:4
    
    N = length(Data.AVG_Data.X{i}); 
    %%%%%%%%%%%%%%%%%     interpolate
    
    Speed_sim = sqrt(Simulations{i}.Sol.X(:,3).^2 + Simulations{i}.Sol.X(:,4).^2);
    
    T = Simulations{i}.Sol.t;
    TT = Data.AVG_Data.Time{i}(51:N);
    dt = TT(2)-TT(1);
    
    T_temp = [];
    count = 1;
    for j = 0.001:dt:T(end)
        Speed_temp(count) = interp1(T,Speed_sim,j);
        X_temp(count) = interp1(T,Simulations{i}.Sol.X(:,1),j);
        Y_temp(count) = interp1(T,Simulations{i}.Sol.X(:,2),j);
        T_temp = [T_temp, j];
        count = count + 1;
    end
    
    numpt = length(X_temp); % numpt for simulations
    
    id0_data = [101:20:400]; % 300*0.001 = 0.3 ms time window
    id0_sim =  [1:20:300];% Keep it this way
    
    error = 10000;
    
    for k = id0_data
        for m = id0_sim

            id_data = [k:N];
            id_sim = [m:numpt];
            
            l = min(length(id_data), length(id_sim));
            id_data = [k:k+l-1];
            id_sim = [m:m+l-1];

            diff_x = mean((Data.AVG_Data.X{i}(id_data) - sho_x - X_temp(id_sim)').^2);
            diff_y = mean((Data.AVG_Data.Y{i}(id_data) - sho_y - Y_temp(id_sim)').^2);
            
            error_temp = sqrt( diff_x + diff_y );
            
            if error_temp<error
                error = error_temp;
%                 plot(Data.AVG_Data.Speed{i}(id_data) - sho_x, '--')
%                 plot(Speed_temp(id_sim))
%                 pause(0.05)
            end
            
            
        end
    end
    
%     display(k)
%     display(m)
    Error(i) = error;
    
end

end