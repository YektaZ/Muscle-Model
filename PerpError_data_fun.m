% compute the perp error for data.

%interpolate 

function PE = PerpError_data_fun(Data)

PE = zeros(1,4);

for i = 1:4
    
    id0_data = find(Data.BlockData.Time{i}>0, 1);
    all_sub_id = Data.BlockData.X_Cnt{i}>=Data.BlockData.X_Cnt{i}(1);
    id = id0_data:sum(all_sub_id);
    Xr_sim = [Data.BlockData.X_avg{i}(id), Data.BlockData.Y_avg{i}(id)];
    
    xstart = Xr_sim(1,:);

    %%%%%%%%%%%%%%% PE
    
    if mod(i,2)==1
       temp = Xr_sim(:,2) - xstart(2);
       [~, id] = max(abs(temp));
       PE(i) =  max(temp(id));
    else
        temp = Xr_sim(:,1) - xstart(1);
        [~, id] = max(abs(temp));
        PE(i) =  max(temp(id));
    end
    
end



end