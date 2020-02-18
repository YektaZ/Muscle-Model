% compute the perp error for data.

%interpolate 

function PE = PerpError_data_indv_fun(Data)

PE = zeros(1,4);

for i = 1:4
    
%     id0_data = find(Data.AVG_Data.Time{i}>0, 1);
%     all_sub_id = Data.AVG_Data.X_Cnt{i}>9;
%     id = id0_data:sum(all_sub_id);
    Xr_sim = [Data.AVG_Data.X{i}, Data.AVG_Data.Y{i}];
    
    xstart = Xr_sim(1,:);

    %%%%%%%%%%%%%%% 
    
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