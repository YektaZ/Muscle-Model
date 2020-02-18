% compute the normalized path length for four simulated reaches.

%interpolate (first interpolate so the synce looks better)

function NL = NPathLength_data_fun(Data)

NL = zeros(1,4);

for i = 1:4
    
    id0_data = find(Data.BlockData.Time{i}>0, 1);
    all_sub_id = Data.BlockData.X_Cnt{i}>=Data.BlockData.X_Cnt{i}(1);
    id = id0_data:sum(all_sub_id);
    Xr_sim = [Data.BlockData.X_avg{i}(id), Data.BlockData.Y_avg{i}(id)]; %?
    
    xstart = Xr_sim(1,:);
    xfinish = Xr_sim(end,:);
    dis = xfinish-xstart;
    dis_norm = norm(dis); % or 0.2 always? 
    
    numpt = length(Xr_sim);
    
    %%%%%%%%%%%%%%% normalized path length
    
    for j = 1:numpt-1
        
        dx = Xr_sim(j,1) - Xr_sim(j+1,1);
        dy = Xr_sim(j,2) - Xr_sim(j+1,2);
        
        NL(i) = NL(i)+ sqrt(dx^2 + dy^2);
        
    end
    
    NL(i) = NL(i)/dis_norm;
    
end



end