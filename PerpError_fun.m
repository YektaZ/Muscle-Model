% compute the normalized path length for four simulated reaches.

%interpolate (first interpolate so the synce looks better)

function PE = PerpError_fun(Simulations)

PE = zeros(1,4);

for i = 1:4
    
    Xr_sim = Simulations{i}.Sol.X(:,1:2);
    xstart = Xr_sim(1,:);
    
    %%%%%%%%%%%%%%% Interpolate
    
    T = Simulations{i}.Sol.t;
    dt = 0.001;
    
    count = 1;
    for j = 0.001:dt:T(end)
        X_temp(count) = interp1(T,Xr_sim(:,1),j);
        Y_temp(count) = interp1(T,Xr_sim(:,2),j);
        count = count + 1;
    end
    
    %%%%%%%%%%%%%%% normalized path length
    
    if mod(i,2)==1
        temp = Y_temp - xstart(2);
        [~, id] = max(abs(temp));
        PE(i) =  max(temp(id));
    else
        temp = X_temp - xstart(1);
        [~, id] = max(abs(temp));
        PE(i) =  max(temp(id));
    end
    
end



end