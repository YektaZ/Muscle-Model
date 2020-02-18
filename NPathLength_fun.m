% compute the normalized path length for four simulated reaches.

%interpolate (first interpolate so the synce looks better)

function NL = NPathLength_fun(Simulations)

NL = zeros(1,4);

for i = 1:4
    
    Xr_sim = Simulations{i}.Sol.X(:,1:2);
    
    xstart = Xr_sim(1,:);
    xfinish = Xr_sim(end,:);
    dis = xfinish-xstart;
    dis_norm = norm(dis); % 0.2 m
    
    %%%%%%%%%%%%%%% Interpolate
    
    T = Simulations{i}.Sol.t;
    dt = 0.001;
    
    count = 1;
    for j = 0.001:dt:T(end)
        %         Speed_temp(count) = interp1(T,Speed_sim,j);
        X_temp(count) = interp1(T,Xr_sim(:,1),j);
        Y_temp(count) = interp1(T,Xr_sim(:,2),j);
        count = count + 1;
    end
    
    numpt = length(X_temp);
    
    %%%%%%%%%%%%%%% normalized path length
    
    for j = 1:numpt-1
        
        dx = X_temp(j) - X_temp(j+1);
        dy = Y_temp(j) - Y_temp(j+1);
        
        NL(i) = NL(i)+ sqrt(dx^2 + dy^2);
        
    end
    
    NL(i) = NL(i)/dis_norm;
    
end



end