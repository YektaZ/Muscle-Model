% compute the enclosed area between the data and the simulations

%first rotate
%interpolate (first interpolate so the synce looks better)
%sync
%get the area

function EArea = GapArea_fun(Data,Simulations, Targets0, TargetsF)

global sho_x sho_y

EArea = zeros(1,4);

for i = 1:4
    
    %%%%%%%%%%%%%%% Rotation matrix
    xstart = Targets0(i,:); %Simulations{i}.Parameters.xstart(1:2)';
    xfinish = TargetsF(i,:); %Simulations{i}.Parameters.xfinish(1:2)';
    dis = xfinish - xstart;
    %
    theta = atan2(dis(2),dis(1));
    R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    Xr_sim = Simulations{i}.Sol.X(:,1:2)*R; % - xstart*R; %shift to the origin
    Xr_data = [Data.BlockData.X_avg{i}, Data.BlockData.Y_avg{i}]*R;
    
    shoulder_pos = [sho_x, sho_y]*R;
    
    %%%%%%%%%%%%%%% Interpolate
    
    id = Data.BlockData.X_Cnt{i}>=Data.BlockData.X_Cnt{i}(1);
    N = length(Data.BlockData.X_avg{i}(id));
    
    Speed_sim = sqrt(Simulations{i}.Sol.X(:,3).^2 + Simulations{i}.Sol.X(:,4).^2);
    
    T = Simulations{i}.Sol.t;
    TT = Data.BlockData.Time{i};
    dt = TT(2)-TT(1);
    
    count = 1;
    for j = 0.001:dt:T(end)
        Speed_temp(count) = interp1(T,Speed_sim,j);
        Y_temp(count) = interp1(T,Xr_sim(:,2),j);
        count = count + 1;
    end
    
    Y_temp = Y_temp(:);
    
    %%%%%%%%%%%%%%% Sync
    id0_data = find(Data.BlockData.Time{i}>0, 1);
    id0_sim = find(Speed_temp>0.05, 1);
    
%     numpt = length(Speed_temp(id0_sim:end));
%     id = floor(linspace(id0_data, N, numpt));
    
    numpt = length(Speed_temp);
    id_sim = floor(linspace(id0_sim, numpt, 500));
    id_data = floor(linspace(id0_data, N, 500));
    
    %%%%%%%%%%%%%%% Compute the gap
    EArea(i) = sum(abs((Xr_data(id_data,2)- shoulder_pos(2)  - Y_temp(id_sim))*dt));
    
end



end