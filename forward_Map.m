
%
% map joint angles and velocities forward to Cartesian coordinates of hand
%

function X = forward_Map(Xsi, a1, a2)

X(1:2,1) = forward_Position_2(Xsi(1:2),a1, a2);
X(3:4,1) = forward_Velocity(Xsi(1:2), Xsi(3:4), a1, a2);


