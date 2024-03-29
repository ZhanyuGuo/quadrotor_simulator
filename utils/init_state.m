function s = init_state(start)
% @brief: initialize 13 x 1 state vector

phi0   = 0.0;
theta0 = 0.0;
psi0   = 0.0;
bRw0   = RPYtoRot_ZXY(phi0, theta0, psi0); 
Quat0  = RotToQuat(bRw0);

s     = zeros(13, 1);  % state
s(1)  = start(1);      % x
s(2)  = start(2);      % y
s(3)  = start(3);      % z
s(4)  = 0;             % dx
s(5)  = 0;             % dy
s(6)  = 0;             % dz
s(7)  = Quat0(1);      % qw
s(8)  = Quat0(2);      % qx
s(9)  = Quat0(3);      % qy
s(10) = Quat0(4);      % qz
s(11) = 0;             % p
s(12) = 0;             % q
s(13) = 0;             % r
end
