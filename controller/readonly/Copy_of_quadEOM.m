function dsdt = Copy_of_quadEOM(t, s, F, M, Fd)
% F = params.mass * params.grav;
% M = [0; 0; 0];
% F = u_1;
% M = u_2;
% x0(1) % x
% x0(2) % y
% x0(3) % z
% x0(4) % xdot
% x0(5) % ydot
% x0(6) % zdot
% Quat0   = R_to_quaternion(ypr_to_R([yaw0 pitch0 roll0])');
% x0(7) % qw
% x0(8) % qx
% x0(9) % qy
% x0(10) % qz
% x0(11) % p
% x0(12) % q
% x0(13) % r
    global params;
    m = params.mass;
    g = params.grav;
    I = params.I;

    dsdt = zeros(13, 1);

    dsdt(1: 3) = s(4: 6);

    Quat = s(7: 10);
    R = QuatToRot(Quat);
    dsdt(4: 6) = (R * [0; 0; F] - [0; 0; m * g]) / m;
    
    p = s(11); q = s(12); r = s(13);
    ddadt = zeros(3, 1);
    ddadt(1) = (M(1) + q * r * (I(2, 2) - I(3, 3))) / I(1, 1);
    ddadt(2) = (M(2) + p * r * (I(3, 3) - I(1, 1))) / I(2, 2);
    ddadt(3) = (M(3) + p * q * (I(1, 1) - I(2, 2))) / I(3, 3);
    dsdt(11: 13) = ddadt;

    Q = [
        Quat(1), -Quat(2), -Quat(3), -Quat(4); ...
        Quat(2),  Quat(1), -Quat(4),  Quat(3); ...
        Quat(3),  Quat(4),  Quat(1), -Quat(2); ...
        Quat(4), -Quat(3),  Quat(2),  Quat(1)];

    dsdt(7: 10) = 1/2 * Q * [0; p; q; r];
end
