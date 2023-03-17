function sdot = quadEOM_readonly(t, s, F, M, params)
%%
% Solve quadrotor equation of motion, used by the ode solver.
%
% INPUTS:
% t      -  1 x 1, time
% s      - 13 x 1, state vector = [x, y, z, xd, yd, zd, qw, qx, qy, qz, p, q, r]
% F      -  1 x 1, thrust output from controller (only used in simulation)
% M      -  3 x 1, moment output from controller (only used in simulation)
% params - struct, output from quadModel_readonly() and whatever parameters you want to pass in
%
% OUTPUTS:
% sdot   - 13 x 1, derivative of state vector s
%%
    armlength = params.armlength;

    M = M + params.Md;

    % Limit the force and moments due to actuator limits
    A = [0.25,              0, -0.5/armlength; ...
         0.25,  0.5/armlength,              0; ...
         0.25,              0,  0.5/armlength; ...
         0.25, -0.5/armlength,              0];

    prop_thrusts = A*[F; M(1:2)]; % Not using moment about Z-axis for limits
    prop_thrusts_clamped = max(min(prop_thrusts, params.maxF/4), params.minF/4);

    B = [         1,         1,         1,          1; ...
                  0, armlength,         0, -armlength; ...
         -armlength,         0, armlength,          0];
    F = B(1, :)*prop_thrusts_clamped;
    M = [B(2:3, :)*prop_thrusts_clamped; M(3)];

    % Assign states
    x  = s(1);
    y  = s(2);
    z  = s(3);
    dx = s(4);
    dy = s(5);
    dz = s(6);
    qW = s(7);
    qX = s(8);
    qY = s(9);
    qZ = s(10);
    p  = s(11);
    q  = s(12);
    r  = s(13);

    quat = [qW; qX; qY; qZ];
    bRw = QuatToRot(quat);
    wRb = bRw';

    % Acceleration
    acc = 1/params.mass*(wRb*([0; 0; F] + params.Fd) - [0; 0; params.mass*params.grav]);

    % Angular velocity
    K_quat = 2; % this enforces the magnitude 1 constraint for the quaternion
    quaterror = 1 - (qW^2 + qX^2 + qY^2 + qZ^2);
    dq = -1/2*[0, -p, -q, -r; ...
               p,  0, -r,  q; ...
               q,  r,  0, -p; ...
               r, -q,  p,  0]*quat + K_quat*quaterror*quat;

    % Angular acceleration
    omega = [p; q; r];
    dpqr = params.invI*(M - cross(omega, params.I*omega));

    % Assemble sdot
    sdot = zeros(13,1);
    sdot(1)  = dx;
    sdot(2)  = dy;
    sdot(3)  = dz;
    sdot(4)  = acc(1);
    sdot(5)  = acc(2);
    sdot(6)  = acc(3);
    sdot(7)  = dq(1);
    sdot(8)  = dq(2);
    sdot(9)  = dq(3);
    sdot(10) = dq(4);
    sdot(11) = dpqr(1);
    sdot(12) = dpqr(2);
    sdot(13) = dpqr(3);
end