function [x, y] = controllerLQR(x0, y0, x_d, y_d,vx_d,vy_d,ax_d,ay_d, t_step)
A = [1 0, t_step, 0; 0, 1, 0, t_step; 0, 0, 1, 0; 0, 0, 0, 1];
B = [0.5 * t_step .^2, 0; 0, 0.5 * t_step .^ 2; t_step, 0; 0, t_step];
Q = [1, 0, 0, 0; 0, 1, 0, 0; 0, 0, 1, 0; 0, 0, 0, 1];
R = [0.1, 0; 0, 0.1];
[K, S] = dlqr(A, B, Q, R);
k = 1;
x(k) = x0; y(k) = y0; vx(k) = 0; vy(k) = 0;

pts = size(x_d, 2);
for k = 1: pts
    state_now = [x(k); y(k); vx(k); vy(k)];
    state_des = [x_d(k); y_d(k); vx_d(k); vy_d(k)];
    a = -K * (state_now - state_des) + [ax_d(k); ay_d(k)];
    state_next = A * state_now + B * a;
    x(k+1)  = state_next(1);
    y(k+1)  = state_next(2);
    vx(k+1) = state_next(3);
    vy(k+1) = state_next(4);
end
end
