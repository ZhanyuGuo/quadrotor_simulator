function [x, y] = controllerPID(x0, y0, x_d, y_d, t_step)
kp = 10.0; ki = 0.0; kd = 100.0;

k = 1;
x(k) = x0; y(k) = y0; vx(k) = 0; vy(k) = 0;
exi = 0; exp_last = 0; eyi = 0; eyp_last = 0;

ux = []; uy = [];
pts = size(x_d, 2);
for k = 1: pts
    exp = x_d(k) - x(k);
    eyp = y_d(k) - y(k);

    exi = exi + exp;
    eyi = eyi + eyp;

    exd = exp - exp_last;
    eyd = eyp - eyp_last;

    exp_last = exp;
    eyp_last = eyp;

    ux(k) = kp * exp + ki * exi + kd * exd;
    uy(k) = kp * eyp + ki * eyi + kd * eyd;

    vx(k + 1) = vx(k) + ux(k) * t_step;
    vy(k + 1) = vy(k) + uy(k) * t_step;

    x(k + 1) = x(k) + vx(k) * t_step + 0.5 * ux(k) * t_step .^2;
    y(k + 1) = y(k) + vy(k) * t_step + 0.5 * uy(k) * t_step .^2;
end
end
