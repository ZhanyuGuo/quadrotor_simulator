%% test quadrotor trajectory generation and tracking in 2d (ablation on trajectory)

%% clear before running
close all; clear; clc;

%% add path
addpath(genpath('./trajectory_generation/'), genpath('./controller'));

%% configurations
display_ratio   = 1.0;
figure_width    = 1920 / display_ratio;
figure_height   = 1080 / display_ratio;
figure_size     =  800 / display_ratio;
figure_position = [
    0.5 * (figure_width - figure_size), ...
    0.5 * (figure_height - figure_size), ...
    figure_size, ...
    figure_size];

f1 = figure(1);
set(f1, 'position', figure_position);
axis([0 100 0 100]); grid on; hold on;

% time configurations
t_M    = 10;
t_step = 0.01;

%% main process
% set points
waypoints = setPoints(f1);
start_point = waypoints(1, :) + [5.0, -5.0];

% get minimum snap trajectory
[poly_coef_x, poly_coef_y, ts, n_order, n_seg] = getMinimumSnap(waypoints, t_M);

% extract from polynomial
k = 1;
for i = 0: n_seg - 1
    Pxi = flipud(poly_coef_x((n_order + 1) * i + 1: (n_order + 1) * i + n_order + 1));
    Pyi = flipud(poly_coef_y((n_order + 1) * i + 1: (n_order + 1) * i + n_order + 1));
    dPxi = polyder(Pxi);
    dPyi = polyder(Pyi);
    ddPxi = polyder(dPxi);
    ddPyi = polyder(dPyi);
    for t = 0: t_step: ts(i + 1)
        x_des(k) = polyval(Pxi, t);
        y_des(k) = polyval(Pyi, t);
        dx_des(k) = polyval(dPxi, t);
        dy_des(k) = polyval(dPyi, t);
        ddx_des(k) = polyval(ddPxi, t);
        ddy_des(k) = polyval(ddPyi, t);
        k = k + 1;
    end
end

% plot desired trajectory
trj_1 = plot(x_des, y_des, 'Color', 'g', 'LineWidth', 2);

% PID controller
% [x_1, y_1] = controllerPID(start_point(1), start_point(2), x_des, y_des, t_step);

% LQR controller
[x_1, y_1] = controllerLQR(start_point(1), start_point(2), x_des, y_des, dx_des, dy_des, ddx_des, ddy_des, t_step);

pts = size(x_des, 2);
ex_1 = x_des - x_1(1:pts);
ey_1 = y_des - y_1(1:pts);

% get line trajectory
clear x_des y_des dx_des dy_des ddx_des ddy_des;

[poly_coef_x, poly_coef_y, ts] = getLine(waypoints, t_M);

% extract
k = 1;
for i = 0: n_seg - 1
    Pxi = flipud(poly_coef_x(i * 2 + 1: i * 2 + 2));
    Pyi = flipud(poly_coef_y(i * 2 + 1: i * 2 + 2));
    dPxi = polyder(Pxi);
    dPyi = polyder(Pyi);
    ddPxi = polyder(dPxi);
    ddPyi = polyder(dPyi);
    for t = 0: t_step: ts(i + 1)
        x_des(k) = polyval(Pxi, t);
        y_des(k) = polyval(Pyi, t);
        dx_des(k) = polyval(dPxi, t);
        dy_des(k) = polyval(dPyi, t);
        ddx_des(k) = polyval(ddPxi, t);
        ddy_des(k) = polyval(ddPyi, t);
        k = k + 1;
    end
end

% plot desired trajectory
trj_2 = plot(x_des, y_des, 'Color', 'b', 'LineWidth', 2);

% PID controller
% [x_2, y_2] = controllerPID(start_point(1), start_point(2), x_des, y_des, t_step);

% LQR controller
[x_2, y_2] = controllerLQR(start_point(1), start_point(2), x_des, y_des, dx_des, dy_des, ddx_des, ddy_des, t_step);

pts = size(x_des, 2);
ex_2 = x_des - x_2(1:pts);
ey_2 = y_des - y_2(1:pts);

% get lagrange trajectory
clear x_des y_des dx_des dy_des ddx_des ddy_des;

[poly_coef_x, poly_coef_y, ta] = getPoly(waypoints, t_M);

% extract
k = 1;
dPxi = polyder(poly_coef_x);
dPyi = polyder(poly_coef_y);
ddPxi = polyder(dPxi);
ddPyi = polyder(dPyi);
for t = 0: t_step: t_M
    x_des(k) = polyval(poly_coef_x, t);
    y_des(k) = polyval(poly_coef_y, t);
    dx_des(k) = polyval(dPxi, t);
    dy_des(k) = polyval(dPyi, t);
    ddx_des(k) = polyval(ddPxi, t);
    ddy_des(k) = polyval(ddPyi, t);
    k = k + 1;
end

% plot desired trajectory
trj_3 = plot(x_des, y_des, 'Color', 'r', 'LineWidth', 2);

% PID controller
% [x_3, y_3] = controllerPID(start_point(1), start_point(2), x_des, y_des, t_step);

% LQR controller
[x_3, y_3] = controllerLQR(start_point(1), start_point(2), x_des, y_des, dx_des, dy_des, ddx_des, ddy_des, t_step);

pts = size(x_des, 2);
ex_3 = x_des - x_3(1:pts);
ey_3 = y_des - y_3(1:pts);

% video config
% v = VideoWriter('trajectory_tracking_LQR', 'MPEG-4');
% v.FrameRate = 1 / t_step;
% v.Quality = 100;
% open(v);

%% visualization
vis_init = false;
% draw the motion
for i = 1: min([size(x_1, 2), size(x_2, 2), size(x_3, 2)])
    % for snapping
    %     if ~mod(i, 100)
    %         input('Enter to continue');
    %     end

    if ~vis_init
        vis_init = true;
        p_1 = scatter(x_1(i), y_1(i), 'filled', 'MarkerFaceColor', 'g', 'MarkerEdgeColor', 'bl');
        p_2 = scatter(x_2(i), y_2(i), 'filled', 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'bl');
        p_3 = scatter(x_3(i), y_3(i), 'filled', 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'bl');
        trj_pid_1 = plot(x_1(i), y_1(i), 'Color', 'g', 'LineStyle', ':', 'LineWidth', 2);
        trj_pid_2 = plot(x_2(i), y_2(i), 'Color', 'b', 'LineStyle', ':', 'LineWidth', 2);
        trj_pid_3 = plot(x_3(i), y_3(i), 'Color', 'r', 'LineStyle', ':', 'LineWidth', 2);
        legend([trj_1, trj_2, trj_3], {'Minimum Snap', 'Line', 'Lagrange'})
    else
        set(p_1, 'XData', x_1(i));
        set(p_1, 'YData', y_1(i));
        set(p_2, 'XData', x_2(i));
        set(p_2, 'YData', y_2(i));
        set(p_3, 'XData', x_3(i));
        set(p_3, 'YData', y_3(i));
        set(trj_pid_1, 'XData', [get(trj_pid_1, 'XData') x_1(i)]);
        set(trj_pid_1, 'YData', [get(trj_pid_1, 'YData') y_1(i)]);
        set(trj_pid_2, 'XData', [get(trj_pid_2, 'XData') x_2(i)]);
        set(trj_pid_2, 'YData', [get(trj_pid_2, 'YData') y_2(i)]);
        set(trj_pid_3, 'XData', [get(trj_pid_3, 'XData') x_3(i)]);
        set(trj_pid_3, 'YData', [get(trj_pid_3, 'YData') y_3(i)]);
    end
    drawnow;
    %     frame = getframe;
    %     writeVideo(v, frame);
end

% input('enter');
% delete([trj_1, trj_2, trj_3]);

% end of video
% close(v);

pts = min([size(ex_1, 2), size(ex_2, 2), size(ex_3, 2),]);
t_span = linspace(0, t_M, pts);
figure(2);

subplot(2, 1, 1);
axis([0, t_M, -10, 10]); grid on; hold on;
xlabel('Time (s)');
ylabel('X error (m)');
plot(t_span, ex_1(1:pts), 'Color', 'g');
plot(t_span, ex_2(1:pts), 'Color', 'b');
plot(t_span, ex_3(1:pts), 'Color', 'r');
legend({'Minimum Snap', 'Line', 'Lagrange'}, 'Location', 'best');

subplot(2, 1, 2);
axis([0, t_M, -10, 10]); grid on; hold on;
xlabel('Time (s)');
ylabel('Y error (m)');
plot(t_span, ey_1(1:pts), 'Color', 'g');
plot(t_span, ey_2(1:pts), 'Color', 'b');
plot(t_span, ey_3(1:pts), 'Color', 'r');
legend({'Minimum Snap', 'Line', 'Lagrange'}, 'Location', 'best');
