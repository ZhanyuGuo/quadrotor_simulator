%% test quadrotor trajectory generation and tracking in 2d (ablation on controller)

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
trj_ref = plot(x_des, y_des, 'Color', 'g', 'LineWidth', 2);

% PID controller
[x_pid, y_pid] = controllerPID(start_point(1), start_point(2), x_des, y_des, t_step);

% LQR controller
[x_lqr, y_lqr] = controllerLQR(start_point(1), start_point(2), x_des, y_des, dx_des, dy_des, ddx_des, ddy_des, t_step);

% video config
% v = VideoWriter('trajectory_tracking', 'MPEG-4');
% v.FrameRate = 1 / t_step;
% v.Quality = 100;
% open(v);

%% visualization
vis_init = false;
% draw the motion
for i = 1: min([size(x_pid, 2), size(x_lqr,2)])
    % for snapping
    %     if ~mod(i, 100)
    %         input('Enter to continue');
    %     end

    if ~vis_init
        vis_init = true;
        p_pid = scatter(x_pid(i), y_pid(i), 'filled', 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'bl');
        p_lqr = scatter(x_lqr(i), y_lqr(i), 'filled', 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'bl');
        trj_pid = plot(x_pid(i), y_pid(i), 'Color', 'r', 'LineStyle', ':', 'LineWidth', 2);
        trj_lqr = plot(x_lqr(i), y_lqr(i), 'Color', 'b', 'LineStyle', ':', 'LineWidth', 2);
        legend([trj_ref, trj_pid, trj_lqr], {'Reference', 'PID controller', 'LQR controller'})
    else
        set(p_pid, 'XData', x_pid(i));
        set(p_pid, 'YData', y_pid(i));
        set(p_lqr, 'XData', x_lqr(i));
        set(p_lqr, 'YData', y_lqr(i));
        set(trj_pid, 'XData', [get(trj_pid, 'XData') x_pid(i)]);
        set(trj_pid, 'YData', [get(trj_pid, 'YData') y_pid(i)]);
        set(trj_lqr, 'XData', [get(trj_lqr, 'XData') x_lqr(i)]);
        set(trj_lqr, 'YData', [get(trj_lqr, 'YData') y_lqr(i)]);
    end
    drawnow;
    %     frame = getframe;
    %     writeVideo(v, frame);
end

% end of video
% close(v);

pts = size(x_des, 2);
t_span = linspace(0, t_M, pts);
figure(2);

subplot(2, 2, 1);
axis([0, t_M, 0, 100]); grid on; hold on;
xlabel('Time (s)');
ylabel('X position (m)');
plot(t_span, x_des, 'Color', 'g');
plot(t_span, x_pid(1:pts), 'Color', 'r');
plot(t_span, x_lqr(1:pts), 'Color', 'b');
legend({'Reference', 'PID controller', 'LQR controller'}, 'Location', 'best');

subplot(2, 2, 2);
axis([0, t_M, 0, 100]); grid on; hold on;
xlabel('Time (s)');
ylabel('Y position (m)');
plot(t_span, y_des, 'Color', 'g');
plot(t_span, y_pid(1:pts), 'Color', 'r');
plot(t_span, y_lqr(1:pts), 'Color', 'b');
legend({'Reference', 'PID controller', 'LQR controller'}, 'Location', 'best');

subplot(2, 2, 3);
axis([0, t_M, -10, 10]); grid on; hold on;
xlabel('Time (s)');
ylabel('X error (m)');
plot(t_span, x_des - x_pid(1:pts), 'Color', 'r');
plot(t_span, x_des - x_lqr(1:pts), 'Color', 'b');
legend({'PID controller', 'LQR controller'}, 'Location', 'best');

subplot(2, 2, 4);
axis([0, t_M, -10, 10]); grid on; hold on;
xlabel('Time (s)');
ylabel('Y error (m)');
plot(t_span, y_des - y_pid(1:pts), 'Color', 'r');
plot(t_span, y_des - y_lqr(1:pts), 'Color', 'b');
legend({'PID controller', 'LQR controller'}, 'Location', 'best');
