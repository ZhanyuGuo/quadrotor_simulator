%% test quadrotor motion equation

%% clear before running
close all; clear; clc;

%% add path
addpath(genpath('./model'), genpath('./utils'));

%% initialization
% time
time   = 0;      % current time
t_step = 0.002;  % time step for solving equations of motion
c_step = 0.01;   % time step for controller
t_M    = 10;     % total simulate time

% parameters
params = quadModel_readonly();

% state
start  = [0; 0; 0];
true_s = init_state(start);

% input
F = params.mass * params.grav;
M = [0; 0; 0];

% External disturbance
params.Fd = zeros(3, 1);
params.Md = zeros(3, 1);

t_list = [];  % time span
s_list = [];  % state
u_list = [];  % input
d_list = [];  % disturbance
a_list = [];  % angle

%% simulation
disp('Start Simulation ...');
while (time <= t_M)
    % impuse F
    % if time >= t_M / 2 && time <= t_M / 2 + c_step
    %     F = 1e6;
    % else
    %     F = params.mass * params.grav;
    % end

    % step F
    % if time >= t_M / 2
    %     F = 1.1 * params.mass * params.grav;
    % else
    %     F = params.mass * params.grav;
    % end

    % sin F
    % F = 0.5 * sin(time) + params.mass * params.grav;

    % impuse Mx
    % if time >= t_M / 2 && time <= t_M / 2 + c_step
    %     M(1) = 5;
    % else
    %     M(1) = 0;
    % end

    % step Mx
    % if time >= t_M / 2
    %     M(1) = 0.005;
    % else
    %     M(1) = 0;
    % end

    % sin Mx
    % M(1) = 0.01 * sin(time);

    % impuse Mz
    % if time >= t_M / 2 && time <= t_M / 2 + c_step
    %     M(3) = 5;
    % else
    %     M(3) = 0;
    % end

    % step Mz
    % if time >= t_M / 2
    %     M(3) = 0.01;
    % else
    %     M(3) = 0;
    % end

    % sin Mz
    % M(3) = 0.01 * sin(time);

    % impuse Fdx
    % if time >= t_M / 2 && time <= t_M / 2 + c_step
    %     params.Fd(1) = 1e6;
    % else
    %     params.Fd(1) = 0;
    % end

    % step Fdx
    % if time >= t_M / 2
    %     params.Fd(1) = 1;
    % else
    %     params.Fd(1) = 0;
    % end

    % sin Fdx
    params.Fd(1) = 0.5 * sin(time);

    timeint = time: t_step: time + c_step;
    [~, xsave] = ode45(@(t, s) quadEOM_readonly(t, s, F, M, params), timeint', true_s);
    true_s = xsave(end, :)';

    t_list = [t_list; time];
    s_list = [s_list; true_s'];
    u_list = [u_list; F, M'];
    d_list = [d_list; params.Fd', params.Md'];
    a = RotToRPY_ZXY_wrapper(QuatToRot(true_s(7: 10)))';
    a_list = [a_list; a];

    time = time + c_step;
end
disp('Finish Simulation ...');

%% visualization
f1 = figure(1); sgtitle('Quadrotor Simulation');
f1.WindowState = 'maximized';
subplot(4, 2, 1), hold on, grid on;
plot(t_list, s_list(:, 1), 'Color', 'r', 'LineWidth', 2);
plot(t_list, s_list(:, 2), 'Color', 'g', 'LineWidth', 2);
plot(t_list, s_list(:, 3), 'Color', 'b', 'LineWidth', 2);
xlabel('t (s)'), ylabel('Position (m)');
legend('x', 'y', 'z', 'Location','best');
hold off;

subplot(4, 2, 2), hold on, grid on;
plot(t_list, u_list(:, 1), 'LineWidth', 2);
xlabel('t (s)'), ylabel('Force (N)');
legend('F', 'Location', 'best');
hold off;

subplot(4, 2, 3), hold on, grid on;
plot(t_list, s_list(:, 4), 'Color', 'r', 'LineWidth', 2);
plot(t_list, s_list(:, 5), 'Color', 'g', 'LineWidth', 2);
plot(t_list, s_list(:, 6), 'Color', 'b', 'LineWidth', 2);
xlabel('t (s)'), ylabel('Velocity (m/s)');
legend('vx', 'vy', 'vz', 'Location', 'best');
hold off;

subplot(4, 2, 4), hold on, grid on;
plot(t_list, u_list(:, 2), 'Color', 'r', 'LineWidth', 2);
plot(t_list, u_list(:, 3), 'Color', 'g', 'LineWidth', 2);
plot(t_list, u_list(:, 4), 'Color', 'b', 'LineWidth', 2);
xlabel('t (s)'), ylabel('Moment (N*m)');
legend('Mx', 'My', 'Mz', 'Location', 'best');
hold off;

subplot(4, 2, 5), hold on, grid on;
plot(t_list, a_list(:, 1), 'Color', 'r', 'LineWidth', 2);
plot(t_list, a_list(:, 2), 'Color', 'g', 'LineWidth', 2);
plot(t_list, a_list(:, 3), 'Color', 'b', 'LineWidth', 2);
xlabel('t (s)'), ylabel('Angle (rad)');
legend('phi', 'theta', 'psi', 'Location', 'best');
hold off;

subplot(4, 2, 6), hold on, grid on;
plot(t_list, d_list(:, 1), 'Color', 'r', 'LineWidth', 2);
plot(t_list, d_list(:, 2), 'Color', 'g', 'LineWidth', 2);
plot(t_list, d_list(:, 3), 'Color', 'b', 'LineWidth', 2);
xlabel('t (s)'), ylabel('Disturbe Force (N)');
legend('Fdx', 'Fdy', 'Fdz', 'Location', 'best');
hold off;

subplot(4, 2, 7), hold on, grid on;
plot(t_list, s_list(:, 11), 'Color', 'r', 'LineWidth', 2);
plot(t_list, s_list(:, 12), 'Color', 'g', 'LineWidth', 2);
plot(t_list, s_list(:, 13), 'Color', 'b', 'LineWidth', 2);
xlabel('t (s)'), ylabel('Angular Velocity (rad/s)');
legend('dphi', 'dtheta', 'dpsi', 'Location', 'best');
hold off;

subplot(4, 2, 8), hold on, grid on;
plot(t_list, d_list(:, 4), 'Color', 'r', 'LineWidth', 2);
plot(t_list, d_list(:, 5), 'Color', 'g', 'LineWidth', 2);
plot(t_list, d_list(:, 6), 'Color', 'b', 'LineWidth', 2);
xlabel('t (s)'), ylabel('Disturbe Moment (N*m)');
legend('Mdx', 'Mdy', 'Mdz', 'Location', 'best');
hold off;

% configurations
display_ratio   = 1.0;
figure_width    = 1920 / display_ratio;
figure_height   = 1080 / display_ratio;
figure_size     = 800 / display_ratio;
figure_position = [
    0.5 * (figure_width - figure_size), ...
    0.5 * (figure_height - figure_size), ...
    figure_size, ...
    figure_size];

f2 = figure(2); set(f2, 'position', figure_position);
grid on; hold on; view(45, 45); axis vis3d;

vis_init = false;
ifov = 90;  % Camera field of view

for i = 1:size(s_list, 1)
    true_s = s_list(i, :)';
    ll = 0.175;
    rr = 0.1;
    ff = 0.3;
    nprop = 40;
    propangs = linspace(0, 2 * pi, nprop);
    tR = QuatToRot(true_s(7: 10))';
    tpoint1 = tR * [ll; 0; 0];
    tpoint2 = tR * [0; ll; 0];
    tpoint3 = tR * [-ll; 0; 0];
    tpoint4 = tR * [0; -ll; 0];
    tproppts = rr * tR * [cos(propangs); sin(propangs); zeros(1, nprop)];
    twp1 = true_s(1: 3) + tpoint1;
    twp2 = true_s(1: 3) + tpoint2;
    twp3 = true_s(1: 3) + tpoint3;
    twp4 = true_s(1: 3) + tpoint4;
    tprop1 = tproppts + twp1 * ones(1, nprop);
    tprop2 = tproppts + twp2 * ones(1, nprop);
    tprop3 = tproppts + twp3 * ones(1, nprop);
    tprop4 = tproppts + twp4 * ones(1, nprop);
    tfov0 = true_s(1: 3);
    tfov1 = tR * [ff;  ff * tan(ifov * pi / 180 / 2);  ff * tan(ifov * pi / 180 / 2)] + true_s(1: 3);
    tfov2 = tR * [ff;  ff * tan(ifov * pi / 180 / 2); -ff * tan(ifov * pi / 180 / 2)] + true_s(1: 3);
    tfov3 = tR * [ff; -ff * tan(ifov * pi / 180 / 2); -ff * tan(ifov * pi / 180 / 2)] + true_s(1: 3);
    tfov4 = tR * [ff; -ff * tan(ifov * pi / 180 / 2);  ff * tan(ifov * pi / 180 / 2)] + true_s(1: 3);

    if ~vis_init
        vis_init = true;
        thtraj = plot3(true_s(1), true_s(2), true_s(3), 'b-', 'LineWidth', 3);
        tharm1 = line([twp1(1), twp3(1)], [twp1(2), twp3(2)], [twp1(3), twp3(3)], 'Color', 'b');
        tharm2 = line([twp2(1), twp4(1)], [twp2(2), twp4(2)], [twp2(3), twp4(3)], 'Color', 'b');
        thprop1 = plot3(tprop1(1, :), tprop1(2, :), tprop1(3, :), 'r-');
        thprop2 = plot3(tprop2(1, :), tprop2(2, :), tprop2(3, :), 'b-');
        thprop3 = plot3(tprop3(1, :), tprop3(2, :), tprop3(3, :), 'b-');
        thprop4 = plot3(tprop4(1, :), tprop4(2, :), tprop4(3, :), 'b-');
        thfov1 = line([tfov0(1) tfov1(1) tfov2(1)], [tfov0(2) tfov1(2) tfov2(2)], [tfov0(3) tfov1(3) tfov2(3)], 'Color', 'k');
        thfov2 = line([tfov0(1) tfov2(1) tfov3(1)], [tfov0(2) tfov2(2) tfov3(2)], [tfov0(3) tfov2(3) tfov3(3)], 'Color', 'k');
        thfov3 = line([tfov0(1) tfov3(1) tfov4(1)], [tfov0(2) tfov3(2) tfov4(2)], [tfov0(3) tfov3(3) tfov4(3)], 'Color', 'k');
        thfov4 = line([tfov0(1) tfov4(1) tfov1(1)], [tfov0(2) tfov4(2) tfov1(2)], [tfov0(3) tfov4(3) tfov1(3)], 'Color', 'k');
    else
        set(thtraj, 'XData', [get(thtraj, 'XData') true_s(1)]);
        set(thtraj, 'YData', [get(thtraj, 'YData') true_s(2)]);
        set(thtraj, 'ZData', [get(thtraj, 'ZData') true_s(3)]);
        set(thprop1, 'XData', tprop1(1, :));
        set(thprop1, 'YData', tprop1(2, :));
        set(thprop1, 'ZData', tprop1(3, :));
        set(thprop2, 'XData', tprop2(1, :));
        set(thprop2, 'YData', tprop2(2, :));
        set(thprop2, 'ZData', tprop2(3, :));
        set(thprop3, 'XData', tprop3(1, :));
        set(thprop3, 'YData', tprop3(2, :));
        set(thprop3, 'ZData', tprop3(3, :));
        set(thprop4, 'XData', tprop4(1, :));
        set(thprop4, 'YData', tprop4(2, :));
        set(thprop4, 'ZData', tprop4(3, :));
        set(tharm1, 'XData', [twp1(1), twp3(1)]);
        set(tharm1, 'YData', [twp1(2), twp3(2)]);
        set(tharm1, 'ZData', [twp1(3), twp3(3)]);
        set(tharm2, 'XData', [twp2(1), twp4(1)]);
        set(tharm2, 'YData', [twp2(2), twp4(2)]);
        set(tharm2, 'ZData', [twp2(3), twp4(3)]);
        set(thfov1, 'XData', [tfov0(1) tfov1(1) tfov2(1)]);
        set(thfov1, 'YData', [tfov0(2) tfov1(2) tfov2(2)]);
        set(thfov1, 'ZData', [tfov0(3) tfov1(3) tfov2(3)]);
        set(thfov2, 'XData', [tfov0(1) tfov2(1) tfov3(1)]);
        set(thfov2, 'YData', [tfov0(2) tfov2(2) tfov3(2)]);
        set(thfov2, 'ZData', [tfov0(3) tfov2(3) tfov3(3)]);
        set(thfov3, 'XData', [tfov0(1) tfov3(1) tfov4(1)]);
        set(thfov3, 'YData', [tfov0(2) tfov3(2) tfov4(2)]);
        set(thfov3, 'ZData', [tfov0(3) tfov3(3) tfov4(3)]);
        set(thfov4, 'XData', [tfov0(1) tfov4(1) tfov1(1)]);
        set(thfov4, 'YData', [tfov0(2) tfov4(2) tfov1(2)]);
        set(thfov4, 'ZData', [tfov0(3) tfov4(3) tfov1(3)]);
    end
    drawnow;
end
