%% test quadrotor trajectory time allocation

%% clear before running
close all; clear; clc;

%% add path
addpath(genpath('./trajectory_generation/'));

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

f1 = figure(1); set(f1, 'position', figure_position);
axis([-5, 5, -5, 5]); grid on; hold on;

t_step   = 0.01;  % time step
t_M      = 10;    % total time
show_all = true;  % show all trajectory

%% main process
% set points
waypoints = setPoints(f1);

% get minimum snap trajectory
[poly_coef_x, poly_coef_y, ts, n_order, n_seg] = getMinimumSnap(waypoints, t_M);

% extract from polynomial
k = 1;
for i = 0: n_seg - 1
    Pxi = flipud(poly_coef_x((n_order + 1) * i + 1: (n_order + 1) * i + n_order + 1));
    Pyi = flipud(poly_coef_y((n_order + 1) * i + 1: (n_order + 1) * i + n_order + 1));
    for t = 0: t_step: ts(i + 1)
        x_des(k) = polyval(Pxi, t);
        y_des(k) = polyval(Pyi, t);
        k = k + 1;
    end
end

% plot desired trajectory
trj_1 = plot(x_des, y_des, 'Color', 'g', 'LineWidth', 2);

% get minimum snap trajectory
[poly_coef_x, poly_coef_y, ts, n_order, n_seg] = getMinimumSnapUnit(waypoints);

clear x_des y_des;
% extract from polynomial
k = 1;
for i = 0: n_seg - 1
    Pxi = flipud(poly_coef_x((n_order + 1) * i + 1: (n_order + 1) * i + n_order + 1));
    Pyi = flipud(poly_coef_y((n_order + 1) * i + 1: (n_order + 1) * i + n_order + 1));
    for t = 0: t_step: ts(i + 1)
        x_des(k) = polyval(Pxi, t);
        y_des(k) = polyval(Pyi, t);
        k = k + 1;
    end
end

% plot desired trajectory
hold on;
trj_2 = plot(x_des, y_des, 'Color', 'r', 'LineWidth', 2);
hold off;

legend([trj_1, trj_2], ["Spatial-Temporal", "Spatial"]);
