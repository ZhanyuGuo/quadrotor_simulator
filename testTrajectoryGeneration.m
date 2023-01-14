clc; clear; close all;

% path
addpath(genpath('./trajectory_generation/'));

% configurations
display_ratio   = 1.25;
figure_width    = 1920 / display_ratio;
figure_height   = 1080 / display_ratio;
figure_size     =  800 / display_ratio;
figure_position = [
    0.5*(figure_width - figure_size), ...
    0.5*(figure_height - figure_size), ...
    figure_size, ...
    figure_size];

f1 = figure(1);
set(f1, 'position', figure_position, 'Renderer', 'painters');
axis([-5, 5, -5, 5]); grid on; hold on;

t_M         = 10;   % total time
t_step      = 0.01; % time step
show_all    = true; % show all trajectory

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

if show_all
    % get line trajectory
    [poly_coef_x, poly_coef_y, ts] = getLine(waypoints, t_M);
    
    clear x_des y_des;
    % extract from polynomial
    k = 1;
    for i = 0: n_seg - 1
        Pxi = flipud(poly_coef_x(i * 2 + 1: i * 2 + 2));
        Pyi = flipud(poly_coef_y(i * 2 + 1: i * 2 + 2));
        for t = 0: t_step: ts(i + 1)
            x_des(k) = polyval(Pxi, t);
            y_des(k) = polyval(Pyi, t);
            k = k + 1;
        end
    end
    
    % plot desired trajectory
    trj_2 = plot(x_des, y_des, 'Color', 'b', 'LineWidth', 2);
    
    % get lagrange trajectory
    [poly_coef_x, poly_coef_y, ta] = getPoly(waypoints, t_M);
    
    clear x_des y_des;
    % extract from polynomial
    k = 1;
    for t = 0: t_step: t_M
        x_des(k) = polyval(poly_coef_x, t);
        y_des(k) = polyval(poly_coef_y, t);
        k = k + 1;
    end
    
    % plot desired trajectory
    trj_3 = plot(x_des, y_des, 'Color', 'r', 'LineWidth', 2);

    legend([trj_1, trj_2, trj_3], ["Minimum Snap", "Line", "Polynomial"]);
end
