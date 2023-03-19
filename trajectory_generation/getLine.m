function [poly_coef_x, poly_coef_y, ts] = getLine(waypoints, t_M)
n_seg = size(waypoints, 1) - 1;  % segment number
ts    = zeros(n_seg, 1);         % time distribution

% calculate time distribution in proportion to distance between 2 points
distance = zeros(n_seg, 1);
distance_sum = 0;
for i = 1: n_seg
    distance(i) = dist(waypoints(i + 1), waypoints(i)');
    distance_sum = distance_sum + distance(i);
end

for i = 1: n_seg
    ts(i) = distance(i) / distance_sum * t_M;
end

poly_coef_x = lineSolver(waypoints(:, 1), ts, n_seg);
poly_coef_y = lineSolver(waypoints(:, 2), ts, n_seg);

    function poly_coef = lineSolver(waypoints, ts, n_seg)
        poly_coef = zeros(n_seg * 2, 1);
        for ii = 0: n_seg - 1
            poly_coef(ii * 2 + 1) = waypoints(ii + 1);
            poly_coef(ii * 2 + 2) = (waypoints(ii + 2) - waypoints(ii + 1)) / ts(ii + 1);
        end
    end
end
