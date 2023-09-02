function [poly_coef_x, poly_coef_y, poly_coef_z, ts, n_order, n_seg] = getMinimumSnap3(waypoints, t_M)
% @brief: get 3d minimum-snap trajectory
n_order = 7;                        % order of poly
n_seg   = size(waypoints, 1) - 1;   % segment number
ts      = zeros(n_seg, 1);          % time distribution

% calculate time distribution in proportion to distance between 2 points
distance = zeros(n_seg, 1);
distance_sum = 0;
for i = 1: n_seg
    distance(i) = dist(waypoints(i + 1, :), waypoints(i, :));
    distance_sum = distance_sum + distance(i);
end

for i = 1: n_seg
    ts(i) = t_M * distance(i) / distance_sum;
end

poly_coef_x = minimumSnapQPSolver(waypoints(:, 1), ts, n_seg, n_order);
poly_coef_y = minimumSnapQPSolver(waypoints(:, 2), ts, n_seg, n_order);
poly_coef_z = minimumSnapQPSolver(waypoints(:, 3), ts, n_seg, n_order);

    function poly_coef = minimumSnapQPSolver(waypoints, ts, n_seg, n_order)
        start_cond = [waypoints(1), 0, 0, 0];
        end_cond   = [waypoints(end), 0, 0, 0];

        Q = getQ(n_seg, n_order, ts);
        [Aeq, beq] = getAbeq(n_seg, n_order, waypoints, ts, start_cond, end_cond);

        f = zeros(size(Q, 1), 1);
        poly_coef = quadprog(Q, f, [], [], Aeq, beq);
    end
end
