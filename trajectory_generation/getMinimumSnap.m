function [poly_coef_x, poly_coef_y, ts, n_order, n_seg] = getMinimumSnap(waypoints, t_M)
    n_order = 7;                        % order of poly
    n_seg   = size(waypoints, 1) - 1;   % segment number
    ts      = zeros(n_seg, 1);          % time distribution
    
    % calculate time distribution in proportion to distance between 2 points
    dist = zeros(n_seg, 1);
    dist_sum = 0;
    
    for i = 1: n_seg
        dist(i) = sqrt((waypoints(i + 1, 1) - waypoints(i, 1))^2 + ...
            (waypoints(i + 1, 2) - waypoints(i, 2))^2);
        dist_sum = dist_sum + dist(i);
    end
    
    for i = 1: n_seg
        ts(i) = dist(i) / dist_sum * t_M;
    end
    
    poly_coef_x = minimumSnapQPSolver(waypoints(:, 1), ts, n_seg, n_order);
    poly_coef_y = minimumSnapQPSolver(waypoints(:, 2), ts, n_seg, n_order);
    
    function poly_coef = minimumSnapQPSolver(waypoints, ts, n_seg, n_order)
        start_cond  = [waypoints(1), 0, 0, 0];
        end_cond    = [waypoints(end), 0, 0, 0];

        Q = getQ(n_seg, n_order, ts);
        [Aeq, beq] = getAbeq(n_seg, n_order, waypoints, ts, start_cond, end_cond);

        f = zeros(size(Q, 1), 1);
        poly_coef = quadprog(Q, f, [], [], Aeq, beq);
    end
end
