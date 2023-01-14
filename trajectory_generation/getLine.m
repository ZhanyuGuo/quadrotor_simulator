function [poly_coef_x, poly_coef_y, ts] = getLine(waypoints, t_M)
    n_seg   = size(waypoints, 1) - 1;   % segment number
    ts      = zeros(n_seg, 1);          % time distribution
    
    % calculate time distribution in proportion to distance between 2 points
    dist = zeros(n_seg, 1);
    dist_sum = 0;
    
    for i = 1: n_seg
        dist(i) = sqrt((waypoints(i + 1, 1) - waypoints(i, 1))^2 + (waypoints(i + 1, 2) - waypoints(i, 2))^2);
        dist_sum = dist_sum + dist(i);
    end
    
    for i = 1: n_seg
        ts(i) = dist(i) / dist_sum * t_M;
    end
    
    poly_coef_x = lineSolver(waypoints(:, 1), ts, n_seg);
    poly_coef_y = lineSolver(waypoints(:, 2), ts, n_seg);
    
    function poly_coef = lineSolver(waypoints, ts, n_seg)
        poly_coef = zeros(n_seg * 2, 1);
        for i = 0: n_seg - 1
            poly_coef(i * 2 + 1) = waypoints(i + 1);
            poly_coef(i * 2 + 2) = (waypoints(i + 2) - waypoints(i + 1)) / ts(i + 1);
        end
    end
end
