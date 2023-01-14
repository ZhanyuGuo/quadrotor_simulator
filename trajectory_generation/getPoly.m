function [poly_coef_x, poly_coef_y, ta] = getPoly(waypoints, t_M)
    n_seg   = size(waypoints, 1) - 1;   % segment number
    ta      = zeros(n_seg, 1);          % time distribution

    % calculate time distribution in proportion to distance between 2 points
    dist = zeros(n_seg, 1);
    dist_sum = 0;

    for i = 1: n_seg
        dist(i) = sqrt((waypoints(i + 1, 1) - waypoints(i, 1))^2 + (waypoints(i + 1, 2) - waypoints(i, 2))^2);
        dist_sum = dist_sum + dist(i);
    end
    
    ta(1) = dist(1) / dist_sum * t_M;
    for i = 2: n_seg
        ta(i) = dist(i) / dist_sum * t_M + ta(i - 1);
    end

    poly_coef_x = LagrangeSolver(waypoints(:, 1), [0; ta], n_seg + 1);
    poly_coef_y = LagrangeSolver(waypoints(:, 2), [0; ta], n_seg + 1);

    function poly_coef = LagrangeSolver(waypoints, ta, pts)
        poly_coef = polyfit(ta, waypoints, pts - 1);

%         t = sym('t');
%         for i = 1: pts
%             ti = ta(i);
%             num = t - ta([1: i - 1, i + 1: end]);
%             den = ti - ta([1: i - 1, i + 1: end]);
%             fi = prod(num) / prod(den);
%             L1(i) = waypoints(i) * fi;
%         end
%         poly_coef = sym2poly(expand(sum(L1)));
    end
end
