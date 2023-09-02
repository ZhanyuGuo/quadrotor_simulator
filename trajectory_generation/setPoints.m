function waypoints = setPoints(f)
% @brief: set 2d points
figure(f);
k = 1;
while (1)
    p = ginput(1);
    if size(p, 1) == 0
        break
    else
        scatter(p(1), p(2), 'MarkerEdgeColor', 'black', 'LineWidth', 2);
        waypoints(k, :) = p;
        k = k + 1;
    end
end
end
