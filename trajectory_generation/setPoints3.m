function waypoints = setPoints3(f)
    figure(f);
    
    k = 1;
    while (1)
        p = ginput(1);
        if size(p, 1) == 0
            break;
        else
            z = input('Input z = ');
            scatter3(p(1), p(2), z, 'MarkerEdgeColor', 'black', 'LineWidth', 2);
            waypoints(k, :) = [p, z];
            k = k + 1;
        end
    end
end
