function d = dist(x, y)
% @brief: calculate the distance between two points
d = sum((x - y).^2).^0.5;
end
