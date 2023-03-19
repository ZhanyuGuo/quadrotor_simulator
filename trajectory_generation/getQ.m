function Q = getQ(n_seg, n_order, ts)
Q = [];
n_poly_perseg = n_order + 1;
for j = 1: n_seg
    Q_j = zeros(n_poly_perseg, n_poly_perseg);
    for i = 4: n_order
        for k = 4: n_order
            Q_j(i + 1, k + 1) = factorial(i) / factorial(i - 4) * ...
                factorial(k) / factorial(k - 4) / (i + k - n_order) * ...
                ts(j) ^ (i + k - n_order);
        end
    end
    Q = blkdiag(Q, Q_j);
end
end
