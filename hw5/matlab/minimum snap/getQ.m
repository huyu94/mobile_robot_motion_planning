function Q = getQ(n_seg, n_order,t_order,ts)
    Q = [];
    for k = 1:n_seg
        % 7阶，所以有8个自由度
        Q_k = zeros(n_order+1,n_order+1);
        %#####################################################
        % STEP 1.1: calculate Q_k of the k-th segment 
        for i=0:n_order
            for j=0:n_order
                if((i<t_order)||(j<t_order))
                    continue;
                else
                    Q_k(i+1,j+1) = factorial(i)/ factorial(i-t_order) * factorial(j) / factorial(j-t_order) /(i+j-n_order) * ts(k) ^ (i+j-n_order);
                end
            end
        end
        Q = blkdiag(Q, Q_k);
    end
end