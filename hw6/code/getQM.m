function [Q, M] = getQM(n_seg, n_order,d_order, ts)
    Q = [];
    M = [];
    poly_num = n_order+1;
    M_k = getM(n_order);
    for k = 1:n_seg
        %#####################################################
        % STEP 2.1 calculate Q_k of the k-th segment 
        Q_k = zeros(poly_num,poly_num);
        for i = 0:n_order
            for j = 0:n_order
                if((i<d_order)||(j<d_order))
                    continue;
                else
                    Q_k(i+1,j+1) = factorial(i)/factorial(i-d_order) * ...
                        factorial(j)/factorial(j-d_order) * ...
                        ts(k)^(i+j-2*d_order) /(i+j-n_order);
                end
            end
        end 
        Q = blkdiag(Q, Q_k);
        M = blkdiag(M, M_k);
    end
end