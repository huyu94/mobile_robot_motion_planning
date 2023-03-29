function M = getM(n_seg, n_order,t_order, ts)

    coeff = getCoeff(n_order,t_order);
    M = [];
    for k = 1:n_seg
        M_k = zeros(n_order+1);
        %#####################################################
        % STEP 1.1: calculate M_k of the k-th segment 
        
        for i=1:t_order
            for j=1:n_order+1
                % 后面t_order个变量，要带上t
                M_k(i+4,j) = coeff(i,j) * ts(k)^(j-i);
                % 前面t_order个变量，不用带t
                if i==j
                    M_k(i,j) = coeff(i,j) * ts(k)^(j-i);
                end
            end
        end
        M = blkdiag(M, M_k);
    end
end