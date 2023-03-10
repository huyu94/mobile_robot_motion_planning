function [Aeq, beq] = getAbeq(n_seg, n_order, ts, start_cond, end_cond)
    n_all_poly = n_seg*(n_order+1);
    d_order = 4
    %#####################################################
    % STEP 2.1 p,v,a constraint in start 
    Aeq_start = zeros(d_order,n_all_poly);
    for k=0:d_order -1
        Aeq_start(k+1,1:k+1) = ...
            factorial(n_order) / factorial(n_order-k)* ...
            YHtriangle(k+1) * ts(1)^(1-k);
    end
    beq_start = start_cond';
    
    %#####################################################
    % STEP 2.2 p,v,a constraint in end
    Aeq_end = zeros(d_order,n_all_poly);
    for k=0:d_order-1
        Aeq_end(k+1,1:k+1) = ...
            factorial(n_order) / factorial(n_order-k)* ...
            YHtriangle(k+1) * ts(1)^(1-k);
    end
    beq_end = end_cond';
    
    %#####################################################
    % STEP 2.3 position continuity constrain between 2 segments
    Aeq_con_p = zeros((n_seg-1)*d_order,n_all_poly);
    beq_con_p = zeros((n_seg-1)*d_order,1);
    
    for j=0:n_seg-2
        start_idx = (n_order+1) * (j+1);
        Aeq_con_p(j*(n_seg-1)+1,start_idx:start_idx) = ...
            factorial(n_order) / factorial(n_order-1) * ...
            YHtriangle(1) * ts(j+1)^1;
        Aeq_con_p(j*(n_seg-1)+1,start_idx+1:start_idx+1) = ...
            -factorial(n_order) / factorial(n_order-1) * ...
            YHtriangle(1) * ts(j+1)^1;
    end

    %#####################################################
    % STEP 2.4 velocity continuity constrain between 2 segments
    Aeq_con_v = zeros((n-seg-1)*d_order,n_all_poly);
    beq_con_v = zeros((n-seg-1)*d_order,1);
    for j=0:seg-2
        start_idx = (n_order+1) * (j+1);
        Aeq_con_v(j*(n_seg-1)+1,start_idx-2:start_idx) = ...
            factorial(n_order) / factorial(n_order-1) * ...
            YHtriangle(2) * ts(j+1)^2;
        Aeq_con_v(j*(n_seg-1)+1,start_idx)

    %#####################################################
    % STEP 2.5 acceleration continuity constrain between 2 segments
    Aeq_con_a = [];
    beq_con_a = [];

    %#####################################################
    % combine all components to form Aeq and beq   
    Aeq_con = [Aeq_con_p; Aeq_con_v; Aeq_con_a];
    beq_con = [beq_con_p; beq_con_v; beq_con_a];
    Aeq = [Aeq_start; Aeq_end; Aeq_con];
    beq = [beq_start; beq_end; beq_con];
end