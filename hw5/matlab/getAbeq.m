function [Aeq beq]= getAbeq(n_seg, n_order,t_order, waypoints, ts, start_cond, end_cond)
    n_all_poly = n_seg*(2 * t_order);
    coeff = getCoeff(n_order,t_order);
    %#####################################################
    % p,v,a,j constraint in start, 
    Aeq_start = zeros(t_order, n_all_poly);
    beq_start = zeros(t_order, 1);
    % STEP 2.1: write expression of Aeq_start and beq_start
    beq_start = start_cond';
    for i=1:t_order
        for j=1:2*t_order
            if i==j
                Aeq_start(i,j) = coeff(i,j); % 因为是起始点，所以t = 0,不用加
            end
        end
    end
    
    %#####################################################
    % p,v,a constraint in end
    Aeq_end = zeros(4, n_all_poly);
    beq_end = zeros(4, 1);
    % STEP 2.2: write expression of Aeq_end and beq_end
    beq_end = end_cond';
    for i=1:t_order
        for j=1:(n_order+1)
            Aeq_end(i,j+(n_seg-1)*(2*t_order)) = coeff(i,j) * ts(n_seg) ^ (j-i);
        end
    end
    
    %#####################################################
    % position constrain in all middle waypoints
    Aeq_wp = zeros(n_seg-1, n_all_poly);
    beq_wp = zeros(n_seg-1, 1);
    % STEP 2.3: write expression of Aeq_wp and beq_wp
%     beq_wp = waypoints(2:n_seg)';
    for k = 0:n_seg-2
        beq_wp(k+1,1) = waypoints(k+2);
        for j = 1:n_order+1
            Aeq_wp(k+1,j+k*(n_order+1)) = coeff(1,j) * ts(k+1)^(j-1);
        end
    end

    
    %#####################################################
    % position continuity constrain between each 2 segments
    Aeq_con_p = zeros(n_seg-1, n_all_poly);
    beq_con_p = zeros(n_seg-1, 1);
    % velocity continuity constrain between each 2 segments
    Aeq_con_v = zeros(n_seg-1, n_all_poly);
    beq_con_v = zeros(n_seg-1, 1);
    % acceleration continuity constrain between each 2 segments
    Aeq_con_a = zeros(n_seg-1, n_all_poly);
    beq_con_a = zeros(n_seg-1, 1);    
    % jerk continuity constrain between each 2 segments
    Aeq_con_j = zeros(n_seg-1, n_all_poly);
    beq_con_j = zeros(n_seg-1, 1);
    Aeq_con = [Aeq_con_p; Aeq_con_v; Aeq_con_a; Aeq_con_j];
    beq_con = [beq_con_p; beq_con_v; beq_con_a; beq_con_j];

    % STEP 2.4: write expression of Aeq_con and beq_con
    for k=0:n_seg-2
        for i=1:t_order
            for j=1:n_order+1
                Aeq_con(k*t_order+i,k*(n_order+1)+j) = coeff(i,j) * ts(k+1)^(j-i);
                if i==j
                    Aeq_con(k*t_order+i,(k+1)*(n_order+1)+j) = -coeff(i,j);
                end
            end
        end
    end


    %#####################################################
    % combine all components to form Aeq and beq   
    Aeq = [Aeq_start; Aeq_end; Aeq_wp; Aeq_con];
    beq = [beq_start; beq_end; beq_wp; beq_con];
end