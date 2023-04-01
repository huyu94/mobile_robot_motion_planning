function [Aeq, beq] = getAbeq(n_seg, n_order, ts, start_cond, end_cond)
%   只约束到三阶状态量 p,v,a
    d_order = 3;
    poly_num = n_order+1;
    n_all_poly = n_seg*poly_num;
    %#####################################################
    % STEP 2.1 p,v,a constraint in start 
    Aeq_start = zeros(d_order,n_all_poly);
    for k=0:d_order-1
        Aeq_start(k+1,1:k+1) = ...
            factorial(n_order) / factorial(n_order-k)* ...
            YHtriangle(k+1) * ts(1)^(1-k);
    end
    beq_start = start_cond(1:3)';
%     disp("Aeq_start:")
%     disp(Aeq_start);
    
    %#####################################################
    % STEP 2.2 p,v,a constraint in end
    Aeq_end = zeros(d_order,n_all_poly);
    for i=0:d_order-1
        Aeq_end(i+1,end-i:end) = factorial(n_order)/factorial(n_order-i)*YHtriangle(i+1)*ts(end)^(1-i);
    end
    beq_end = end_cond(1:3)';
%     disp("Aeq_end:")
%     disp(Aeq_end);
    
    %#####################################################
    % STEP 2.3 position continuity constrain between 2 segments
    Aeq_con_p = zeros(n_seg-1,n_all_poly); %中间点有n_seg-1个，
    beq_con_p = zeros(n_seg-1,1);
    for k=1:n_seg-1
        Aeq_con_p(k,k*(n_order+1)) = 1 * ts(k)^(1);
        Aeq_con_p(k,k*(n_order+1)+1) = -1 * ts(k)^(1);
    end
%     disp("Aeq_con_p:")
%     disp(Aeq_con_p);


    %#####################################################
    % STEP 2.4 velocity continuity constrain between 2 segments
    Aeq_con_v = zeros(n_seg-1,n_all_poly);
    beq_con_v = zeros(n_seg-1,1);
    for k=1:n_seg-1
        Aeq_con_v(k,k*(n_order+1)-1:k*(n_order+1)) = factorial(n_order)/factorial(n_order-1) * YHtriangle(2) * ts(k)^(0);
        Aeq_con_v(k,k*(n_order+1)+1:k*(n_order+1)+2) = -factorial(n_order)/factorial(n_order-1) * YHtriangle(2) * ts(k)^(0);
    end
%     disp("Aeq_con_v:")
%     disp(Aeq_con_v);    

    %#####################################################
    % STEP 2.5 acceleration continuity constrain between 2 segments
    Aeq_con_a = zeros(n_seg-1,n_all_poly);
    beq_con_a = zeros(n_seg-1,1);
    for k=1:n_seg-1
        Aeq_con_a(k,k*(n_order+1)-2:k*(n_order+1)) = factorial(n_order)/factorial(n_order-2) * YHtriangle(3) * ts(k)^(-1);
        Aeq_con_a(k,k*(n_order+1)+1:k*(n_order+1)+3) = -factorial(n_order)/factorial(n_order-2) * YHtriangle(3) * ts(k)^(-1);
    end
%     disp("Aeq_con_a:")
%     disp(Aeq_con_a);  

    %#####################################################
    % STEP 2.6 jerk continuity constrain between 2 segments
%     Aeq_con_j = zeros(n_seg-1,n_all_poly);
%     beq_con_j = zeros(n_seg-1,1);
%     for k=1:n_seg-1
%         Aeq_con_j(k,k*(n_order+1)-3:k*(n_order+1)) = factorial(n_order)/factorial(n_order-3) * YHtriangle(4) * ts(k)^(-2);
%         Aeq_con_j(k,k*(n_order+1)+1:k*(n_order+1)+4) = -factorial(n_order)/factorial(n_order-3) * YHtriangle(4) * ts(k)^(-2);
%     end
%     disp("Aeq_con_j:")
%     disp(Aeq_con_j);  
    %#####################################################
    %或者直接整个计算
%     for k=0:d_order-1
%         start_idx_1 = k*(n_seg-1);
%         for j=0:n_seg-2
%             start_idx_2 = (n_order+1)*(j+1);
%             Aeq_con(start_idx_1+j+1,start_idx_2-k:start_idx_2) = ...
%                 factorial(n_order)/factorial(n_order-k)*...
%                 YHtriangle(k+1)*ts(j+1)^(1-k);
%             Aeq_con(start_idx_1+j+1,start_idx_2+1:start_idx_2+1+k) = ...
%                 -factorial(n_order)/factorial(n_order-k)*...
%                 YHtriangle(k+1)*ts(j+2)^(1-k);
%         end
%     end


    %#####################################################
    % combine all components to form Aeq and beq   
    Aeq_con = [Aeq_con_p; Aeq_con_v; Aeq_con_a];
    beq_con = [beq_con_p; beq_con_v; beq_con_a];
    Aeq = [Aeq_start; Aeq_end; Aeq_con];
    beq = [beq_start; beq_end; beq_con];
end