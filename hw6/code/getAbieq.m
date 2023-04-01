function [Aieq, bieq] = getAbieq(n_seg, n_order, corridor_range, ts)
%   只约束到p,v,a 三阶状态量
    d_order = 3;
    n_all_poly = n_seg*(n_order+1);

    Aieq = zeros(2*n_seg*d_order*(n_order+1-(d_order-1)/2),n_all_poly);
    bieq = zeros(2*n_seg*d_order*(n_order+1-(d_order-1)/2),1);


    for k=0:d_order-1
        for j=0:n_seg-1
            start_idx_1 = 2*n_seg*k*(n_order+1-(k-1)/2)+2*j*(1+n_order-k);  %2*j(1+n_order-k)段偏移，2*n_seg*k*(n_order+1-(k-1)/2)是状态偏移
            start_idx_2 = (n_order+1)*j;  %段偏移
            for i=0:n_order-k 
                Aieq(start_idx_1+2*i+1:start_idx_1+2*i+2,start_idx_2+i+1:start_idx_2+i+1+k) = ...
                    [YHtriangle(k+1)*factorial(n_order)/factorial(n_order-k)*ts(j+1)^(1-k);
                    -YHtriangle(k+1)*factorial(n_order)/factorial(n_order-k)*ts(j+1)^(1-k)];
                bieq(start_idx_1+2*i+1:start_idx_1+2*i+2,1) = corridor_range(j+1,2*k+1:2*k+2);
            end
        end
    end
%     disp(Aieq)


    %#####################################################
    % STEP 3.2.1 p constraint
    
%     Aieq_p = [];
%     bieq_p = [];
%     for k=1:n_seg
%         coeff_p(1+(k-1)*(n_order+1):k*(n_order+1)) = coeff_p(1+(k-1)*(n_order+1):k*(n_order+1)) * ts(k) ^ (1);
%         bieq_p = [bieq_p;ones(n_order+1,1)*corridor_range(k,2)];
%     for k=1:n_seg
%         bieq_p = [bieq_p;ones(n_order+1,1)*corridor_range(k,1)*(-1)];
%     end
%     Aieq_p = diag(coeff_p,0);
%     Aieq_p = [Aieq_p;-Aieq_p];
% 
%     %#####################################################
%     % STEP 3.2.2 v constraint   
%     num_control = n_order+1;  %控制点应该是n_order+1
%     num_eq = num_control*n_seg*2;   %需要多少个方程：一个控制点有上下两个界，也就是2*seg*num_control
%     Aieq_v = zeros(num_eq/2,n_all_poly);  % 而每两个方程是一样的，所以只要初始化一半就可以了
%     for k=1:n_seg
%         for n = 1:num_control
%             index_col = k*(n_order+1)-1;
%             inedx_row = n+(k-1)*num_control;
%             Aieq_v(inedx_row,index_col:index_col+1) = n_order * YHtriangle(2) * ts(k)^(0);
%         end
%     end
%     Aieq_v = [Aieq_v;-Aieq_v];
%     bieq_v = ones(num_eq,1) * v_max;
% 
%     %#####################################################
%     % STEP 3.2.3 a constraint   
%     num_control = n_order+1;  %控制点应该是n_order+1
%     num_eq = num_control*n_seg*2;   %需要多少个方程：一个控制点有上下两个界，也就是2*seg*num_control
%     Aieq_a = zeros(num_eq/2,n_all_poly);  % 而每两个方程是一样的，所以只要初始化一半就可以了
%     for k=1:n_seg
%         for n = 1:num_control
%             index_col = k*(n_order+1)-2;
%             inedx_row = n+(k-1)*num_control;
%             Aieq_a(inedx_row,index_col:index_col+2) = n_order * (n_order - 1) * YHtriangle(3) * ts(k)^(-1);
%         end
%     end
%     Aieq_a = [Aieq_a;-Aieq_a];
%     bieq_a = ones(num_eq,1) * a_max;
% 
% 
%     %#####################################################
%     % STEP 3.2.4 j constraint   
%     num_control = n_order+1;  %控制点应该是n_order+1
%     num_eq = num_control*n_seg*2;   %需要多少个方程：一个控制点有上下两个界，也就是2*seg*num_control
%     Aieq_j = zeros(num_eq/2,n_all_poly);  % 而每两个方程是一样的，所以只要初始化一半就可以了
%     for k=1:n_seg
%         for n = 1:num_control
%             index_col = k*(n_order+1)-3;
%             inedx_row = n+(k-1)*num_control;
%             Aieq_j(inedx_row,index_col:index_col+3) = factorial(n_order) / factorial(n_order-3) * YHtriangle(4) * ts(k)^(-2);
%         end
%     end
%     Aieq_j = [Aieq_j;-Aieq_j];
%     bieq_j = ones(num_eq,1) * a_max;    
%     
%     %#####################################################
%     % combine all components to form Aieq and bieq   
%     Aieq = [Aieq_p; Aieq_v; Aieq_a];
%     bieq = [bieq_p; bieq_v; bieq_a];
%     Aieq = Aieq_p;
%     bieq = bieq_p;
end