function Ct = getCt(n_seg, n_order,t_order)
    %#####################################################
    %STEP 2.1: finish the expression of Ct
    ct_rows = t_order * 2 * n_seg; %一段轨迹的初状态和末状态，一共有这么多个变量 2 * t_order 
    % 每两段轨迹共有4 * t_order个变量，然后减去中间重复的t_order个
    % 那么n_seg条轨迹，就共有2*t_order*n_seg个变量，中间重复的就有(n_seg-1)*t_order
    ct_cols = t_order * 2 * n_seg - (n_seg-1)*t_order; 
    Ct = zeros(ct_rows,ct_cols);
    
    start_index_df = 1; %约束变量起始坐标
    end_index_df = t_order * 2 + (n_seg-1); %约束变量结尾坐标
    start_index_dp = t_order * 2 + (n_seg-1); %自由变量起始坐标
    end_index_dp = ct_cols; % 自由变量结尾坐标
    overlap_jump = 2*t_order; % 重复变量数，每个pt要跳多少行
    % 第一段轨迹端点初始的t_order个状态变量
    for i=1:t_order
        Ct(i,i) = 1;
    end
    % 中间每个航路点的位置状态
    for i=0:n_seg-2
        for j=1:t_order
            if j==1
                % 上下同列的两个1
                Ct(t_order+i*overlap_jump+j,        t_order+i+1) = 1; 
                Ct(t_order+i*overlap_jump+j+t_order,t_order+i+1) = 1;
            else
                % 后面的自由变量
                Ct(t_order+i*overlap_jump+j,        start_index_dp+i*(t_order-1)+j-1) = 1;
                Ct(t_order+i*overlap_jump+j+t_order,start_index_dp+i*(t_order-1)+j-1) = 1;
            end
        end
    end

    %最后一段终末点的t_order个状态变量
    for i=1:t_order
        for j=1:t_order
            start_row_last_df = ct_rows - t_order + 0;
            start_col_last_df = end_index_df - t_order + 0;
            if i == j
                Ct(start_row_last_df + i,start_col_last_df + j) = 1;
            end
        end
    end

end