clc;clear;close all
v_max = 400;
a_max = 400;
color = ['r', 'b', 'm', 'g', 'k', 'c', 'c'];

%% specify the center points of the flight corridor and the region of corridor
path = [50, 50;
       100, 120;
       180, 50;
       250, 80;
       230, 160];
x_length = 100;
y_length = 100;

n_order = 7;   % 8 control points
d_order = 4;
n_seg = size(path, 1);

corridor = zeros(n_seg,4);
for i = 1:n_seg
    corridor(i,:) = [path(i,1), path(i,2), x_length/2, y_length/2];
end

%% specify ts for each segment
ts = zeros(n_seg, 1);
for i = 1:n_seg
    ts(i,1) = 1;
end

corridor_x = [corridor(:,1)-corridor(:,3),corridor(:,1)+corridor(:,3)];
poly_coef_x = MinimumSnapCorridorBezierSolver(1, path(:, 1), corridor_x, ts, n_seg, n_order,d_order);
corridor_y = [corridor(:,2)-corridor(:,4),corridor(:,2)+corridor(:,4)];
poly_coef_y = MinimumSnapCorridorBezierSolver(2, path(:, 2), corridor_y, ts, n_seg, n_order,d_order);

%% display the trajectory and cooridor
plot(path(:,1), path(:,2), '*r'); hold on;
for i = 1:n_seg
    plot_rect([corridor(i,1);corridor(i,2)], corridor(i,3), corridor(i,4));hold on;
end
hold on;
x_pos = [];y_pos = [];
idx = 1;

%% #####################################################
% STEP 4: draw bezier curve
for k = 1:n_seg
    for t = linspace(0,1)
        x_pos(idx) = 0.0;
        y_pos(idx) = 0.0;
        for i = 0:n_order
            basis_p = nchoosek(n_order, i) * t^i * (1-t)^(n_order-i);
            x_pos(idx) = x_pos(idx) + ts(k,1) * poly_coef_x((k-1)*(n_order+1)+i+1) * basis_p;
            y_pos(idx) = y_pos(idx) + ts(k,1) * poly_coef_y((k-1)*(n_order+1)+i+1) * basis_p;
        end
        idx = idx + 1;
    end
end

% scatter(...);
% plot(...);
for k=1:n_seg
    f1 = plot(x_pos((k-1)*100+1:k*100),y_pos((k-1)*100+1:k*100),"DisplayName",'Bezier Curves');
    f1.Color = color(k);
    scatter(ts(k,1) * poly_coef_x((k-1)*(n_order+1)+1:k*(n_order+1)), ...
        ts(k,1) * poly_coef_y((k-1)*(n_order+1)+1:k*(n_order+1)),color(k));
end



function poly_coef = MinimumSnapCorridorBezierSolver(axis, waypoints, corridor, ts, n_seg, n_order,d_order)
    start_cond = [waypoints(1), 0, 0, 0];
    end_cond   = [waypoints(end), 0, 0, 0];   
    v_max = 4000;
    a_max = 4000;
    j_max = 4000;
    
    %% #####################################################
    % STEP 1: compute Q_0 of c'Q_0c
    [Q, M]  = getQM(n_seg, n_order,d_order, ts);
    Q_0 = M'*Q*M;
    Q_0 = nearestSPD(Q_0);
    
    %% #####################################################
    % STEP 2: get Aeq and beq
    [Aeq, beq] = getAbeq(n_seg, n_order, ts, start_cond, end_cond);
    
    %% #####################################################
    % STEP 3: get corridor_range, Aieq and bieq 
    
    % STEP 3.1: get corridor_range of x-axis or y-axis,
    % you can define corridor_range as
    % [p1_max, p1_min, v1_max, v1_min, a1_max,a1_min,j1_max,j1_min;
    %  p2_max, p2_min, v2_max, v2_min, a2_max,a2_min,j2_max,j2_min;
    %                   ...,
    %  pn_max, pn_min, vn_max, vn_min, an_max,an_min,jn_max,jn_min];
    corridor_range = zeros(n_seg,2*(d_order-1));
    for k=1:n_seg
        p_bottom = corridor(k,1);
        p_top = corridor(k,2);
        corridor_range(k,:) = [p_top,-p_bottom,v_max,v_max,a_max,a_max];
    end
    
    % STEP 3.2: get Aieq and bieq
    [Aieq, bieq] = getAbieq(n_seg, n_order, corridor_range, ts);
    
    f = zeros(size(Q_0,1),1);
    poly_coef = quadprog(Q_0,f,Aieq, bieq, Aeq, beq);
end

function plot_rect(center, x_r, y_r)
    p1 = center+[-x_r;-y_r];
    p2 = center+[-x_r;y_r];
    p3 = center+[x_r;y_r];
    p4 = center+[x_r;-y_r];
    plot_line(p1,p2);
    plot_line(p2,p3);
    plot_line(p3,p4);
    plot_line(p4,p1);
end

function plot_line(p1,p2)
    a = [p1(:),p2(:)];    
    plot(a(1,:),a(2,:),'b');
end