function corridor = setCorridor()
    path = [50, 50;
           100, 120;
           180, 150;
           250, 80;
           280, 0];
    x_length = 100;
    y_length = 100;
    
    n_order = 7;   % 8 control points
    n_seg = size(path, 1);
    
    corridor = zeros(n_seg,4);
    for i = 1:n_seg
        corridor(i,:) = [path(i,1), path(i,2), x_length/2, y_length/2];
    end
    plot(path(:,1), path(:,2), '*r'); hold on;
    for i = 1:n_seg
        plot_rect([corridor(i,1);corridor(i,2)], corridor(i,3), corridor(i,4));hold on;
    end
    hold on;


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