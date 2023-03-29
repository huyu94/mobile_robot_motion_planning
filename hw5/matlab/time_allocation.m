% 用来实现trapezoidal velocity time allocation


Vel = 10.0;
Acc = 5.0;
t_slope = Vel / Acc;
max_s_triangle = t_slope * Vel / 2;

time = [];
path = 1:1:50;
for i=1:size(path,2)
    s = path(i);
    if s < max_s_triangle
        time(i) = 2.0 * s / Acc;
    elseif s < 2 * max_s_triangle
        time(i) = t_slope + 2 * (s-max_s_triangle) / Acc; 
    else
        time(i) = (s - 2 * max_s_triangle) / Acc + 2 * t_slope;
    end
end

% max_y = 100;
% incre_y = 1;
% y = [];
% for t=1:500
%     if t < 100
%         y(t) = t;
%     elseif t < 400
%         y(t) = 100;
%     else
%         y(t) = 500 - t;
%     end
% end
% plot(y)
% hold on;



plot(time);
hold on;