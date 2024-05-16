clear all;
close all;
clc

X = [];
Y = [];
for i =0:249
    for j = 0:249
        x = 12.45 - 0.1 * i;
        y = 12.45 - 0.1 * j;
        X = [X,x];
        Y = [Y,y];
    end
end

X_space_l = -12.5;
X_space_r = 12.5;
%Y_space_up = 2.5;
Y_space_up = 12.0;
Y_space_down = -2.1;
X_park_l = -1.5;
X_park_r = 1.5;
Park_wigh = -5.5;

minx = -12.5;
maxx = 12.5;
miny = -12.5;
maxy = 12.5;
gres = 0.1;

point_map = [];
map = zeros(1,62500)+1;


%////////////////////////////////////////////////////////////////////
park_wid = 3;
park_len = 5.8;

point_map = plot_line(point_map, -12.5,6, 12,6);

point_map = plot_line(point_map, 12,6,12, -park_len);

point_map = plot_line(point_map, 12, -park_len, 12-park_wid, -park_len);
point_map = plot_line(point_map, 12-park_wid, -park_len, 12-park_wid, 0);

point_map = plot_line(point_map, 12-park_wid, 0, 12-park_wid-8,0);
point_map = plot_line(point_map, 12-park_wid-8, 0, 12-park_wid-8,-park_len);
point_map = plot_line(point_map, 12-park_wid-8,-park_len, 12-2*park_wid-8,-park_len);
point_map = plot_line(point_map, 12-2*park_wid-8,-park_len, 12-2*park_wid-8,0);
point_map = plot_line(point_map, 12-2*park_wid-8,0, -12.5,0);


figure(1)
plot(point_map(1,:),point_map(2,:),'.r');hold on;
axis equal
xlim([-12.5,12.5]);ylim([-12.5,12.5]);
test = [];

%disp(size(point_map));
%disp((point_map));

for index = 1:size(point_map,2)
	idx_x = ceil((maxx - point_map(1,index)) / gres);
	idx_y = ceil((maxy - point_map(2,index)) / gres);
    if idx_x==250
        idx_x = 249;
    end
    if idx_y==250
        idx_y = 249;
    end
	index_xy = (idx_x) * 250 + (idx_y);
    test = [test,index_xy];
    map(1,index_xy) = 0;
end

%disp(size(test));

filename=['map','.txt'];
dlmwrite(filename,map,'delimiter',',','newline','pc');

fusion_pos = [0.000000,0.000000,0.000000];
car_pos = [800,200,0];
park_pos = [920,0,920,-580,1180,-580,1180,0];
fusion_theta = 90;
park_type_ = 1;

dlmwrite('map.txt', fusion_pos, '-append', 'delimiter', ',');
dlmwrite('map.txt', car_pos, '-append', 'delimiter', ',');
dlmwrite('map.txt', park_pos, '-append', 'delimiter', ',');
dlmwrite('map.txt', fusion_theta, '-append', 'delimiter', ',');
dlmwrite('map.txt', park_type_, '-append', 'delimiter', ',');

function map = plot_line(map, x1, y1, x2, y2)
    if x1==x2
        if y1<y2
            for index =y1:0.05:y2
                map = [map,[x1;index]];
            end
        else
            for index =y2:0.05:y1
                map = [map,[x1;index]];
            end
        end
    else
        if x1<x2
            for index =x1:0.05:x2
                map = [map,[index;y1]];
            end
        else
            for index =x2:0.05:x1
                map = [map,[index;y1]];
            end
        end
    end
end

function map = oblique_line(map,x1, y1, x2, y2)
    % 计算两点之间直线经过的所有坐标
    
    % 计算两点之间的距离
    distance = max(abs(x2 - x1), abs(y2 - y1));
    
    % 在两点之间生成等间距的参数值
    t = linspace(0, 1, 10*distance + 1);
    
    % 使用参数化直线方程计算每个点的坐标
    x_coords = round(x1 + t * (x2 - x1));
    y_coords = round(y1 + t * (y2 - y1));
    
    % 组合坐标
    coords = [x_coords; y_coords];
    map = [map,coords];
end