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
Y_space_up = 2.5;
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
for index = X_space_l:0.05:X_space_r
    point_map = [point_map,[index;Y_space_up]];
end

for index = X_space_l:0.05:X_park_l
    point_map = [point_map,[index;Y_space_down]];
end

for index = X_park_l:0.05:X_park_r
    point_map = [point_map,[index;Y_space_down + Park_wigh]];
end

for index = X_park_r:0.05:X_space_r
    point_map = [point_map,[index;Y_space_down]];
end

for index = Y_space_down + Park_wigh:0.05:Y_space_down
    point_map = [point_map,[X_park_l;index]];
end

for index = Y_space_down + Park_wigh:0.05:Y_space_down
    point_map = [point_map,[X_park_r;index]];
end
figure(1)
plot(point_map(1,:),point_map(2,:),'.r');hold on;
axis equal
xlim([-12.5,12.5]);ylim([-12.5,12.5]);
test = [];
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

filename=['map','.txt'];
dlmwrite(filename,map,'delimiter',',','newline','pc');