clear all;
close all;
clc;

map=load('map.txt');
Map=reshape(map,250,250);
map=Map';
Map = flip(Map, 2);

X = [];
Y = [];
for i =0:249
    for j = 0:249
        a = 12.45 - 0.1 * i;
        y = 12.45 - 0.1 * j;
        X = [X,a];
        Y = [Y,y];
    end
end
X_car = reshape(X,250,250);
X_car = X_car';
Y_car = reshape(Y,250,250);
Y_car = Y_car';

figure;
for index_x = 1:250
    for index_y = 1:250
        if map(index_x,index_y) == 0
           plot(X_car(index_x,index_y),Y_car(index_x,index_y),'.r');hold on;
        end
    end
end
axis equal
xlim([-12.5,12.5]);ylim([-12.5,12.5]);

dismap=generateDistanceMap(Map);
figure;
imagesc(dismap);
colorbar

% 设置图形的轴标签和标题
xlabel('Column');
ylabel('Row');
title('Binary Matrix Visualization');

function distanceMap = generateDistanceMap(grid)
    [N, ~] = size(grid);
    distanceMap = ones(N, N) * inf;
    q = [];

    % Define the directions: up, down, left, right
    dx = [0, 0, -1, 1, 1, 1, -1, -1];
    dy = [-1, 1, 0, 0, 1, -1, 1, -1];

    % Initialize the queue and distance map
    for i = 1:N
        for j = 1:N
            if grid(i, j) == 0
                distanceMap(i, j) = 0;
                q = [q; [i, j]];
            end
        end
    end

    % Perform BFS
    while ~isempty(q)
        x = q(1, 1);
        y = q(1, 2);
        q(1, :) = [];

        for k = 1:4
            nx = x + dx(k);
            ny = y + dy(k);
            if nx >= 1 && nx <= N && ny >= 1 && ny <= N && distanceMap(nx, ny) == inf
                distanceMap(nx, ny) = distanceMap(x, y) + 1;
                q = [q; [nx, ny]];
            end
        end
    end
end


