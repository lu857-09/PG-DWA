%% 创建初始环境图
initialEnvFig = figure('Name', '无人机初始环境', 'Position', [50, 350, 800, 600]);

% 设置额外数据
setappdata(initialEnvFig, 'mapSize', mapSize);

% 绘制环境
subplot(1, 1, 1);
hold on;
grid on;
view(3);
title('无人机初始环境 (无路径)');
xlabel('X轴');
ylabel('Y轴');
zlabel('Z轴');
axis([0 mapSize(1) 0 mapSize(2) 0 mapSize(3)]);

% 绘制地面
surf([0 mapSize(1); 0 mapSize(1)], [0 0; mapSize(2) mapSize(2)], zeros(2), ...
     'FaceColor', [0.8 0.8 0.8], 'EdgeColor', 'none', 'FaceAlpha', 0.3);

% 绘制建筑物
for i = 1:size(buildings, 1)
    x = buildings(i, 1);
    y = buildings(i, 2);
    width = buildings(i, 3);
    depth = buildings(i, 4);
    height = buildings(i, 5);
    
    % 绘制建筑物
    plotBuilding([x-width/2, y-depth/2, groundLevel], width, depth, height, [0.7 0.7 0.7], 0.3);
end

% 绘制树木
for i = 1:size(trees, 1)
    x = trees(i, 1);
    y = trees(i, 2);
    radius = trees(i, 3);
    height = trees(i, 4);
    
    % 绘制树干
    trunkRadius = radius/3;
    [cx, cy, cz] = cylinder(trunkRadius, 8);
    cz = cz * height * 0.6;
    surf(cx+x, cy+y, cz, 'FaceColor', [0.6 0.3 0], 'EdgeColor', 'none', 'FaceAlpha', 0.3);
    
    % 绘制树冠
    [sx, sy, sz] = sphere(8);
    treeTop = height * 0.6;
    surf(radius*sx+x, radius*sy+y, radius*sz+treeTop+radius, ...
         'FaceColor', [0.1 0.6 0.1], 'EdgeColor', 'none', 'FaceAlpha', 0.4);
end

% 绘制电线杆和电线
for i = 1:size(powerLines, 1)
    x1 = powerLines(i, 1);
    y1 = powerLines(i, 2);
    x2 = powerLines(i, 3);
    y2 = powerLines(i, 4);
    height = powerLines(i, 5);
    
    % 绘制电线杆1
    [cx1, cy1, cz1] = cylinder(0.5, 6);
    cz1 = cz1 * height;
    surf(cx1+x1, cy1+y1, cz1, 'FaceColor', [0.4 0.4 0.4], 'EdgeColor', 'none', 'FaceAlpha', 0.3);
    
    % 绘制电线杆2
    [cx2, cy2, cz2] = cylinder(0.5, 6);
    cz2 = cz2 * height;
    surf(cx2+x2, cy2+y2, cz2, 'FaceColor', [0.4 0.4 0.4], 'EdgeColor', 'none', 'FaceAlpha', 0.3);
    
    % 绘制电线
    line([x1, x2], [y1, y2], [height, height], 'Color', [0.5, 0.5, 0.5], 'LineWidth', 1.5);
end

% 绘制动态障碍物初始位置
for i = 1:size(globalObstacles.dynamic, 1)
    obsPos = globalObstacles.dynamic(i, 1:3);
    obsRadius = globalObstacles.dynamic(i, 4);
    obsColor = globalObstacles.dynamic(i, 5:7);
    
    % 创建障碍物 - 使用球体
    [sx, sy, sz] = sphere(20);
    surf(obsRadius*sx+obsPos(1), obsRadius*sy+obsPos(2), obsRadius*sz+obsPos(3), ...
         'FaceColor', obsColor, 'EdgeColor', 'none', 'FaceAlpha', 0.5);
end

% 绘制起点和终点
plot3(start(1), start(2), start(3), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
plot3(goal(1), goal(2), goal(3), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');