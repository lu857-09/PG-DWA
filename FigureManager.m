%% 创建飞行实时轨迹图
flightFig = figure('Name', '无人机实时飞行轨迹', 'Position', [50, 100, 900, 600], 'CloseRequestFcn', @closeCallback);

% 设置额外数据
setappdata(flightFig, 'mapSize', mapSize);

% 为主窗口添加键盘控制
set(flightFig, 'KeyPressFcn', @keyPressCallback);

% 显示控制说明 - 增加R键重规划功能和O键停止障碍物功能
uicontrol('Style', 'text', 'String', ...
    {'控制说明:', ...
     '1-3: 选择障碍物', ...
     '方向键: 移动障碍物(水平)', ...
     'PgUp/PgDn: 上下移动障碍物', ...
     'I: 障碍物干扰无人机', ...
     'O: 停止/启动障碍物移动', ...
     'R: 手动触发路径重规划', ...
     'ESC: 取消选择', ...
     '空格键: 暂停/继续', ...
     'S: 保存状态', ...
     'D: 切换调试模式', ...
     'T: 切换路径跟随模式'}, ...
    'Position', [10, 10, 150, 230], 'FontSize', 9, ...
    'HorizontalAlignment', 'left', 'BackgroundColor', [0.9 0.9 0.9]);

% 添加暂停/继续按钮
uicontrol('Style', 'pushbutton', 'String', '暂停/继续', ...
    'Position', [10, 250, 150, 30], ...
    'Callback', @pauseButtonCallback);

% 添加重规划按钮
uicontrol('Style', 'pushbutton', 'String', '重规划路径', ...
    'Position', [10, 290, 150, 30], ...
    'Callback', @replanPathCallback);

% 添加障碍物干扰按钮
uicontrol('Style', 'pushbutton', 'String', '障碍物干扰', ...
    'Position', [10, 330, 150, 30], ...
    'Callback', @obstacleInterferenceCallback);

% 添加停止/启动障碍物按钮
uicontrol('Style', 'pushbutton', 'String', '停止/启动障碍物', ...
    'Position', [10, 370, 150, 30], ...
    'Callback', @toggleObstacleMovement);

% 添加路径跟随模式切换按钮
uicontrol('Style', 'pushbutton', 'String', '切换路径跟随模式', ...
    'Position', [10, 410, 150, 30], ...
    'Callback', @togglePathFollowingMode);
uicontrol('Style', 'pushbutton', 'String', '更新信息素可视化', ...
    'Position', [10, 310, 150, 30], ...
    'Callback', @(src,event)updatePheromoneVisualizationCallback());

% 添加查询点设置按钮
uicontrol('Style', 'pushbutton', 'String', '设置查询点', ...
    'Position', [10, 270, 150, 30], ...
    'Callback', @(src,event)setQueryPointCallback());

% 添加使用当前位置按钮
uicontrol('Style', 'pushbutton', 'String', '查询当前位置', ...
    'Position', [10, 230, 150, 30], ...
    'Callback', @(src,event)queryCurrentPositionCallback());
% 在飞行图中绘制基本环境
ax = subplot(1, 1, 1);
currentAxes = ax;
hold on;
grid on;
view(3);
title('无人机实时飞行轨迹 (数字键1-3选择障碍物, I键干扰飞行, R键重规划, O键停止障碍物)');
xlabel('X轴');
ylabel('Y轴');
zlabel('Z轴');
axis([0 mapSize(1) 0 mapSize(2) 0 mapSize(3)]);

% 绘制地面
surf([0 mapSize(1); 0 mapSize(1)], [0 0; mapSize(2) mapSize(2)], zeros(2), ...
     'FaceColor', [0.8 0.8 0.8], 'EdgeColor', 'none', 'FaceAlpha', 0.3);

% 绘制主要障碍物
for i = 1:size(buildings, 1)
    x = buildings(i, 1);
    y = buildings(i, 2);
    width = buildings(i, 3);
    depth = buildings(i, 4);
    height = buildings(i, 5);
    
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
         'FaceColor', [0.1 0.6 0.1], 'EdgeColor', 'none', 'FaceAlpha', 0.3);
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

% 绘制动态障碍物 - 使用球体
for i = 1:size(globalObstacles.dynamic, 1)
    obsPos = globalObstacles.dynamic(i, 1:3);
    obsRadius = globalObstacles.dynamic(i, 4);
    obsColor = globalObstacles.dynamic(i, 5:7);
    
    % 创建球体障碍物
    [sx, sy, sz] = sphere(20);
    dynObsHandles{i} = surf(obsRadius*sx+obsPos(1), obsRadius*sy+obsPos(2), obsRadius*sz+obsPos(3), ...
                          'FaceColor', obsColor, 'EdgeColor', 'none', 'FaceAlpha', 0.7);
    
    % 添加编号文本标签
    text(obsPos(1), obsPos(2), obsPos(3)+obsRadius+1, num2str(i), ...
        'Color', 'k', 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
end

% 绘制路径参考线 - 蓝色 - 保持可见
planPathHandle = plot3(flightPath(:,1), flightPath(:,2), flightPath(:,3), 'b-', 'LineWidth', 2);

% 绘制起点和终点
plot3(start(1), start(2), start(3), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
plot3(goal(1), goal(2), goal(3), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');

% 添加避障可视化元素
avoidanceVectorHandle = quiver3(0, 0, 0, 0, 0, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'Visible', 'off');
avoidanceTextHandle = text(5, 5, 40, '', 'Color', 'r', 'FontWeight', 'bold', 'FontSize', 10, 'BackgroundColor', [1 1 1 0.7], 'Visible', 'off');

% 添加信息素影响指示器 - 新增
pheromoneInfluenceTextHandle = text(5, 5, 50, '', 'Color', [0 0.5 0], 'FontWeight', 'bold', 'FontSize', 10, 'BackgroundColor', [1 1 1 0.7], 'Visible', 'off');

% 添加重规划状态指示器
replanTextHandle = text(50, 50, 35, '路径重规划中...', 'Color', 'g', 'FontWeight', 'bold', 'FontSize', 14, 'BackgroundColor', [1 1 1 0.7], 'HorizontalAlignment', 'center', 'Visible', 'off');

% 添加路径跟随模式指示器
pathFollowModeTextHandle = text(50, 50, 45, '严格路径跟随模式', 'Color', 'b', 'FontWeight', 'bold', 'FontSize', 12, ...
                               'HorizontalAlignment', 'center', 'Visible', 'on');

% 添加暂停状态文本
pauseTextHandle = text(50, 50, 40, '仿真已暂停', 'Color', 'r', 'FontWeight', 'bold', 'FontSize', 16, ...
                     'HorizontalAlignment', 'center', 'Visible', 'off');

% 创建状态监控面板
% 创建固定位置的状态信息面板 - 使用uipanel确保固定位置
statusPanel = uipanel('Parent', flightFig, 'Title', '飞行状态信息', ...
                     'Position', [0.70, 0.70, 0.28, 0.28], ...
                     'BackgroundColor', [0.95 0.95 0.95], ...
                     'ForegroundColor', [0 0 0.5], ...
                     'HighlightColor', [0.3 0.3 0.7], ...
                     'BorderType', 'none', ...
                     'FontWeight', 'bold');

% 创建分区显示不同类型的信息
% 1. 避障信息区
avoidanceInfoPanel = uipanel('Parent', statusPanel, 'Title', '避障状态', ...
                           'Position', [0.05, 0.60, 0.90, 0.35], ...
                           'BackgroundColor', [0.95 0.95 0.95], ...
                           'BorderType', 'line');
avoidanceTextHandle = uicontrol('Parent', avoidanceInfoPanel, 'Style', 'text', ...
                              'Position', [10, 5, 200, 40], ...
                              'String', '正常飞行中，无障碍物', ...
                              'HorizontalAlignment', 'left', ...
                              'FontWeight', 'normal', ...
                              'BackgroundColor', [0.95 0.95 0.95]);
avoidanceStatusPanelHandle = avoidanceInfoPanel;

% 2. 信息素区
pheromoneInfoPanel = uipanel('Parent', statusPanel, 'Title', '信息素影响', ...
                           'Position', [0.05, 0.35, 0.90, 0.22], ...
                           'BackgroundColor', [0.95 0.95 0.95], ...
                           'BorderType', 'line');
pheromoneInfluenceTextHandle = uicontrol('Parent', pheromoneInfoPanel, 'Style', 'text', ...
                                       'Position', [10, 5, 200, 25], ...
                                       'String', '信息素影响: 正常', ...
                                       'HorizontalAlignment', 'left', ...
                                       'FontWeight', 'normal', ...
                                       'BackgroundColor', [0.95 0.95 0.95]);

% 3. 系统状态区
systemStatusPanel = uipanel('Parent', statusPanel, 'Title', '系统状态', ...
                          'Position', [0.05, 0.05, 0.90, 0.28], ...
                          'BackgroundColor', [0.95 0.95 0.95], ...
                          'BorderType', 'line');
systemStatusTextHandle = uicontrol('Parent', systemStatusPanel, 'Style', 'text', ...
                                 'Position', [10, 5, 200, 35], ...
                                 'String', '系统状态: 初始化中', ...
                                 'HorizontalAlignment', 'left', ...
                                 'FontWeight', 'normal', ...
                                 'BackgroundColor', [0.95 0.95 0.95]);

% 创建状态监控图
statsFig = figure('Name', '无人机状态监控', 'Position', [850, 100, 900, 600]);

% 高度-时间曲线
subplot(2, 3, 1);
hold on;
grid on;
title('高度-时间曲线');
xlabel('时间(s)');
ylabel('高度(m)');
axis([0 60 0 mapSize(3)]);

% 速度-时间曲线
subplot(2, 3, 2);
hold on;
grid on;
title('速度-时间曲线');
xlabel('时间(s)');
ylabel('速度(m/s)');
axis([0 60 0 maxSpeed]);

subplot(2, 3, 3);
hold on;
grid on;
title('能量-时间曲线');
xlabel('时间(s)');
ylabel('能量(%)');
axis([0 60 0 100]);

subplot(2, 3, 4);
hold on;
grid on;
title('垂直速度-时间曲线');
xlabel('时间(s)');
ylabel('垂直速度(m/s)');
axis([0 60 -maxVerticalVelocity maxVerticalVelocity]);

subplot(2, 3, 5);
hold on;
grid on;
title('偏航角速度-时间曲线');
xlabel('时间(s)');
ylabel('角速度(rad/s)');
axis([0 60 -maxAngularVelocity maxAngularVelocity]);

% 飞行数据显示面板
subplot(2, 3, 6);
axis off;
flightDataText = text(0.1, 0.9, '', 'FontSize', 10);

% 创建三维投影图
projectionFig = figure('Name', '无人机飞行轨迹投影', 'Position', [850, 750, 900, 400]);
subplot(1, 3, 1);
hold on;
grid on;
title('XY平面投影 (俯视图)');
xlabel('X轴');
ylabel('Y轴');


% 绘制建筑物俯视图
for i = 1:size(buildings, 1)
    x = buildings(i, 1);
    y = buildings(i, 2);
    width = buildings(i, 3);
    depth = buildings(i, 4);
    
    % 使用patch代替rectangle
    xCoords = [x-width/2, x+width/2, x+width/2, x-width/2];
    yCoords = [y-depth/2, y-depth/2, y+depth/2, y+depth/2];
    patch(xCoords, yCoords, [0.7 0.7 0.7], 'EdgeColor', 'k', 'FaceAlpha', 0.3);
end

% 绘制路径和起始点
plot(start(1), start(2), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
plot(goal(1), goal(2), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
plot(flightPath(:,1), flightPath(:,2), 'b-', 'LineWidth', 1);
axis([0 mapSize(1) 0 mapSize(2)]);
axis manual;

subplot(1, 3, 2);
hold on;
grid on;
title('XZ平面投影 (侧视图)');
xlabel('X轴');
ylabel('Z轴');


% 绘制建筑物侧视图
for i = 1:size(buildings, 1)
    x = buildings(i, 1);
    width = buildings(i, 3);
    height = buildings(i, 5);
    
    % 使用patch代替rectangle
    xCoords = [x-width/2, x+width/2, x+width/2, x-width/2];
    yCoords = [0, 0, height, height];
    patch(xCoords, yCoords, [0.7 0.7 0.7], 'EdgeColor', 'k', 'FaceAlpha', 0.3);
end

% 绘制路径和起始点
plot(start(1), start(3), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
plot(goal(1), goal(3), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
plot(flightPath(:,1), flightPath(:,3), 'b-', 'LineWidth', 1);
axis([0 mapSize(1) 0 mapSize(3)]);
axis manual;

subplot(1, 3, 3);
hold on;
grid on;
title('YZ平面投影 (侧视图)');
xlabel('Y轴');
ylabel('Z轴');
axis([0 mapSize(2) 0 mapSize(3)]);

% 绘制建筑物侧视图
for i = 1:size(buildings, 1)
    y = buildings(i, 2);
    depth = buildings(i, 4);
    height = buildings(i, 5);
    
    % 使用patch代替rectangle
    xCoords = [y-depth/2, y+depth/2, y+depth/2, y-depth/2];
    yCoords = [0, 0, height, height];
    patch(xCoords, yCoords, [0.7 0.7 0.7], 'EdgeColor', 'k', 'FaceAlpha', 0.3);
end

% 绘制路径和起始点
plot(start(2), start(3), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
plot(goal(2), goal(3), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
plot(flightPath(:,2), flightPath(:,3), 'b-', 'LineWidth', 1);
axis([0 mapSize(2) 0 mapSize(3)]);
axis manual;

% 创建信息素可视化窗口 - 修改为2D切片热力图
pheromoneFig = figure('Name', '信息素分布可视化', 'Position', [50, 750, 800, 600]);

% 更新按钮区域的背景面板
uicontrol('Style', 'frame', 'Position', [10, 200, 170, 130], 'BackgroundColor', [0.9 0.9 0.9]);

% 添加信息素更新按钮
uicontrol('Style', 'pushbutton', 'String', '更新信息素可视化', ...
    'Position', [10, 310, 150, 30], ...
    'Callback', @(src,event)visualizePheromoneSlices(pheromoneManager, pheromoneFig));

% 显示信息素参数
uicontrol('Style', 'text', 'String', ...
    {'信息素参数:', ...
     ['蒸发率: ', num2str(pheromoneManager.evaporationRate)], ...
     ['扩散率: ', num2str(pheromoneManager.diffusionRate)], ...
     ['沉积强度: ', num2str(pheromoneManager.pheromoneQ)], ...
     ['A*权重: ', num2str(pgdwaStar.adaptiveWeights.astar)], ... 
     ['DWA权重: ', num2str(pgdwaStar.adaptiveWeights.dwa)]}, ... 
    'Position', [10, 210, 150, 100], 'FontSize', 9, ...
    'HorizontalAlignment', 'left', 'BackgroundColor', [0.9 0.9 0.9]);

% 初始化显示信息素切片
visualizePheromoneSlices(pheromoneManager, pheromoneFig);