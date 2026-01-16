%% 创建仿真环境 - 真实地形障碍物
disp('正在创建真实环境障碍物...');

% 创建地面
groundLevel = 0;

% 创建真实障碍物（建筑物、树木等）
buildings = [];
trees = [];
terrain = [];
powerLines = [];
dynObstacles = [];  % 动态障碍物

% 添加建筑物
buildingCount = 8;
for i = 1:buildingCount
    x = randi([15, mapSize(1)-15]);
    y = randi([15, mapSize(2)-15]);
    width = randi([5, 15]);
    depth = randi([5, 15]);
    height = randi([10, 30]);
    
    % 确保建筑物不覆盖起点和终点
    while (sqrt((x-start(1))^2 + (y-start(2))^2) < width + 15) || ...
          (sqrt((x-goal(1))^2 + (y-goal(2))^2) < width + 15)
        x = randi([15, mapSize(1)-15]);
        y = randi([15, mapSize(2)-15]);
    end
    
    buildings = [buildings; x, y, width, depth, height];
end

% 添加树木
treeCount = 10;
for i = 1:treeCount
    x = randi([5, mapSize(1)-5]);
    y = randi([5, mapSize(2)-5]);
    radius = randi([2, 4]);  % 树冠半径
    height = randi([8, 15]); % 树高
    
    % 确保树木不覆盖起点和终点
    while (sqrt((x-start(1))^2 + (y-start(2))^2) < radius*2 + 10) || ...
          (sqrt((x-goal(1))^2 + (y-goal(2))^2) < radius*2 + 10)
        x = randi([5, mapSize(1)-5]);
        y = randi([5, mapSize(2)-5]);
    end
    
    trees = [trees; x, y, radius, height];
end

% 添加地形高度变化
terrainPoints = 4;
terrainGrid = 20;
for i = 1:terrainPoints
    x = randi([10, mapSize(1)-10]);
    y = randi([10, mapSize(2)-10]);
    radius = randi([10, 20]);
    height = randi([3, 8]);
    
    terrain = [terrain; x, y, radius, height];
end

% 添加电线杆和电线
poleCount = 3;
for i = 1:poleCount
    x1 = randi([10, mapSize(1)-20]);
    y1 = randi([10, mapSize(2)-20]);
    x2 = x1 + randi([15, 30]);
    y2 = y1 + randi([15, 30]);
    height = randi([15, 25]);
    
    powerLines = [powerLines; x1, y1, x2, y2, height];
end

% 添加动态障碍物 - 3个 (1个可控 + 2个自动移动)
dynObstacleCount = 3; 
colorOptions = [0.8 0.2 0.2; 0.2 0.8 0.2; 0.2 0.2 0.8]; % 红、绿、蓝

for i = 1:dynObstacleCount
    x = randi([20, mapSize(1)-20]);
    y = randi([20, mapSize(2)-20]);
    z = randi([10, 40]);
    
    % 动态障碍物半径
    radius = dynObsRadius;
    
    % 为每个障碍物选择不同颜色
    color = colorOptions(mod(i-1, size(colorOptions, 1))+1, :);
    
    % 确保动态障碍物不覆盖起点和终点
    while (sqrt((x-start(1))^2 + (y-start(2))^2 + (z-start(3))^2) < radius + 15) || ...
          (sqrt((x-goal(1))^2 + (y-goal(2))^2 + (z-goal(3))^2) < radius + 15)
        x = randi([20, mapSize(1)-20]);
        y = randi([20, mapSize(2)-20]);
        z = randi([10, 40]);
    end
    
    dynObstacles = [dynObstacles; x, y, z, radius, color];
    
    % 初始化自动移动障碍物的参数 (所有障碍物都自动移动)
    % 随机初始方向
    theta = rand() * 2 * pi;
    phi = (rand() - 0.5) * pi * 0.5; % 限制垂直角度
    
    dx = cos(theta) * cos(phi);
    dy = sin(theta) * cos(phi);
    dz = sin(phi);
    
    autoObsDirections = [autoObsDirections; [dx, dy, dz]];
    
    % 随机速度 (0.5-1.5)
    autoObsSpeeds = [autoObsSpeeds; 0.5 + rand()];
end