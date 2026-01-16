% 无人机三维飞行控制仿真 - 基于增强P-G-DWA*算法
% 实现三维环境中的无人机路径规划与避障控制
% 集成多层信息素机制和传统规划技术，实现自适应路径优化
% 全部代码整合为一个文件
 
% 清空工作区和命令窗口
clear all;
clc;

%% 全局变量定义
global globalObstacles;     % 全局障碍物数据
global dynObsHandles;       % 动态障碍物图形句柄
global selectedObstacle;    % 当前选中的障碍物索引
global mapSize;             % 地图尺寸
global currentAxes;         % 当前活动坐标轴
global droneState;          % 无人机状态
global finalPath;           % 实际飞行路径
global energy;              % 剩余能量
global simulationTime;      % 模拟时间
global isPaused;            % 暂停标志
global avoidanceVectorHandle; % 避障向量显示句柄
global avoidanceTextHandle;   % 避障信息文本句柄
global planning_dynObsHandles; % 规划图中的动态障碍物句柄
global initialEnvFig;       % 初始环境图窗口句柄
global flightFig;           % 飞行图窗口句柄
global planningFig;         % 规划图窗口句柄
global pheromoneFig;        % 信息素图窗口句柄
global lastKeyPressTime;    % 上次按键时间 (用于平滑移动)
global lastMoveDirection;   % 上次移动方向 (用于平滑移动)
global stuckDetection;      % 卡住检测数据
global enableDynamicObstacles; % 是否启用动态障碍物避障
global originalPath;        % 存储原始A*路径供预测使用
global pathSegments;        % 存储路径段信息，用于路径曲率计算
global pgdwaStar;           % 增强路径规划器
global pheromoneManager;    % 信息素管理器
global debugMode;           % 调试模式标志
global lastObstacleCount;   % 上次障碍物避障计数
global isReplanning;        % 是否正在重规划路径
global replanTextHandle;    % 重规划文本句柄
global lastReplanTime;      % 上次重规划时间
global flightPath;          % 当前规划路径
global currentGoalIdx;      % 当前目标点索引
global goal;                % 终点坐标
global currentTargetHandle; % 当前目标点句柄
global isRecoveringPath;    % 路径恢复标志
global recoveryStartTime;   % 路径恢复开始时间
global lastAvoidState;      % 上次避障状态
global pathHandle;          % 实际飞行路径句柄
global planPathHandle;      % 规划路径句柄 - 新增：单独管理规划路径显示
global autoMovingObstacles; % 自动移动的障碍物数据
global autoObsDirections;   % 自动移动障碍物的方向
global autoObsSpeeds;       % 自动移动障碍物的速度
global avoidanceStatus;     % 避障状态信息
global strictPathFollowing; % 严格路径跟随模式
global pathFollowModeTextHandle; % 路径跟随模式文本句柄
global continuousCollisionDetection; % 连续碰撞检测标志
global lastPathUpdateTime;  % 上次路径更新时间
global pathTrackingStatus;  % 路径跟踪状态信息
global pathAdvancementStuckCounter; % 路径推进卡住计数器
global collisionSafetyMargin; % 碰撞检测安全边距
global trajectoryCheckDivisions; % 轨迹检测分段数
global pauseTextHandle;     % 暂停状态文本句柄
global allowPathCollisions; % 允许路径上的碰撞检测（减少误报）
global dynObsRadius;        % 动态障碍物半径
global pheromoneInfluenceTextHandle; % 信息素影响指示器 - 新增
global pheromoneInfluenceStatus;     % 信息素影响状态 - 新增
global avoidanceInfoEnhanced;        % 增强的避障信息 - 新增
global systemStatusTextHandle;    % 系统状态文本句柄 - 新增
global statsFig;            % 状态监控图窗口句柄
global projectionFig;       % 投影图窗口句柄
global maxAltitude;         % 最大高度记录
global avoidanceStatusPanelHandle; % 新增: 避障状态面板句柄
global avoidanceEventCount; % 避障事件次数
global totalAvoidanceTime;  % 避障总时间
global avoidanceStartTime;  % 当前避障开始时间
global isObstacleMoving;    % 障碍物是否在移动
global pheromoneUpdateCounter;
global queryPoint;           % 查询点坐标
queryPoint = []; 
% 初始化全局变量
selectedObstacle = 0;
isPaused = false;
avoidanceVectorHandle = [];
avoidanceTextHandle = [];
dynObsHandles = cell(1, 3); % 修改为3个动态障碍物 (1个可控+2个自动)
planning_dynObsHandles = cell(1, 3); % 修改为3个动态障碍物
lastKeyPressTime = 0;
lastMoveDirection = [0,0,0];
stuckDetection = struct('lastPos', [0,0,0], 'stuckTime', 0, 'stuckCount', 0, 'lastCheckTime', 0);
enableDynamicObstacles = true; % 默认启用动态障碍物避障
pathSegments = struct('startIdx', {}, 'endIdx', {}, 'length', {}, 'curvature', {}, 'direction', {}); % 确保初始化为空结构体数组
debugMode = true; % 启用调试模式
lastObstacleCount = 0;
isReplanning = false;
lastReplanTime = 0;
currentTargetHandle = [];
isRecoveringPath = false;
recoveryStartTime = 0;
lastAvoidState = false;
pathHandle = [];
planPathHandle = []; % 新增：单独管理规划路径的句柄
autoMovingObstacles = []; % 自动移动的障碍物
autoObsDirections = [];   % 自动移动障碍物的方向
autoObsSpeeds = [];       % 自动移动障碍物的速度
avoidanceStatus = struct('isAvoiding', false, 'message', '', 'startTime', 0, 'target', [0,0,0]); % 初始化避障状态信息
strictPathFollowing = true; % 默认启用严格路径跟随模式
continuousCollisionDetection = true; % 启用连续碰撞检测
lastPathUpdateTime = 0;   % 初始化上次路径更新时间
pathTrackingStatus = struct('isOnTrack', true, 'lastOnTrackTime', 0, 'recoveryAttempts', 0); % 新增路径跟踪状态
pathAdvancementStuckCounter = 0; % 初始化路径推进卡住计数器
collisionSafetyMargin = 1.5; % 初始化碰撞安全边距 - 减小边距以减少误报
trajectoryCheckDivisions = 8; % 初始化轨迹检测分段数
pauseTextHandle = [];     % 初始化暂停状态文本句柄
allowPathCollisions = true; % 初始默认允许路径上的碰撞（改进的碰撞检测）
dynObsRadius = 4.0;       % 设置动态障碍物半径
pheromoneInfluenceTextHandle = []; % 初始化信息素影响指示器
pheromoneInfluenceStatus = struct('active', false, 'influence', 0, 'message', ''); % 初始化信息素影响状态
avoidanceInfoEnhanced = struct('active', false, 'message', '', 'detailText', '', 'color', [1 0 0]); % 初始化增强避障信息
maxAltitude = 0;          % 初始化最大高度记录
avoidanceStatusPanelHandle = []; % 初始化避障状态面板句柄
avoidanceEventCount = 0;  % 初始化避障事件次数
totalAvoidanceTime = 0;   % 初始化避障总时间
avoidanceStartTime = 0;   % 初始化当前避障开始时间
isObstacleMoving = true;  % 默认障碍物处于移动状态
pheromoneUpdateCounter = 0;
%% 询问是否加载上次状态
loadPreviousState = false;
if exist('drone_simulation_state.mat', 'file')
    reply = input('是否加载上次的仿真状态? (y/n): ', 's');
    if strcmpi(reply, 'y')
        loadPreviousState = true;
    end
end

%% 询问是否使用上次的环境布局
useLastEnvironment = false;
if exist('drone_simulation_state.mat', 'file') && ~loadPreviousState
    reply = input('是否使用上次的环境布局? (y/n): ', 's');
    if strcmpi(reply, 'y')
        useLastEnvironment = true;
    end
end

%% 增加选项：是否重新开始飞行
restartFlight = false;
if loadPreviousState
    restart = input('是否重新开始飞行? (y/n): ', 's');
    if strcmpi(restart, 'y')
        restartFlight = true;
    end
end

%% 询问是否启用动态障碍物避障
reply = input('是否启用动态障碍物避障? (y/n): ', 's');
if strcmpi(reply, 'n')
    enableDynamicObstacles = false;
    disp('已关闭动态障碍物避障，无人机将严格按照A*路径飞行');
else
    enableDynamicObstacles = true;
    disp('已启用动态障碍物避障');
end

%% 询问是否启用严格路径跟随模式
reply = input('是否启用严格路径跟随模式? (y/n): ', 's');
if strcmpi(reply, 'n')
    strictPathFollowing = false;
    disp('已关闭严格路径跟随模式');
else
    strictPathFollowing = true;
    disp('已启用严格路径跟随模式，无人机将尽可能精确跟随规划路径');
end

%% 参数设置与环境初始化
disp('正在初始化系统...');

% 环境参数
mapSize = [100, 100, 60]; % 三维地图尺寸 [x, y, z]
start = [10, 10, 5];      % 起点
goal = [90, 90, 25];      % 终点
minHeight = 3;            % 最小飞行高度

% 算法参数 - 优化参数以提高飞行稳定性和避障能力
gridSize = 3;             % 栅格尺寸
maxSpeed = 3.5;           % 最大速度 - 降低以提高转弯精度
maxAngularVelocity = 2.2; % 最大角速度 - 减小以提高稳定性
maxVerticalVelocity = 2.0;% 最大垂直速度 - 减小以提高稳定性
dt = 0.2;                 % 时间步长
goalWeight = 0.1;         % 目标权重
obstacleWeight = 5.0;     % 障碍物权重
headingWeight = 1.0;      % 航向权重
pathFollowWeight = 60.0;  % 路径跟踪权重
clearance = 5.0;          % 安全距离
waypointReachedDist = 2.0;% 航点到达距离
goalReachedDist = 2.0;    % 目标点到达距离

% 严格路径跟随参数
strictPathFollowingWeight = 100.0; % 严格路径跟随权重
turnSlowdownFactor = 0.6;  % 转弯减速因子 - 增加减速以提高转弯精度
preciseWaypointReachedDist = 1.2; % 精确航点到达距离

% 能源参数
initialEnergy = 1000;     % 初始能量
baseEnergyRate = 0.8;     % 基础能量消耗率
speedEnergyFactor = 0.4;  % 速度对能耗的影响因子
climbEnergyFactor = 0.8;  % 爬升对能耗的影响因子
turnEnergyFactor = 0.2;   % 转向对能耗的影响因子

% 可视化参数
visualUpdateRate = 3;     % 更新频率 - 降低以提高性能

% 新增控制参数
hoverTimeLimit = 3.0;     % 悬浮等待时间上限(秒)
maxHoverCount = 8;        % 最大悬浮次数
hoverStartTime = 0;       % 悬浮开始时间
pathReplanThreshold = 0.8; % 路径重规划触发阈值
pathRecoveryThreshold = 3.0; % 路径恢复触发阈值
maxRecoveryTime = 8.0;    % 最大路径恢复时间

% 碰撞检测参数 - 修复误报
collisionSafetyMargin = 1.5; % 碰撞检测安全余量 - 减小以减少误报
trajectoryCheckDivisions = 6; % 轨迹检测细分数 - 降低以减少误报

% 路径跟踪参数
pathUpdateThreshold = 1.0; % 路径更新阈值(秒)
maxStuckTime = 5.0;        % 最大卡住时间(秒)
lookAheadDistFactor = 1.2; % 前瞻距离因子 - 减小以提高转弯跟踪精度
minLookAheadPoints = 2;    % 最小前瞻点数
maxPathAdvancementStuck = 3; % 最大路径推进卡住次数

% 信息素系统参数 - 强化信息素影响
pheromoneParams = struct();
pheromoneParams.evaporationRate = 0.2;     % 蒸发率
pheromoneParams.diffusionRate = 0.4;       % 扩散率
pheromoneParams.pheromoneQ = 70.0;          % 沉积强度 - 增强信息素影响
pheromoneParams.initialPheromone = 0.1;    % 初始值
pheromoneParams.maxPheromone = 250.0;       % 最大值 - 增加最大值
pheromoneParams.lengthWeight = 0.5;         % 路径长度权重
pheromoneParams.smoothnessWeight = 0.3;     % 路径平滑度权重
pheromoneParams.energyWeight = 0.2;         % 路径能耗权重
pheromoneParams.isVisualized = true;        % 是否可视化
pheromoneParams.pathLayerWeight = 0.4;      % 路径层权重 - 增强路径层影响
pheromoneParams.avoidanceLayerWeight = 3.6; % 避障层权重 - 增强避障层影响
pheromoneParams.energyLayerWeight = 0.9;    % 能耗层权重 - 增强能耗层影响

% PG-DWA*参数
pgdwaParams = struct();
pgdwaParams.astarPheromoneWeight = 0.5;     % A*信息素权重 - 增强信息素影响
pgdwaParams.dwaPheromoneWeight = 0.6;       % DWA信息素权重 - 增强信息素影响
pgdwaParams.evaporationRate = pheromoneParams.evaporationRate;
pgdwaParams.diffusionRate = pheromoneParams.diffusionRate;
pgdwaParams.pheromoneQ = pheromoneParams.pheromoneQ;
pgdwaParams.initialPheromone = pheromoneParams.initialPheromone;
pgdwaParams.maxPheromone = pheromoneParams.maxPheromone;
pgdwaParams.lengthWeight = pheromoneParams.lengthWeight;
pgdwaParams.smoothnessWeight = pheromoneParams.smoothnessWeight;
pgdwaParams.energyWeight = pheromoneParams.energyWeight;

% 如果要加载上次状态，则加载数据
if loadPreviousState
    [loaded, loadedDroneState, loadedPath, loadedObstacles, loadedEnergy, loadedTime, loadedOriginalPath] = loadSimulationState();
    if loaded
        disp('成功加载上次仿真状态，将在初始化完成后应用');
    else
        loadPreviousState = false;
    end
end

%% 设置随机种子
if loadPreviousState || useLastEnvironment
    rng(2); % 固定随机种子，与上次环境保持一致
    disp('使用上次的环境布局...');
else
    rng(2); % 随机种子，基于当前时间，每次运行都不同
    disp('使用随机生成的新环境布局...');
end

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

% 将所有障碍物合并到一个结构体中
globalObstacles = struct();
globalObstacles.buildings = buildings;
globalObstacles.trees = trees;
globalObstacles.terrain = terrain;
globalObstacles.powerLines = powerLines;
globalObstacles.dynamic = dynObstacles;

%% 创建信息素管理器
pheromoneManager = createPheromoneManager(mapSize, gridSize, pheromoneParams);

% ===== 🔥 新增:强制初始化障碍物信息素 =====
forceInitializeObstaclePheromones();

%% 创建路径规划器
pgdwaStar = createPGDWAStar(globalObstacles, mapSize, gridSize, minHeight, pgdwaParams, pheromoneManager);

%% 路径规划
disp('执行初始路径规划...');

% 使用P-G-DWA*算法规划路径
flightPath = planPath(pgdwaStar, start, goal);
originalPath = flightPath;

% 如果路径规划失败，创建简单路径
if isempty(flightPath)
    disp('无法规划路径，创建简单路径');
    midZ = max(start(3), goal(3)) + 10;
    midPoint = [(start(1)+goal(1))/2, (start(2)+goal(2))/2, midZ];
    flightPath = [start; midPoint; goal];
    originalPath = flightPath;
end

% 计算路径段信息
pathSegments = calculatePathSegmentInfo(flightPath);


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

%% 创建规划图窗口
planningFig = figure('Name', '无人机路径规划', 'Position', [850, 350, 800, 600]);

% 设置额外数据
setappdata(planningFig, 'mapSize', mapSize);

% 绘制环境
subplot(1, 1, 1);
hold on;
grid on;
view(3);
title('无人机路径规划 (固定规划路径)');
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

% 绘制动态障碍物
for i = 1:size(globalObstacles.dynamic, 1)
    obsPos = globalObstacles.dynamic(i, 1:3);
    obsRadius = globalObstacles.dynamic(i, 4);
    obsColor = globalObstacles.dynamic(i, 5:7);
    
    % 创建障碍物 - 使用球体
    [sx, sy, sz] = sphere(20);
    planning_dynObsHandles{i} = surf(obsRadius*sx+obsPos(1), obsRadius*sy+obsPos(2), obsRadius*sz+obsPos(3), ...
                                'FaceColor', obsColor, 'EdgeColor', 'none', 'FaceAlpha', 0.5);
end

% 绘制路径 - 明显的蓝色
plot3(flightPath(:,1), flightPath(:,2), flightPath(:,3), 'b-', 'LineWidth', 2.5);

% 绘制起点和终点
plot3(start(1), start(2), start(3), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
plot3(goal(1), goal(2), goal(3), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');

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

%% 初始化无人机参数
% 无人机外形参数 - 修改：缩小无人机模型大小
droneSize = 1.5;  % 从3.0减小到1.5
[sx, sy, sz] = sphere(10);

% 初始化无人机状态
if loadPreviousState && loaded && ~restartFlight
    droneState = loadedDroneState;
    finalPath = loadedPath;
    energy = loadedEnergy;
    simulationTime = loadedTime;
    disp('已还原无人机状态');
else
    % 初始位置、姿态和速度
    droneState = [start, 0, 0, 0, 0, 0, 0]; % [x, y, z, yaw, pitch, roll, vx, vy, vz]
    
    % 计算初始路径方向并设置无人机朝向和初始速度
    if size(flightPath, 1) >= 2
        initialDir = flightPath(2,:) - flightPath(1,:);
        if norm(initialDir) > 0
            initialDir = initialDir / norm(initialDir);
            initialYaw = atan2(initialDir(2), initialDir(1));
            initialPitch = asin(max(-1, min(1, initialDir(3))));
            
            % 更新无人机的初始朝向
            droneState(4) = initialYaw;  % 设置初始航向
            droneState(5) = initialPitch; % 设置初始俯仰角
            
            % 设置初始速度向量与朝向一致
            initialSpeed = maxSpeed * 0.3; % 设置初始速度为最大速度的30%
            droneState(7) = initialSpeed * cos(initialYaw) * cos(initialPitch); % vx
            droneState(8) = initialSpeed * sin(initialYaw) * cos(initialPitch); % vy
            droneState(9) = initialSpeed * sin(initialPitch); % vz
        end
    end
    
    finalPath = start;
    energy = initialEnergy;
    simulationTime = 0;
    disp('初始化无人机状态');
end

% 绘制初始位置的无人机
droneHandle = surf(droneSize/2*sx + droneState(1), droneSize/2*sy + droneState(2), droneSize/2*sz + droneState(3), ...
              'FaceColor', 'b', 'EdgeColor', 'none', 'FaceAlpha', 0.8);
headingLength = 5;
headingX = [droneState(1), droneState(1) + headingLength * cos(droneState(4)) * cos(droneState(5))];
headingY = [droneState(2), droneState(2) + headingLength * sin(droneState(4)) * cos(droneState(5))];
headingZ = [droneState(3), droneState(3) + headingLength * sin(droneState(5))];
headingLine = plot3(headingX, headingY, headingZ, 'r-', 'LineWidth', 2);

% 绘制路径轨迹 - 使用明显的绿色
pathHandle = [];

% 绘制投影图
xyHandle = [];
xzHandle = [];
yzHandle = [];

% 绘制状态图表
altHandle = [];
velHandle = [];
engHandle = [];
vzHandle = [];
omegaHandle = [];

% 设置当前目标点
currentGoalIdx = 2;
if currentGoalIdx <= size(flightPath, 1)
    currentTargetHandle = plot3(flightPath(currentGoalIdx,1), flightPath(currentGoalIdx,2), flightPath(currentGoalIdx,3), ...
                           'mo', 'MarkerSize', 6, 'MarkerFaceColor', 'm');
else
    currentTargetHandle = [];
end

% 初始化飞行统计
pathLength = 0;
averageSpeed = 0;
totalFlightTime = 0;
avoidanceEventCount = 0;
totalAvoidanceTime = 0;
avgAvoidanceTime = 0;
pathDeviationCount = 0;
visualCounter = 0;
lastPathAdvanceTime = 0;
lastPathDeviationTime = 0;
lastAvoidanceInfo = struct('active', false, 'direction', [0,0,0], 'distance', 0, 'text', '');
isDeviated = false;
maxAltitude = droneState(3);

% 历史数据
altitudeHistory = droneState(3);
velocityHistory = 0;
timeHistory = 0;
yawHistory = droneState(4);
omegaHistory = 0;
vzHistory = 0;
energyHistory = (energy/initialEnergy)*100;

% 设置初始标志
reachedFinalGoal = false;

%% 主仿真循环
disp('开始飞行仿真...');

while energy > 0 && simulationTime < 300 && ~reachedFinalGoal
    % 检查暂停状态
    if isPaused
        pause(0.1);
        continue;
    end
    
   % 使用全局变量代替 persistent
global pheromoneUpdateCounter;
if isempty(pheromoneUpdateCounter)
    pheromoneUpdateCounter = 0;
end
pheromoneUpdateCounter = pheromoneUpdateCounter + 1;

% 每5个循环周期执行一次（约1秒）
if mod(pheromoneUpdateCounter, 5) == 0
    % 1. 蒸发
    evapRate = pheromoneManager.evaporationRate * 0.2;
    
    % 路径层正常蒸发
    pheromoneManager.pathPheromones = max(pheromoneManager.initialPheromone, ...
        pheromoneManager.pathPheromones * (1 - evapRate * 0.8));
    
    % ⭐ 避障层不蒸发（保持障碍物信息）
    % 删除原来的蒸发代码
    
    pheromoneManager.energyPheromones = max(pheromoneManager.initialPheromone, ...
        pheromoneManager.energyPheromones * (1 - evapRate * 0.7));
    
    % 2. ⭐ 新增：实时更新动态障碍物的避障信息素
    if enableDynamicObstacles && isfield(globalObstacles, 'dynamic') && ~isempty(globalObstacles.dynamic)
        for i = 1:size(globalObstacles.dynamic, 1)
            obsPos = globalObstacles.dynamic(i, 1:3);
            obsRadius = globalObstacles.dynamic(i, 4);
            effectiveRadius = obsRadius * 3.0;
            
            centerIdx = worldToGrid(pheromoneManager, obsPos);
            radiusGrid = ceil(effectiveRadius / pheromoneManager.gridSize);
            dims = size(pheromoneManager.pheromoneMatrix);
            
            % 更新这个障碍物周围的避障信息素
            for dx = -radiusGrid:radiusGrid
                for dy = -radiusGrid:radiusGrid
                    for dz = -radiusGrid:radiusGrid
                        idx = centerIdx + [dx, dy, dz];
                        if all(idx > 0) && all(idx <= dims)
                            dist = sqrt(dx^2 + dy^2 + dz^2) * pheromoneManager.gridSize;
                            if dist <= effectiveRadius
                                factor = exp(-dist^2/(2*(effectiveRadius/3.0)^2));
                                
                                % 更新避障层
                                avoidCost = 50.0 * factor;
                                pheromoneManager.avoidancePheromones(idx(1), idx(2), idx(3)) = ...
                                    max(pheromoneManager.avoidancePheromones(idx(1), idx(2), idx(3)), avoidCost);
                                
                                % 同时降低路径层
                                reductionFactor = 1.0 - 0.95 * factor;
                                pheromoneManager.pathPheromones(idx(1), idx(2), idx(3)) = ...
                                    min(pheromoneManager.pathPheromones(idx(1), idx(2), idx(3)), ...
                                        pheromoneManager.initialPheromone * reductionFactor);
                            end
                        end
                    end
                end
            end
        end
    end
    
    % 3. 在当前位置沉积信息素（代码保持不变）
    currentGridIdx = ceil(droneState(1:3) / gridSize);
    dims = size(pheromoneManager.pheromoneMatrix);
    
    if all(currentGridIdx > 0) && all(currentGridIdx <= dims)
        currentSpeed = sqrt(droneState(7)^2 + droneState(8)^2 + droneState(9)^2);
        speedFactor = currentSpeed / maxSpeed;
        energyFactor = energy / initialEnergy;
        baseDepositAmount = pheromoneManager.pheromoneQ * 0.15 * speedFactor * energyFactor;
        
        for i = -1:1
            for j = -1:1
                for k = -1:1
                    idx = currentGridIdx + [i, j, k];
                    if all(idx > 0) && all(idx <= dims)
                        distance = sqrt(i^2 + j^2 + k^2);
                        decayFactor = exp(-distance * 0.5);
                        depositAmount = baseDepositAmount * decayFactor;
                        
                        currentValue = pheromoneManager.pathPheromones(idx(1), idx(2), idx(3));
                        pheromoneManager.pathPheromones(idx(1), idx(2), idx(3)) = ...
                            min(pheromoneManager.maxPheromone, currentValue + depositAmount);
                    end
                end
            end
        end
    end
    
    % 4. 扩散（代码保持不变）
    if pheromoneManager.diffusionRate > 0 && mod(pheromoneUpdateCounter, 25) == 0
        kernel = ones(3,3,3) / 27;
        diffused = convn(pheromoneManager.pathPheromones, kernel, 'same');
        pheromoneManager.pathPheromones = pheromoneManager.pathPheromones * (1 - pheromoneManager.diffusionRate * 0.5) + ...
            diffused * (pheromoneManager.diffusionRate * 0.5);
    end
    
    % 5. ⭐ 关键修改：更新主矩阵 = 路径层 - 避障层
    pheromoneManager.pheromoneMatrix = pheromoneManager.pathPheromones - pheromoneManager.avoidancePheromones;
    
    % 确保值在有效范围内（允许负值）
    pheromoneManager.pheromoneMatrix = max(-50, pheromoneManager.pheromoneMatrix);
    pheromoneManager.pheromoneMatrix = min(pheromoneManager.maxPheromone, pheromoneManager.pheromoneMatrix);
    
    % 6. 调试信息
    if debugMode && mod(pheromoneUpdateCounter, 50) == 0
        fprintf('[信息素] T=%.1fs, Max=%.1f, Min=%.1f, Avg=%.2f, 障碍区=%d\n', ...
            simulationTime, ...
            max(pheromoneManager.pheromoneMatrix(:)), ...
            min(pheromoneManager.pheromoneMatrix(:)), ...
            mean(pheromoneManager.pheromoneMatrix(:)), ...
            sum(pheromoneManager.pheromoneMatrix(:) < 0));
    end
end
    
    % 获取当前位置
    currentPos = droneState(1:3);
    
    % ... 原有代码继续 ...    
    % 检查是否到达终点
    distToGoal = norm(currentPos - goal);
    if distToGoal < goalReachedDist
        disp('到达终点!');
        reachedFinalGoal = true;
        continue;
    end
    
    % 改进的路径跟踪 - 增加严格路径跟随模式支持
    if strictPathFollowing
        % 严格路径跟随模式下，使用更精确的前瞻点选择
        [closestIdx, closestDist, lookAheadIdx] = enhancedPathFollowing(currentPos, flightPath, droneState);
    else
        % 普通模式下使用原来的路径跟随
        [closestIdx, closestDist, lookAheadIdx] = standardPathFollowing(currentPos, flightPath, droneState);
    end
    
    % 确保索引有效
    if isempty(lookAheadIdx) || lookAheadIdx < 1
        lookAheadIdx = min(2, size(flightPath, 1));
    elseif lookAheadIdx > size(flightPath, 1)
        lookAheadIdx = size(flightPath, 1);
    end
    
    if isempty(closestIdx) || closestIdx < 1
        closestIdx = 1;
    elseif closestIdx > size(flightPath, 1)
        closestIdx = size(flightPath, 1);
    end
    
    % 修复路径推进逻辑 - 防止无人机在转弯处停止跟随路径
    if simulationTime - lastPathUpdateTime > pathUpdateThreshold
        % 更新路径跟踪状态
        pathTrackingStatus.isOnTrack = (closestDist < 5.0);
        
        if pathTrackingStatus.isOnTrack
            pathTrackingStatus.lastOnTrackTime = simulationTime;
            pathTrackingStatus.recoveryAttempts = 0;
        elseif simulationTime - pathTrackingStatus.lastOnTrackTime > maxStuckTime
            % 如果离开路径太久，尝试恢复路径跟踪
            pathTrackingStatus.recoveryAttempts = pathTrackingStatus.recoveryAttempts + 1;
            
            if pathTrackingStatus.recoveryAttempts > 3 && ~isReplanning
                % 多次尝试恢复路径失败，触发重规划
                disp('无法回到规划路径，触发重规划...');
                replanPathCallback([], []);
                pathTrackingStatus.recoveryAttempts = 0;
            end
        end
        
        % 检查是否需要更新当前目标点 - 防止无人机在转弯处卡住
        if currentGoalIdx < size(flightPath, 1)
            % 检查当前目标点是否长时间未更新
            if simulationTime - lastPathAdvanceTime > 5.0
                % 强制推进目标点
                currentGoalIdx = min(currentGoalIdx + 1, size(flightPath, 1));
                disp(['强制更新目标点至索引 ', num2str(currentGoalIdx), ' 以防止路径跟踪卡住']);
                lastPathAdvanceTime = simulationTime;
                pathAdvancementStuckCounter = pathAdvancementStuckCounter + 1;
                
                % 如果多次强制推进仍然卡住，考虑重规划
                if pathAdvancementStuckCounter >= maxPathAdvancementStuck && ~isReplanning
                    disp('多次路径推进仍然卡住，触发重规划...');
                    replanPathCallback([], []);
                    pathAdvancementStuckCounter = 0;
                end
            end
        else
            pathAdvancementStuckCounter = 0;
        end
        
        lastPathUpdateTime = simulationTime;
    end
    
    % 获取当前目标点
    if currentGoalIdx <= size(flightPath, 1)
        % 正常情况下使用当前目标索引
        localGoal = flightPath(currentGoalIdx, :);
    else
        % 如果索引超出范围，使用最后一个点
        localGoal = flightPath(end, :);
    end

    % 使用信息素加权DWA*算法计算控制指令
    try
        [v, vz, omega, pitchRate, avoidingObstacle, pathDeviation, avoidanceDirection, avoidanceDistance, avoidanceMessage, pheromoneInfluence] = ...
            EnhancedPheromoneWeightedDWA(droneState, localGoal, flightPath, globalObstacles, maxSpeed, maxAngularVelocity, ...
                                 maxVerticalVelocity, minHeight, pathFollowWeight, closestIdx, lookAheadIdx, ...
                                 pheromoneManager, pgdwaParams.dwaPheromoneWeight);
        
        % 更新信息素影响状态 - 新增
        if pheromoneInfluence > 0.1
            pheromoneInfluenceStatus.active = true;
            pheromoneInfluenceStatus.influence = pheromoneInfluence;
            
            % 根据影响程度设置消息
            if pheromoneInfluence > 0.5
                pheromoneInfluenceStatus.message = sprintf('强信息素引导\n影响度: %.1f', pheromoneInfluence);
            else
                pheromoneInfluenceStatus.message = sprintf('信息素引导中\n影响度: %.1f', pheromoneInfluence);
            end
        else
            pheromoneInfluenceStatus.active = false;
        end
    catch ME
        warning(['DWA错误: ', ME.message]);
        % 提供安全默认值
        v = 1.0; % 安全默认速度
        vz = 0;
        omega = 0;
        pitchRate = 0;
        avoidingObstacle = false;
        pathDeviation = false;
        avoidanceDirection = [0, 0, 1]; % 默认向上
        avoidanceDistance = 10.0;
        avoidanceMessage = '';
        pheromoneInfluence = 0;
        pheromoneInfluenceStatus.active = false;
    end
    
    % 更新避障状态信息 - 修改：确保避障状态正确显示
    if avoidingObstacle
        if ~avoidanceStatus.isAvoiding
            avoidanceStatus.isAvoiding = true;
            avoidanceStatus.startTime = simulationTime;
            avoidanceStartTime = simulationTime; % 记录当前避障开始时间
            avoidanceStatus.message = avoidanceMessage; % 使用从控制器返回的消息
            avoidanceStatus.target = currentPos + avoidanceDirection * 10.0;
            disp(['开始避障: ', avoidanceMessage]);
            
            % 更新避障事件计数
            avoidanceEventCount = avoidanceEventCount + 1;
            
            % 立即更新避障状态显示
            if ishandle(avoidanceTextHandle)
                set(avoidanceTextHandle, 'String', sprintf('正在避障!\n%s\n距离: %.1f米', avoidanceMessage, avoidanceDistance), ...
                                      'ForegroundColor', [0.9, 0, 0], ...
                                      'FontWeight', 'bold');
            end
            
            % 修改避障状态面板背景色
            if ishandle(avoidanceStatusPanelHandle)
                set(avoidanceStatusPanelHandle, 'BackgroundColor', [1.0, 0.9, 0.9]);
            end
        end
    else
        if avoidanceStatus.isAvoiding
            % 计算避障持续时间
            avoidDuration = simulationTime - avoidanceStatus.startTime;
            disp(['结束避障，持续时间: ', num2str(avoidDuration), '秒']);
            
            % 更新避障统计
            totalAvoidanceTime = totalAvoidanceTime + avoidDuration;
            avgAvoidanceTime = totalAvoidanceTime / avoidanceEventCount;
            
            avoidanceStatus.isAvoiding = false;
            
            % 重置避障状态面板背景色
            if ishandle(avoidanceStatusPanelHandle)
                set(avoidanceStatusPanelHandle, 'BackgroundColor', [0.95, 0.95, 0.95]);
            end
        end
    end

    % 更新避障可视化信息 - 增强避障提示
    if avoidingObstacle
        lastAvoidanceInfo.active = true;
        lastAvoidanceInfo.direction = avoidanceDirection;
        lastAvoidanceInfo.distance = avoidanceDistance;
        
        % 创建增强的避障文本信息
        avoidanceInfoEnhanced.active = true;
        
        % 根据避障距离确定颜色和紧急程度
        if avoidanceDistance < 4.0
            avoidanceInfoEnhanced.color = [1 0 0]; % 红色表示紧急
            severity = '紧急避障!';
        else
            avoidanceInfoEnhanced.color = [1 0.5 0]; % 橙色表示警告
            severity = '规避障碍物';
        end
        
        % 确定避障方向描述
        direction = '避障中';
        if abs(avoidanceDirection(3)) > max(abs(avoidanceDirection(1)), abs(avoidanceDirection(2)))
            if avoidanceDirection(3) > 0
                direction = '向上避障';
            else
                direction = '向下避障';
            end
        else
            if avoidanceDirection(1) > 0
                direction = '向右避障';
            else
                direction = '向左避障';
            end
        end
        
        % 设置避障主消息
        if isempty(avoidanceMessage)
            avoidanceInfoEnhanced.message = sprintf('%s\n%s', severity, direction);
        else
            avoidanceInfoEnhanced.message = avoidanceMessage;
        end
        
        % 设置详细信息
        avoidanceInfoEnhanced.detailText = sprintf('距离障碍物: %.1f米\n减速: %.0f%%\n避障优先级: 高', ...
                                               avoidanceDistance, ...
                                               (1 - v/maxSpeed) * 100);
        
        % 在障碍物周围创建负信息素
        if avoidanceDistance < 5.0
            % 获取障碍物位置
            obsPos = currentPos + avoidanceDirection * avoidanceDistance;
            
for radius = 5:-0.5:1.5
    % 直接内联避障信息素沉积代码
    centerIdx = worldToGrid(pheromoneManager, obsPos);
    radiusGrid = ceil(radius / pheromoneManager.gridSize);
    
    for dx = -radiusGrid:radiusGrid
        for dy = -radiusGrid:radiusGrid
            for dz = -radiusGrid:radiusGrid
                idx = centerIdx + [dx, dy, dz];
                if isValidGrid(pheromoneManager, idx)
                    dist = sqrt(dx^2 + dy^2 + dz^2) * pheromoneManager.gridSize;
                    if dist <= radius
                        factor = exp(-dist^2/(2*(radius/2.0)^2));
                        reduction = pheromoneManager.maxPheromone * factor * 3.0;
                        
                        pheromoneManager.avoidancePheromones(idx(1), idx(2), idx(3)) = ...
                            pheromoneManager.avoidancePheromones(idx(1), idx(2), idx(3)) + reduction * 3.0;
                        
                        current = pheromoneManager.pathPheromones(idx(1), idx(2), idx(3));
                        pheromoneManager.pathPheromones(idx(1), idx(2), idx(3)) = ...
                            max(pheromoneManager.initialPheromone * 0.1, current - reduction * 2.5);
                        
                        current = pheromoneManager.pheromoneMatrix(idx(1), idx(2), idx(3));
                        pheromoneManager.pheromoneMatrix(idx(1), idx(2), idx(3)) = ...
                            max(pheromoneManager.initialPheromone * 0.1, current - reduction * 2.5);
                        
                        pheromoneManager.shortTermPheromones(idx(1), idx(2), idx(3)) = ...
                            max(0, pheromoneManager.shortTermPheromones(idx(1), idx(2), idx(3)) - reduction * 2.0);
                    end
                end
            end
        end
    end
    updateIntegratedPheromones(pheromoneManager);
end
        end
        
        % 调试输出
        if debugMode && mod(round(simulationTime/dt), 10) == 0
            disp(['避障! 距离: ', num2str(avoidanceDistance), ' 方向: [', ...
                 num2str(avoidanceDirection(1)), ', ', ...
                 num2str(avoidanceDirection(2)), ', ', ...
                 num2str(avoidanceDirection(3)), ']']);
        end
        
        % 检测避障状态的变化，用于路径恢复
        if ~lastAvoidState && avoidingObstacle
            % 开始避障
            lastAvoidState = true;
        elseif lastAvoidState && ~avoidingObstacle
            % 刚结束避障，开始恢复路径
            isRecoveringPath = true;
            recoveryStartTime = simulationTime;
            disp('避障完成，开始尝试恢复路径...');
        end
        
        % 强制更新路径显示
        if avoidingObstacle && mod(round(simulationTime/dt), 10) == 0
            if ishandle(flightFig)
                figure(flightFig);
                
                % 确保规划路径(蓝色)显示 - 修正：保持规划路径可见
                if ~ishandle(planPathHandle)
                    planPathHandle = plot3(flightPath(:,1), flightPath(:,2), flightPath(:,3), 'b-', 'LineWidth', 2);
                end
                
                % 确保当前目标点显示
                if ~ishandle(currentTargetHandle)
                    if currentGoalIdx <= size(flightPath, 1)
                        currentTargetHandle = plot3(flightPath(currentGoalIdx,1), flightPath(currentGoalIdx,2), flightPath(currentGoalIdx,3), ...
                                        'mo', 'MarkerSize', 6, 'MarkerFaceColor', 'm');
                    end
                end
            end
        end
    else
        lastAvoidanceInfo.active = false;
        avoidanceInfoEnhanced.active = false;
        
        % 检测避障状态的变化
        if lastAvoidState && ~avoidingObstacle
            % 刚结束避障，开始恢复路径
            if ~isRecoveringPath
                isRecoveringPath = true;
                recoveryStartTime = simulationTime;
                disp('避障完成，开始尝试恢复路径...');
            end
        end
        lastAvoidState = false;
    end

    % 处理路径恢复逻辑 - 改进版
    if isRecoveringPath && ~avoidingObstacle
        % 查找最近的原始路径点
        [closestOrigPathDist, closestOrigIdx] = findClosestPointOnPath(currentPos, originalPath, 1);
        
        if closestOrigPathDist < pathRecoveryThreshold
            % 已成功恢复到原路径
            isRecoveringPath = false;
            disp(['成功回到原路径，距离: ', num2str(closestOrigPathDist)]);
            
            % 更新当前目标点
            if closestOrigIdx < size(originalPath, 1)
                currentGoalIdx = closestOrigIdx + 1;
            else
                currentGoalIdx = closestOrigIdx;
            end
            
            % 更新目标点显示
            if ishandle(currentTargetHandle)
                delete(currentTargetHandle);
            end
            
            if ishandle(flightFig)
                figure(flightFig);
                currentTargetHandle = plot3(originalPath(currentGoalIdx,1), originalPath(currentGoalIdx,2), originalPath(currentGoalIdx,3), ...
                                'mo', 'MarkerSize', 6, 'MarkerFaceColor', 'm');
            end
        elseif simulationTime - recoveryStartTime > maxRecoveryTime
            % 恢复超时，执行重规划
            isRecoveringPath = false;
            disp('路径恢复超时，触发重规划...');
            
            % 触发路径重规划
            replanPathCallback([], []);
        else
            % 继续尝试恢复路径
            if mod(round(simulationTime/dt), 20) == 0
                disp(['尝试恢复路径中，当前距离: ', num2str(closestOrigPathDist)]);
            end
        end
    end

    % 记录路径偏离和避障
    if pathDeviation && simulationTime - lastPathDeviationTime > 5.0
        pathDeviationCount = pathDeviationCount + 1;
        lastPathDeviationTime = simulationTime;
        isDeviated = true;
        disp(['路径偏离! 时间: ', num2str(simulationTime)]);
    end

    % 检查是否到达当前航点 - 改进版
    distToCurrentGoal = norm(droneState(1:3) - localGoal);
    
    % 根据路径跟随模式确定航点到达距离
    if strictPathFollowing
        effectiveWaypointReachedDist = preciseWaypointReachedDist;
    else
        effectiveWaypointReachedDist = waypointReachedDist;
    end
    
    if distToCurrentGoal < effectiveWaypointReachedDist
        % 航点到达，更新下一个航点
        if currentGoalIdx < size(flightPath, 1)
            currentGoalIdx = currentGoalIdx + 1;
            
            % 更新上次航点更新时间
            lastPathAdvanceTime = simulationTime;
            
            % 重置卡住计数器
            pathAdvancementStuckCounter = 0;
        end
    end

    % 修复：预先检查运动是否会导致碰撞
    % 计算潜在的新位置
    newYaw = droneState(4) + omega * dt;
    newPitch = droneState(5) + pitchRate * dt;
    newPitch = max(-pi/3, min(pi/3, newPitch));
    
    potentialX = droneState(1) + v * cos(newYaw) * cos(newPitch) * dt;
    potentialY = droneState(2) + v * sin(newYaw) * cos(newPitch) * dt;
    potentialZ = droneState(3) + vz * dt + v * sin(newPitch) * dt;
    
    % 检查是否会导致碰撞
    currentPosition = droneState(1:3);
    potentialPosition = [potentialX, potentialY, potentialZ];
    
    % 使用细分轨迹检测是否会穿过障碍物 - 修复：增加检测精度
    collisionDetected = false;
    
    if continuousCollisionDetection
        % 临时禁用路径上的碰撞检测，避免误报
        oldAllowPathCollisions = allowPathCollisions;
        allowPathCollisions = false;
        
        for i = 1:trajectoryCheckDivisions
            t = i / trajectoryCheckDivisions;
            checkPos = currentPosition * (1-t) + potentialPosition * t;
            
            % 修复：当无人机严格跟随轨迹但却检测到碰撞时
            % 计算到最近规划路径点的距离
            [minPathDist, ~] = findClosestPointOnPath(checkPos, flightPath, closestIdx);
            
            % 如果检测点非常接近规划路径，且启用了路径碰撞检测，则可能是误报
            if minPathDist < 1.0 && strictPathFollowing
                % 记录调试信息
                if debugMode
                    disp(['忽略可能的误报碰撞，点在规划路径上，距离: ', num2str(minPathDist)]);
                end
                continue;
            end
            
            % 检查碰撞
            if checkCollision(checkPos, globalObstacles)
                collisionDetected = true;
                if debugMode
                    disp(['检测到轨迹碰撞，位置: [', num2str(checkPos(1),2), ', ', ...
                         num2str(checkPos(2),2), ', ', num2str(checkPos(3),2), ']']);
                end
                break;
            end
        end
        
        % 恢复路径碰撞检测设置
        allowPathCollisions = oldAllowPathCollisions;
    end
    
    % 如果检测到碰撞，修改控制输入
    if collisionDetected
        % 紧急制动
        v = v * 0.3;  % 减速但不完全停止，以防卡死
        vz = vz * 0.3;
        
        % 创建微小的后退/上升方向
        avoidingObstacle = true;
        avoidanceDirection = [0, 0, 1]; % 默认向上
        avoidanceDistance = 0.5;
        avoidanceMessage = '检测到轨迹碰撞，紧急制动!';
        
        % 更新增强避障信息
        avoidanceInfoEnhanced.active = true;
        avoidanceInfoEnhanced.message = '紧急碰撞预警!';
        avoidanceInfoEnhanced.detailText = '轨迹前方有障碍物\n执行紧急制动\n向上规避';
        avoidanceInfoEnhanced.color = [1 0 0]; % 紧急红色
        
        % 立即更新避障状态显示
        if ishandle(avoidanceTextHandle)
            set(avoidanceTextHandle, 'String', '紧急碰撞预警!\n轨迹前方有障碍物', ...
                                  'ForegroundColor', [0.9, 0, 0], ...
                                  'FontWeight', 'bold');
        end
        
        % 修改避障状态面板背景色
        if ishandle(avoidanceStatusPanelHandle)
            set(avoidanceStatusPanelHandle, 'BackgroundColor', [1.0, 0.85, 0.85]);
        end
        
        % 记录避障事件
        if ~avoidanceStatus.isAvoiding
            avoidanceStatus.isAvoiding = true;
            avoidanceStatus.startTime = simulationTime;
            avoidanceStartTime = simulationTime;
            avoidanceEventCount = avoidanceEventCount + 1;
        end
    end
    
    % 更新自动移动障碍物 - 仅在障碍物移动状态下执行
    if mod(round(simulationTime/dt), 5) == 0 && isObstacleMoving
        updateAutoMovingObstacles();
    end

    % 更新无人机状态
    droneState = updateDroneState3D(droneState, v, vz, omega, pitchRate, dt);
    
    % 更新无人机飞行路径和统计数据
    if size(finalPath, 1) > 1
        % 计算飞行距离增量
        distIncrement = norm(droneState(1:3) - finalPath(end, :));
        pathLength = pathLength + distIncrement;
    end
    finalPath = [finalPath; droneState(1:3)];
    
    % 计算平均速度
    currentSpeed = sqrt(droneState(7)^2 + droneState(8)^2 + droneState(9)^2);
    averageSpeed = (averageSpeed * totalFlightTime + currentSpeed * dt) / (totalFlightTime + dt);
    
    % 更新总飞行时间
    totalFlightTime = totalFlightTime + dt;
    
    % 更新模拟时间
    simulationTime = simulationTime + dt;
    
    % 能量消耗计算
    energyConsumption = calculateEnergyConsumption(v, vz, omega, baseEnergyRate, ...
                                                 speedEnergyFactor, climbEnergyFactor, ...
                                                 turnEnergyFactor, dt);
    energy = max(0, energy - energyConsumption);

    % 更新历史数据，用于绘制统计图表
    altitudeHistory = [altitudeHistory, droneState(3)];
    velocityHistory = [velocityHistory, currentSpeed];
    timeHistory = [timeHistory, simulationTime];
    omegaHistory = [omegaHistory, omega];
    vzHistory = [vzHistory, vz];
    energyHistory = [energyHistory, (energy/initialEnergy)*100];
    
    % 更新最大高度记录
    maxAltitude = max(maxAltitude, droneState(3));
    
    % 卡住检测 - 预防无人机在原地徘徊
    if norm(droneState(1:3) - stuckDetection.lastPos) < 0.5 && simulationTime - stuckDetection.lastCheckTime > 5.0
        stuckDetection.stuckTime = stuckDetection.stuckTime + (simulationTime - stuckDetection.lastCheckTime);
        
        if stuckDetection.stuckTime > 10.0 && ~isReplanning && ~avoidingObstacle
            % 检测到卡住，触发重规划
            disp('检测到无人机卡住，触发重规划...');
            stuckDetection.stuckCount = stuckDetection.stuckCount + 1;
            stuckDetection.stuckTime = 0;
            
            % 触发路径重规划
            replanPathCallback([], []);
        end
    else
        stuckDetection.stuckTime = 0;
    end
    stuckDetection.lastPos = droneState(1:3);
    stuckDetection.lastCheckTime = simulationTime;
    
    % 更新视觉效果 - 降低更新频率以减少闪烁
    visualCounter = visualCounter + 1;
    if visualCounter >= visualUpdateRate || reachedFinalGoal || isPaused
        visualCounter = 0;
        
        % 安全更新视觉效果
        try
            % 更新飞行轨迹图
            if ishandle(flightFig)
                figure(flightFig);
                
                % 安全检查 - 确保finalPath至少有一个点
                if size(finalPath, 1) > 0
                    % 更新无人机位置
                    if ishandle(droneHandle)
                        delete(droneHandle);
                    end
                    
                    % 修改：如果处于避障状态，改变无人机颜色
                    if avoidingObstacle
                        droneColor = [1, 0.3, 0.3]; % 红色表示避障
                    else
                        droneColor = [0, 0, 1]; % 蓝色表示正常飞行
                    end
                    
                    droneHandle = surf(droneSize/2*sx + droneState(1), droneSize/2*sy + droneState(2), droneSize/2*sz + droneState(3), ...
                                    'FaceColor', droneColor, 'EdgeColor', 'none', 'FaceAlpha', 0.8);
                    
                    % 更新飞行方向指示器
                    if ishandle(headingLine)
                        delete(headingLine);
                    end
                    headingX = [droneState(1), droneState(1) + headingLength * cos(droneState(4)) * cos(droneState(5))];
                    headingY = [droneState(2), droneState(2) + headingLength * sin(droneState(4)) * cos(droneState(5))];
                    headingZ = [droneState(3), droneState(3) + headingLength * sin(droneState(5))];
                    headingLine = plot3(headingX, headingY, headingZ, 'r-', 'LineWidth', 2);
                    
                    % 更新当前目标点
                    if ishandle(currentTargetHandle)
                        try
                            delete(currentTargetHandle);
                        catch
                            % 忽略删除失败的错误
                        end
                    end
                    
                    try
                        if currentGoalIdx <= size(flightPath, 1)
                            currentTargetHandle = plot3(flightPath(currentGoalIdx,1), flightPath(currentGoalIdx,2), flightPath(currentGoalIdx,3), ...
                                                    'mo', 'MarkerSize', 6, 'MarkerFaceColor', 'm');
                        else
                            currentTargetHandle = plot3(goal(1), goal(2), goal(3), 'mo', 'MarkerSize', 6, 'MarkerFaceColor', 'm');
                        end
                    catch ME
                        disp(['目标点绘制错误: ', ME.message]);
                        % 不中断执行
                    end
                    
                    % 更新避障矢量显示 - 修改：确保避障状态正确显示
                    if avoidanceInfoEnhanced.active || avoidingObstacle
                        % 显示避障方向 - 添加全面的检查确保有效
                        try
                            % 验证避障向量的有效性
                            avoidDir = lastAvoidanceInfo.direction;
                            if any(isnan(avoidDir)) || any(isinf(avoidDir))
                                % 重置为安全值
                                avoidDir = [0, 0, 0];
                            end
                            
                            % 确保状态有效
                            if any(isnan(droneState)) || any(isinf(droneState))
                                % 使用之前缓存的有效位置
                                pos = finalPath(end,:);
                            else
                                pos = droneState(1:3);
                            end
                            
                            % 删除旧的避障向量
                            if ishandle(avoidanceVectorHandle)
                                delete(avoidanceVectorHandle);
                            end
                            
                            % 使用归一化向量绘制
                            if norm(avoidDir) > 0
                                normalizedDir = avoidDir / norm(avoidDir);
                                avoidanceVectorHandle = quiver3(pos(1), pos(2), pos(3), ...
                                               normalizedDir(1)*5, normalizedDir(2)*5, normalizedDir(3)*5, ...
                                               'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
                            else
                                % 创建一个小的默认向量
                                avoidanceVectorHandle = quiver3(pos(1), pos(2), pos(3), 0, 0, 0.1, ...
                                                          'r', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'Visible', 'off');
                            end
                            
                            % 显示增强版避障信息文本 - 在固定面板中
                            if ishandle(avoidanceTextHandle)
                                % 获取方向描述
                                dirText = getDirectionDescription(avoidDir);
                                
                                % 根据避障距离设置不同颜色和消息
                                if avoidanceInfoEnhanced.distance < 4.0
                                    set(avoidanceTextHandle, 'String', sprintf('紧急避障!\n距离: %.1f米\n方向: %s\n减速: %.0f%%', ...
                                                                   avoidanceInfoEnhanced.distance, ...
                                                                   dirText, ...
                                                                   (1-v/maxSpeed)*100), ...
                                                       'ForegroundColor', [0.9, 0, 0], ...
                                                       'FontWeight', 'bold');
                                else
                                    set(avoidanceTextHandle, 'String', sprintf('规避障碍物\n距离: %.1f米\n方向: %s\n减速: %.0f%%', ...
                                                                   avoidanceInfoEnhanced.distance, ...
                                                                   dirText, ...
                                                                   (1-v/maxSpeed)*100), ...
                                                       'ForegroundColor', [0.8, 0.4, 0], ...
                                                       'FontWeight', 'normal');
                                end
                                
                                % 修改避障状态面板背景色
                                if ishandle(avoidanceStatusPanelHandle)
                                    set(avoidanceStatusPanelHandle, 'BackgroundColor', [1.0, 0.9, 0.9]);
                                end
                            end
                        catch ME
                            % 如果仍然出错，记录并隐藏避障向量
                            disp(['避障向量更新错误: ', ME.message]);
                            if ishandle(avoidanceVectorHandle)
                                set(avoidanceVectorHandle, 'Visible', 'off');
                            end
                            if ishandle(avoidanceTextHandle)
                                set(avoidanceTextHandle, 'Visible', 'on');
                            end
                        end
                    else
                        % 隐藏避障向量和重置避障文本
                        if ishandle(avoidanceVectorHandle)
                            set(avoidanceVectorHandle, 'Visible', 'off');
                        end
                        if ishandle(avoidanceTextHandle)
                            set(avoidanceTextHandle, 'String', '正常飞行中，无障碍物', ...
                                                  'ForegroundColor', [0, 0.5, 0], ...
                                                  'FontWeight', 'normal');
                        end
                        
                        % 重置避障状态面板背景色
                        if ishandle(avoidanceStatusPanelHandle)
                            set(avoidanceStatusPanelHandle, 'BackgroundColor', [0.95, 0.95, 0.95]);
                        end
                    end
                    
                    % 更新信息素影响指示器 - 放在信息面板中
                    if pheromoneInfluenceStatus.active
                        if ishandle(pheromoneInfluenceTextHandle)
                            % 根据影响强度调整颜色
                            if pheromoneInfluenceStatus.influence > 0.5
                                set(pheromoneInfluenceTextHandle, 'String', sprintf('强信息素引导\n影响度: %.1f\n优化策略: 路径平滑', ...
                                                                        pheromoneInfluenceStatus.influence), ...
                                                              'ForegroundColor', [0, 0.7, 0], ...
                                                              'FontWeight', 'bold');
                            else
                                set(pheromoneInfluenceTextHandle, 'String', sprintf('信息素引导中\n影响度: %.1f', ...
                                                                        pheromoneInfluenceStatus.influence), ...
                                                              'ForegroundColor', [0, 0.5, 0], ...
                                                              'FontWeight', 'normal');
                            end
                        end
                    else
                        if ishandle(pheromoneInfluenceTextHandle)
                            set(pheromoneInfluenceTextHandle, 'String', '信息素影响: 正常', ...
                                                          'ForegroundColor', [0, 0, 0], ...
                                                          'FontWeight', 'normal');
                        end
                    end
                    
                    % 更新系统状态文本
                    if ishandle(systemStatusTextHandle)
                        % 计算平均避障时间
                        if avoidanceEventCount > 0
                            avgAvoidTime = totalAvoidanceTime / avoidanceEventCount;
                        else
                            avgAvoidTime = 0;
                        end
                        
                        % 更新状态信息
                        statusText = sprintf('剩余能量: %.1f%%\n飞行时间: %.1f秒\n避障事件: %d次\n总避障时间: %.1f秒\n平均避障时长: %.2f秒', ...
                                          (energy/initialEnergy)*100, ...
                                          simulationTime, ...
                                          avoidanceEventCount, ...
                                          totalAvoidanceTime, ...
                                          avgAvoidTime);
                        
                        set(systemStatusTextHandle, 'String', statusText);
                    end
                    
                    % 更新实际飞行路径显示
                    if isempty(pathHandle) || ~ishandle(pathHandle)
                        pathHandle = plot3(finalPath(:,1), finalPath(:,2), finalPath(:,3), 'g-', 'LineWidth', 2);
                    else
                        set(pathHandle, 'XData', finalPath(:,1), 'YData', finalPath(:,2), 'ZData', finalPath(:,3));
                    end
                    
                    % 更新重规划状态指示器
                    if isReplanning
                        if ishandle(replanTextHandle)
                            set(replanTextHandle, 'Visible', 'on');
                        end
                    else
                        if ishandle(replanTextHandle)
                            set(replanTextHandle, 'Visible', 'off');
                        end
                    end
                    
                    % 更新路径跟随模式指示器
                    if ishandle(pathFollowModeTextHandle)
                        if strictPathFollowing
                            set(pathFollowModeTextHandle, 'String', '严格路径跟随模式', 'Color', 'b');
                        else
                            set(pathFollowModeTextHandle, 'String', '普通路径跟随模式', 'Color', [0.5 0.5 0]);
                        end
                    end
                    
                    % 更新暂停状态指示器
                    if isPaused && ishandle(pauseTextHandle)
                        set(pauseTextHandle, 'Visible', 'on');
                    elseif ishandle(pauseTextHandle)
                        set(pauseTextHandle, 'Visible', 'off');
                    end
                end
                
                % 更新投影图
                if ishandle(projectionFig)
                    figure(projectionFig);
                    
                    % 确保finalPath至少有一个点
                    if size(finalPath, 1) > 0
                        % XY平面 (俯视图)
                        subplot(1, 3, 1);
                        if isempty(xyHandle) || ~ishandle(xyHandle)
                            xyHandle = plot(finalPath(:,1), finalPath(:,2), 'g-', 'LineWidth', 2);
                        else
                            set(xyHandle, 'XData', finalPath(:,1), 'YData', finalPath(:,2));
                        end
                        
                        % XZ平面 (侧视图)
                        subplot(1, 3, 2);
                        if isempty(xzHandle) || ~ishandle(xzHandle)
                            xzHandle = plot(finalPath(:,1), finalPath(:,3), 'g-', 'LineWidth', 2);
                        else
                            set(xzHandle, 'XData', finalPath(:,1), 'YData', finalPath(:,3));
                        end
                        
                        % YZ平面 (侧视图)
                        subplot(1, 3, 3);
                        if isempty(yzHandle) || ~ishandle(yzHandle)
                            yzHandle = plot(finalPath(:,2), finalPath(:,3), 'g-', 'LineWidth', 2);
                        else
                            set(yzHandle, 'XData', finalPath(:,2), 'YData', finalPath(:,3));
                        end
                    end
                end
                
                % 安全更新状态图表
                if ishandle(statsFig)
                    figure(statsFig);
                    
                    % 确保历史数据至少有一个点
                    if isempty(timeHistory) || isempty(altitudeHistory)
                        continue;
                    end
                    
                    % 更新高度-时间曲线
                    subplot(2, 3, 1);
                    if isempty(altHandle) || ~ishandle(altHandle)
                        altHandle = plot(timeHistory, altitudeHistory, 'b-', 'LineWidth', 2);
                    else
                        set(altHandle, 'XData', timeHistory, 'YData', altitudeHistory);
                    end
                    
                    % 更新速度-时间曲线
                    subplot(2, 3, 2);
                    if isempty(velHandle) || ~ishandle(velHandle)
                        velHandle = plot(timeHistory, velocityHistory, 'r-', 'LineWidth', 2);
                    else
                        set(velHandle, 'XData', timeHistory, 'YData', velocityHistory);
                    end
                    
                    % 更新能量-时间曲线
                    subplot(2, 3, 3);
                    if isempty(engHandle) || ~ishandle(engHandle)
                        engHandle = plot(timeHistory, energyHistory, 'g-', 'LineWidth', 2);
                    else
                        set(engHandle, 'XData', timeHistory, 'YData', energyHistory);
                    end
                    
                    % 更新垂直速度-时间曲线
                    subplot(2, 3, 4);
                    if isempty(vzHandle) || ~ishandle(vzHandle)
                        vzHandle = plot(timeHistory, vzHistory, 'm-', 'LineWidth', 2);
                    else
                        set(vzHandle, 'XData', timeHistory, 'YData', vzHistory);
                    end
                    
                    % 更新角速度-时间曲线
                    subplot(2, 3, 5);
                    if isempty(omegaHandle) || ~ishandle(omegaHandle)
                        omegaHandle = plot(timeHistory, omegaHistory, 'c-', 'LineWidth', 2);
                    else
                        set(omegaHandle, 'XData', timeHistory, 'YData', omegaHistory);
                    end
                    
                    % 更新飞行数据文本
                    subplot(2, 3, 6);
                    if ishandle(flightDataText)
                        % 修改：添加避障状态信息
                        stateStr = '正常飞行';
                        if avoidingObstacle
                            stateStr = '正在避障';
                        end
                        
                        % 计算平均避障时间
                        if avoidanceEventCount > 0
                            avgAvoidTime = totalAvoidanceTime / avoidanceEventCount;
                        else
                            avgAvoidTime = 0;
                        end
                        
                        flightDataStr = sprintf('位置: (%.1f, %.1f, %.1f)\n速度: %.2f m/s\n高度: %.1f m\n航向: %.1f°\n能量: %.1f%%\n飞行时间: %.1f s\n避障事件: %d次\n避障总时间: %.1f s\n平均避障时长: %.2f s\n信息素影响: %.2f\n状态: %s', ...
                            droneState(1), droneState(2), droneState(3), ...
                            sqrt(droneState(7)^2 + droneState(8)^2), ...
                            droneState(3), ...
                            rad2deg(droneState(4)), ...
                            (energy/initialEnergy)*100, ...
                            simulationTime, ...
                            avoidanceEventCount, ...
                            totalAvoidanceTime, ...
                            avgAvoidTime, ...
                            pheromoneInfluence, ...
                            stateStr);
                        set(flightDataText, 'String', flightDataStr);
                    end
                    
                    % 添加选中障碍物信息
                    if selectedObstacle > 0
                        obsPos = globalObstacles.dynamic(selectedObstacle, 1:3);
                        title(['已选中障碍物', num2str(selectedObstacle), ' 位置: (', ...
                             num2str(obsPos(1), '%.1f'), ', ', ...
                             num2str(obsPos(2), '%.1f'), ', ', ...
                             num2str(obsPos(3), '%.1f'), ')']);
                    else
                        title('飞行数据');
                    end
                    
                    % 适应性调整坐标轴范围
                    subplot(2, 3, 1);
                    xlim([0, max(20, max(timeHistory))]);
                    
                    subplot(2, 3, 2);
                    xlim([0, max(20, max(timeHistory))]);
                    
                    subplot(2, 3, 3);
                    xlim([0, max(20, max(timeHistory))]);
                    
                    subplot(2, 3, 4);
                    xlim([0, max(20, max(timeHistory))]);
                    
                    subplot(2, 3, 5);
                    xlim([0, max(20, max(timeHistory))]);
                end
                
                % 返回到飞行图
                figure(flightFig);
            end
        catch ME
            % 捕获并处理错误，不中断仿真
            warning(['图形更新错误: ', ME.message]);
            % 打印堆栈以便调试
            disp(getReport(ME, 'extended', 'hyperlinks', 'off'));
        end
    end
    
    % 刷新窗口显示 - 也添加错误处理
    try
        drawnow limitrate;
    catch
        % 忽略drawnow可能的错误
    end
    
    % 暂停一小段时间使帧率稳定
    pause(0.01);
end

% 飞行完成，保存最终状态
if reachedFinalGoal || energy <= 0
    disp('正在保存最终状态...');
    try
        saveSimulationState(droneState, finalPath, globalObstacles, energy, simulationTime, flightPath);
    catch
        disp('保存状态出错，继续执行');
    end
end

% 飞行完成，更新信息素分布
if reachedFinalGoal
    disp('飞行成功完成！正在更新信息素分布...');
    
    % 计算路径评估指标
    pathMetrics = struct();
    pathMetrics.path_length = pathLength;
    pathMetrics.smoothness = 0.8;  % 可以根据实际路径平滑度计算
    pathMetrics.energy_efficiency = energy / initialEnergy; % 添加能耗评估
    
    % 增强成功路径上的信息素
    depositPheromones(pheromoneManager, finalPath, pathMetrics);
    
    % 可视化更新后的信息素分布
    if ishandle(pheromoneFig)
        figure(pheromoneFig);
        visualizePheromoneSlices(pheromoneManager, pheromoneFig);
    end
    
    if ishandle(flightFig)
        figure(flightFig);
        title('飞行成功完成!', 'Color', 'g');
    end
elseif energy <= 0
    disp('能量耗尽，飞行终止!');
    if ishandle(flightFig)
        figure(flightFig);
        title('能量耗尽，飞行终止!', 'Color', 'r');
    end
else
    disp('飞行超时，未能到达目标!');
    if ishandle(flightFig)
        figure(flightFig);
        title('飞行超时，未能到达目标!', 'Color', 'r');
    end
end

% 显示飞行统计数据
disp(['总飞行时间: ', num2str(totalFlightTime, '%.1f'), ' 秒']);
disp(['飞行距离: ', num2str(pathLength, '%.1f'), ' 米']);
disp(['平均速度: ', num2str(averageSpeed, '%.2f'), ' m/s']);
disp(['最大高度: ', num2str(maxAltitude, '%.1f'), ' 米']);
disp(['剩余能量: ', num2str(energy, '%.1f'), ' (', num2str((energy/initialEnergy)*100, '%.1f'), '%)']);
disp(['避障事件次数: ', num2str(avoidanceEventCount)]);
disp(['避障总时间: ', num2str(totalAvoidanceTime, '%.1f'), ' 秒']);
if avoidanceEventCount > 0
    disp(['平均每次避障时长: ', num2str(totalAvoidanceTime / avoidanceEventCount, '%.2f'), ' 秒']);
end
disp(['路径偏离次数: ', num2str(pathDeviationCount)]);
disp(['卡住检测次数: ', num2str(stuckDetection.stuckCount)]);
disp(['信息素引导影响: ', num2str(pathLength/(pathLength+sum(vzHistory)*0.5)*100, '%.1f'), '%']);

%% 增强版PheromoneWeightedDWA算法
function [v, vz, omega, pitchRate, avoidingObstacle, pathDeviation, avoidanceDirection, avoidanceDistance, avoidanceMessage, pheromoneInfluence] = EnhancedPheromoneWeightedDWA(state, goal, flightPath, obstacles, maxSpeed, maxOmega, maxVz, minHeight, pathFollowWeight, closestIdx, lookAheadIdx, pheromoneManager, pheromoneWeight)
    % 确保声明所有使用的全局变量
    global enableDynamicObstacles;
    global originalPath;
    global debugMode;
    global mapSize;
    global lastReplanTime;
    global simulationTime;
    global pathReplanThreshold;
    global isReplanning;
    global replanTextHandle;
    global goal;
    global currentGoalIdx;
    global pathSegments;
    global planningFig;
    global flightFig;
    global currentTargetHandle;
    global isRecoveringPath;
    global strictPathFollowing; % 添加严格路径跟随模式标志
    global strictPathFollowingWeight; % 添加严格路径跟随权重
    global pathAdvancementStuckCounter; % 路径推进卡住计数器
    global pgdwaStar; % 规划器对象以支持重规划
    global planPathHandle; % 新增：规划路径句柄
    global avoidanceTextHandle; % 确保可以直接更新避障文本
    global avoidanceStatusPanelHandle; % 确保可以更新避障面板状态
    
    % 初始化返回值
    avoidingObstacle = false;
    pathDeviation = false;
    avoidanceDirection = [0, 0, 0];
    avoidanceDistance = inf;
    avoidanceMessage = '';
    pheromoneInfluence = 0; % 返回信息素影响度
    
    % 默认控制输入
    v = maxSpeed * 0.5;
    vz = 0;
    omega = 0;
    pitchRate = 0;
    
    % 从状态向量提取当前位置和姿态
    pos = state(1:3);
    yaw = state(4);
    pitch = state(5);
    
    % 安全检查：确保路径和索引有效
    if isempty(flightPath)
        return;
    end
    
    % 确保索引在有效范围内
    if isempty(closestIdx) || ~isnumeric(closestIdx) || ~isscalar(closestIdx) || isnan(closestIdx) || isinf(closestIdx) || closestIdx < 1
        closestIdx = 1;
    elseif closestIdx > size(flightPath, 1)
        closestIdx = size(flightPath, 1);
    end
    
    if isempty(lookAheadIdx) || ~isnumeric(lookAheadIdx) || ~isscalar(lookAheadIdx) || isnan(lookAheadIdx) || isinf(lookAheadIdx) || lookAheadIdx < 1
        lookAheadIdx = min(2, size(flightPath, 1));
    elseif lookAheadIdx > size(flightPath, 1)
        lookAheadIdx = size(flightPath, 1);
    end

    % 当前位置距离路径的最小距离
    [pathDist, ~] = findClosestPointOnPath(pos, flightPath, 1);
    
    % 获取当前路径点和目标路径点
    currentPathPoint = flightPath(closestIdx, :);
    targetPathPoint = flightPath(lookAheadIdx, :);
    
    % 信息素空间分析 - 增强信息素影响
    [pheromoneInfo, adaptivePheromoneWeight] = analyzeLocalPheromoneSpace(pheromoneManager, pos, state, pathDist, pheromoneWeight);
    
    % 返回信息素影响度 - 用于状态显示
    pheromoneInfluence = adaptivePheromoneWeight; 
    
    % 计算到边界的距离
    distToBoundaryX = min(pos(1), mapSize(1) - pos(1));
    distToBoundaryY = min(pos(2), mapSize(2) - pos(2));
    distToBoundaryZ = min(pos(3), mapSize(3) - pos(3));
    
    % 边界避障检查
    boundaryAvoidance = false;
    boundaryAvoidDir = [0, 0, 0];
    minBoundaryDist = min([distToBoundaryX, distToBoundaryY, distToBoundaryZ]);
    
    % 边界避障激活条件
    boundaryMargin = 4.0;
    try
        if distToBoundaryX < boundaryMargin || distToBoundaryY < boundaryMargin || distToBoundaryZ < boundaryMargin
            boundaryAvoidance = true;
            
            % 初始化避障向量为安全值
            boundaryAvoidDir = [0, 0, 0];
            
            % 逐个维度计算边界避障向量
            if distToBoundaryX < boundaryMargin && isfinite(distToBoundaryX) && distToBoundaryX > 0
                dirX = sign(mapSize(1)/2 - pos(1));
                if isfinite(dirX) && dirX ~= 0
                    boundaryAvoidDir(1) = dirX * (boundaryMargin - distToBoundaryX) / boundaryMargin;
                end
            end
            
            if distToBoundaryY < boundaryMargin && isfinite(distToBoundaryY) && distToBoundaryY > 0
                dirY = sign(mapSize(2)/2 - pos(2));
                if isfinite(dirY) && dirY ~= 0
                    boundaryAvoidDir(2) = dirY * (boundaryMargin - distToBoundaryY) / boundaryMargin;
                end
            end
            
            if distToBoundaryZ < boundaryMargin && isfinite(distToBoundaryZ) && distToBoundaryZ > 0
                dirZ = sign(mapSize(3)/2 - pos(3));
                if isfinite(dirZ) && dirZ ~= 0
                    boundaryAvoidDir(3) = dirZ * (boundaryMargin - distToBoundaryZ) / boundaryMargin;
                end
            end
            
            % 归一化
            if norm(boundaryAvoidDir) > 0
                boundaryAvoidDir = boundaryAvoidDir / norm(boundaryAvoidDir);
            end
        end
    catch ME
        disp(['边界避障计算错误: ', ME.message]);
        boundaryAvoidance = false;
        boundaryAvoidDir = [0, 0, 0];
    end
    
    % 避障逻辑
    try
        % 动态障碍物避障逻辑
        if enableDynamicObstacles
            % 扫描环境，预测障碍物位置
            [isBlocking, obstacleInfo] = predictDynamicObstacles(state, obstacles, 15.0);
            
            if isBlocking
                avoidingObstacle = true;
                avoidanceDirection = obstacleInfo.avoidDirection;
                avoidanceDistance = obstacleInfo.distance;
                
                % 避障信息字符串
                avoidanceMessage = sprintf('避开障碍物 (%.1f米)', avoidanceDistance);
                
                % 障碍物避障权重
                avoidWeight = 0.8;
                
                % 路径跟随权重
                pathWeight = 0.2;
                
                % 获取信息素引导的避障方向
                pheromoneAvoidDir = analyzePheromoneGradientForAvoidance(pheromoneManager, pos, avoidanceDirection);
                
                % 混合避障方向
                if norm(pheromoneAvoidDir) > 0.05
                    blendedAvoidDir = avoidanceDirection * (1-pheromoneInfluence) + pheromoneAvoidDir * pheromoneInfluence;
                    if norm(blendedAvoidDir) > 0
                        blendedAvoidDir = blendedAvoidDir / norm(blendedAvoidDir);
                    else
                        blendedAvoidDir = avoidanceDirection;
                    end
                else
                    blendedAvoidDir = avoidanceDirection;
                end
                
                % 计算路径方向
                pathDir = calculatePathDirection(flightPath, closestIdx, state);
                
                % 计算混合导航方向
                blendedDir = calculateBlendedDirection(pathDir, pathWeight, blendedAvoidDir, avoidWeight, pheromoneInfo);
                
                % 将混合方向映射到航向角和俯仰角
                targetYaw = atan2(blendedDir(2), blendedDir(1));
                targetPitch = asin(max(-1, min(1, blendedDir(3))));
                
                % 计算控制指令
                [omega, pitchRate, v, vz] = calculatePheromoneAwareControlOutputs(...
                    state, targetYaw, targetPitch, maxOmega, maxSpeed, maxVz, ...
                    targetPathPoint, pos, avoidanceDistance, pheromoneInfo);
                
                % 如果非常接近障碍物，确保仍有最小速度而不是停止
                if avoidanceDistance < 3.0
                    v = max(1.2, v * 0.3); % 减速但不停止
                end
            else
                % 无障碍物 - 检查是否正在恢复原路径
                if isRecoveringPath
                    % 恢复原路径模式
                    % 获取最近的原始路径点
                    [~, closestOrigIdx] = findClosestPointOnPath(pos, originalPath, 1);
                    
                    % 使用原始路径上的目标点
                    targetOrigIdx = min(closestOrigIdx + 2, size(originalPath, 1));
                    recoveryTarget = originalPath(targetOrigIdx, :);
                    
                    % 计算朝向原始路径的方向
                    recoveryDir = recoveryTarget - pos;
                    if norm(recoveryDir) > 0
                        recoveryDir = recoveryDir / norm(recoveryDir);
                    else
                        % 修复：使用航向和俯仰角计算默认方向
                        recoveryDir = [cos(yaw)*cos(pitch), sin(yaw)*cos(pitch), sin(pitch)];
                    end
                    
                    % 设置目标航向角和俯仰角
                    targetYaw = atan2(recoveryDir(2), recoveryDir(1));
                    targetPitch = asin(max(-1, min(1, recoveryDir(3))));
                    
                    % 计算控制指令 - 增强路径恢复的角速度
                    omega = min(maxOmega, max(-maxOmega, wrapToPi(targetYaw - yaw) * 2.5));
                    pitchRate = min(maxOmega, max(-maxOmega, (targetPitch - pitch) * 2.0));
                    
                    % 设置恢复速度
                    v = maxSpeed * 0.5;
                    
                    % 高度控制
                    heightDiff = recoveryTarget(3) - pos(3);
                    vz = min(maxVz, max(-maxVz, heightDiff * 1.5));
                    
                    if debugMode && mod(round(simulationTime/0.2), 20) == 0
                        disp(['路径恢复模式 - 目标: (', num2str(recoveryTarget(1),2), ...
                              ',', num2str(recoveryTarget(2),2), ',', num2str(recoveryTarget(3),2), ')']);
                    end
                else
                    % 正常路径跟踪 - 添加严格模式
                    % 计算路径方向
                    pathDir = calculatePathDirection(flightPath, closestIdx, state);
                    
                    % 信息素影响导航参数 - 增强信息素影响
                    pheromoneFactor = calculateEnhancedPheromoneFactor(pheromoneInfo);
                    pheromoneFactor = pheromoneFactor * 1.2; % 增强信息素影响
                    pheromoneInfluence = pheromoneFactor - 1.0; 
                    
                    % 根据路径方向设置目标航向角和俯仰角
                    targetYaw = atan2(pathDir(2), pathDir(1));
                    targetPitch = asin(max(-1, min(1, pathDir(3))));
                    
                    % 根据路径跟随模式计算控制指令
                    % 严格路径跟随模式 - 更精确的控制
                    if strictPathFollowing
                        % 使用增强的严格路径跟随计算控制输出
                        [omega, pitchRate, v, vz] = calculateStrictPathFollowing(...
                            state, targetYaw, targetPitch, maxOmega, maxSpeed, maxVz,... 
                            targetPathPoint, pos, flightPath, closestIdx, pheromoneInfo, ...
                            strictPathFollowingWeight);
                    else
                        % 计算角速度 - 更平滑的转向
                        yawError = wrapToPi(targetYaw - yaw);
                        omega = min(maxOmega, max(-maxOmega, yawError * 2.5));
                        
                        % 计算俯仰角速度
                        pitchError = targetPitch - pitch;
                        pitchRate = min(maxOmega, max(-maxOmega, pitchError * 2.0));
                        
                        % 速度控制 - 严格跟随路径时降低速度
                        baseSpeed = calculatePrecisePathSpeed(pos, flightPath, closestIdx, maxSpeed, state);
                        v = baseSpeed * 0.8; 
                        
                        % 垂直速度控制 - 精确跟随路径高度
                        heightDiff = targetPathPoint(3) - pos(3);
                        vz = min(maxVz, max(-maxVz, heightDiff * 1.5)); 
                    end
                    
                    % 添加信息素影响调整
                    if pheromoneInfluence > 0.2
                        % 如果信息素影响显著，应用更精确的控制
                        omega = omega * (1.0 + pheromoneInfluence * 0.2);
                        v = v * (1.0 + pheromoneInfluence * 0.15);
                    end
                end
            end
        end
    catch ME
        warning(['PheromoneWeightedDWA错误: ', ME.message]);
        % 提供安全默认值
        v = maxSpeed * 0.5;
        vz = 0;
        omega = 0;
        pitchRate = 0;
    end
    
    % 添加前向碰撞检测 - 改进紧急避障
    [collisionDetected, collisionAvoidance] = detectFrontCollision(...
        state, obstacles, v, omega, vz, maxSpeed, maxOmega, maxVz, pheromoneInfo);
    
    if collisionDetected
        avoidingObstacle = true;
        avoidanceDirection = collisionAvoidance.direction;
        avoidanceDistance = collisionAvoidance.distance;
        
        v = collisionAvoidance.v * 0.7;
        vz = collisionAvoidance.vz;
        omega = collisionAvoidance.omega;
        pitchRate = collisionAvoidance.pitchRate;
        
        % 设置避障消息
        avoidanceMessage = sprintf('紧急避障!\n距离: %.1f米\n速度降低: %.0f%%', ...
                              collisionAvoidance.distance, ...
                              (1-v/maxSpeed)*100);
        
        % 直接更新避障状态显示
        if ishandle(avoidanceTextHandle)
            set(avoidanceTextHandle, 'String', avoidanceMessage, ...
                                 'ForegroundColor', [0.9, 0, 0], ...
                                 'FontWeight', 'bold');
            
            % 修改避障状态面板背景色
            if ishandle(avoidanceStatusPanelHandle)
                set(avoidanceStatusPanelHandle, 'BackgroundColor', [1.0, 0.85, 0.85]);
            end
        end
        
        % 调试输出
        if debugMode
            disp(['紧急避障! 距离:', num2str(collisionAvoidance.distance), ' 速度:', num2str(v)]);
        end
    end
    
    % 强制加入垂直控制
    if ~avoidingObstacle
        % 确保无人机始终尝试达到路径点的高度
        if ~isempty(targetPathPoint)
            heightDiff = targetPathPoint(3) - pos(3);
            
            % 增强垂直响应
            vz = min(maxVz, max(-maxVz, heightDiff * 1.2));
            
            % 避免飞行太低
            if pos(3) < minHeight + 1
                vz = max(vz, 0.5); % 强制上升
            end
        end
    end
    
    % 如果偏离距离大于阈值，增加路径跟踪分量
    if pathDist > 1.5
        % 计算回归路径的力量
        returnForce = 1.0 - min(1.0, exp(-pathDist/2.0));
        
        % 修改控制输出，增强航向控制以回归路径
        yawError = wrapToPi(targetYaw - yaw);
        omega = omega * (1.0 + returnForce);
        omega = min(maxOmega, max(-maxOmega, omega));
    end
    
    % 设置路径偏离标志
    try
        if pathDist > 2.0
            pathDeviation = true;
        end
    catch ME
        warning(['DWA错误: ', ME.message]);
        % 提供安全默认值
        v = 1.0; % 安全默认速度
        vz = 0;
        omega = 0;
        pitchRate = 0;
        avoidingObstacle = false;
        pathDeviation = false;
        avoidanceDirection = [0, 0, 1]; % 默认向上
        avoidanceDistance = 10.0;
        avoidanceMessage = '';
        pheromoneInfluence = 0;
    end
end

%% 信息素梯度分析函数
function pheromoneAvoidDir = analyzePheromoneGradientForAvoidance(pheromoneManager, pos, obstacleAvoidDir)
    % 分析信息素梯度以辅助避障方向选择
    
    % 获取当前位置的信息素梯度
    [~, gradient] = getPheromoneValue(pheromoneManager, pos);
    
    % 初始化避障方向
    pheromoneAvoidDir = [0, 0, 0];
    
    % 如果梯度幅度太小，则直接返回零向量
    if norm(gradient) < 0.05
        return;
    end
    
    % 计算梯度与障碍物避障方向的夹角
    dotProduct = dot(gradient, obstacleAvoidDir);
    
    % 如果梯度与避障方向大致一致，则增强避障方向
    if dotProduct > 0
        % 梯度方向与避障方向一致，加强该方向
        pheromoneAvoidDir = gradient;
        if norm(pheromoneAvoidDir) > 0
            pheromoneAvoidDir = pheromoneAvoidDir / norm(pheromoneAvoidDir);
        end
    else
        % 梯度方向与避障方向不一致，寻找垂直于障碍物方向且与梯度较一致的方向
        % 首先找一个垂直于避障方向的向量
        if abs(obstacleAvoidDir(3)) < 0.9 % 如果避障方向不是接近垂直向上或向下
            perpVector = cross(obstacleAvoidDir, [0, 0, 1]);
        else
            perpVector = cross(obstacleAvoidDir, [1, 0, 0]);
        end
        
        if norm(perpVector) > 0
            perpVector = perpVector / norm(perpVector);
            
            % 找到另一个垂直方向
            perpVector2 = cross(obstacleAvoidDir, perpVector);
            if norm(perpVector2) > 0
                perpVector2 = perpVector2 / norm(perpVector2);
                
                % 计算梯度在这两个垂直方向上的投影
                proj1 = dot(gradient, perpVector);
                proj2 = dot(gradient, perpVector2);
                
                % 选择投影最大的方向
                if abs(proj1) > abs(proj2)
                    pheromoneAvoidDir = perpVector * sign(proj1);
                else
                    pheromoneAvoidDir = perpVector2 * sign(proj2);
                end
            else
                pheromoneAvoidDir = perpVector;
            end
        else
            % 如果无法计算垂直向量，使用信息素梯度
            pheromoneAvoidDir = gradient;
            if norm(pheromoneAvoidDir) > 0
                pheromoneAvoidDir = pheromoneAvoidDir / norm(pheromoneAvoidDir);
            end
        end
    end
end

%% 严格路径跟随模式
function [omega, pitchRate, v, vz] = calculateStrictPathFollowing(...
    state, targetYaw, targetPitch, maxOmega, maxSpeed, maxVz, ...
    targetPoint, currentPos, flightPath, closestIdx, pheromoneInfo, ...
    strictPathFollowingWeight)
    
    % 从状态提取当前姿态
    yaw = state(4);
    pitch = state(5);
    
    % 计算精确的路径位置差异
    if closestIdx < size(flightPath, 1)
        pathSegmentDir = flightPath(closestIdx+1,:) - flightPath(closestIdx,:);
        if norm(pathSegmentDir) > 0
            pathSegmentDir = pathSegmentDir / norm(pathSegmentDir);
        end
        
        % 计算当前位置到路径的垂直距离向量
        pathPoint = flightPath(closestIdx,:);
        toPath = currentPos - pathPoint;
        projOnPath = dot(toPath, pathSegmentDir) * pathSegmentDir;
        perpToPath = toPath - projOnPath;
        
        % 归一化
        if norm(perpToPath) > 0
            perpDirToPath = perpToPath / norm(perpToPath);
        else
            perpDirToPath = [0, 0, 0];
        end
    else
        perpDirToPath = [0, 0, 0];
        perpToPath = [0, 0, 0];
    end
    
    % 计算角速度 - 更精确的路径跟随
    yawError = wrapToPi(targetYaw - yaw);
    
    % 精确转向控制
    yawRateGain = 3.0; % 降低增益以提高稳定性
    omega = min(maxOmega, max(-maxOmega, yawError * yawRateGain));
    
    % 精确俯仰角控制
    pitchError = targetPitch - pitch;
    pitchRateGain = 2.5; % 降低增益以提高稳定性
    pitchRate = min(maxOmega, max(-maxOmega, pitchError * pitchRateGain));
    
    % 对准路径的力 - 垂直于路径方向的力
    if norm(perpToPath) > 0
        perpForce = -perpDirToPath * min(2.0, norm(perpToPath)) * 1.8; % 降低增强以减小振荡
        
        % 将这个力转换为额外的偏航修正
        perpYaw = atan2(perpForce(2), perpForce(1));
        perpYawError = wrapToPi(perpYaw - yaw);
        
        % 添加到总偏航控制，更强的修正力
        omega = omega + perpYawError * 1.5; % 降低修正力以减小振荡
        omega = min(maxOmega, max(-maxOmega, omega));
    end
    
    % 速度控制 - 在转弯处大幅降低速度
    turnFactor = abs(yawError) / pi; 
    speedReduction = 1.0 - turnFactor * 0.8; % 增加减速以提高转弯精度
    
    % 距离路径越远，速度越低
    distFactor = min(1.0, norm(perpToPath) / 3.0);
    distanceReduction = 1.0 - distFactor * 0.6;
    
    % 基础速度 - 更低的基础速度以提高精度
    baseSpeed = maxSpeed * 0.5; % 降低基础速度以提高精度
    v = baseSpeed * speedReduction * distanceReduction;
    
    % 确保最小速度
    v = max(0.2 * maxSpeed, v);
    
    % 垂直速度控制 - 精确跟随路径高度
    heightDiff = targetPoint(3) - currentPos(3);
    vz = min(maxVz, max(-maxVz, heightDiff * 1.5));
end

%% 路径方向计算
function pathDir = calculatePathDirection(flightPath, closestIdx, state)
    % 计算路径跟随方向
    
    % 确保索引有效
    if closestIdx >= size(flightPath, 1)
        closestIdx = size(flightPath, 1) - 1;
    end
    if closestIdx < 1
        closestIdx = 1;
    end
    
    % 如果是最后一个点，使用当前航向
    if closestIdx >= size(flightPath, 1)
        yaw = state(4);
        pitch = state(5);
        pathDir = [cos(yaw)*cos(pitch), sin(yaw)*cos(pitch), sin(pitch)];
        return;
    end
    
    % 计算路径段方向
    pathDir = flightPath(closestIdx+1,:) - flightPath(closestIdx,:);
    
    % 归一化
    if norm(pathDir) > 0
        pathDir = pathDir / norm(pathDir);
    else
        % 如果路径段太短，使用当前航向
        yaw = state(4);
        pitch = state(5);
        pathDir = [cos(yaw)*cos(pitch), sin(yaw)*cos(pitch), sin(pitch)];
    end
end

function complexity = calculateEnvironmentComplexity(planner, start, goal)
    % 计算环境复杂度 - 用于自适应参数调整

    % 1. 考虑直线路径上的障碍物数量
    directLine = [linspace(start(1), goal(1), 20);
                  linspace(start(2), goal(2), 20);
                  linspace(start(3), goal(3), 20)]';

    obstacleCount = 0;
    for i = 1:size(directLine, 1)
        if checkCollision(directLine(i,:), planner.obstacles)
            obstacleCount = obstacleCount + 1;
        end
    end

    % 2. 考虑动态障碍物的数量和分布
    dynamicObstacleCount = 0;
    if isfield(planner.obstacles, 'dynamic')
        dynamicObstacleCount = size(planner.obstacles.dynamic, 1);
    end

    % 3. 考虑总体高度变化
    heightChange = abs(goal(3) - start(3)) / planner.mapSize(3);

    % 4. 考虑总距离
    distance = norm(goal - start) / norm(planner.mapSize);

    % 综合计算复杂度指标 (0-1范围)
    complexity = (0.4 * min(1, obstacleCount/10) + ...
                  0.3 * min(1, dynamicObstacleCount/5) + ...
                  0.1 * heightChange + ...
                  0.2 * distance);
end

%% 增强版本的路径规划
function path = planPath(planner, start, goal)
    % 执行完整的规划过程
    planner.iteration = planner.iteration + 1;
    
    % 根据环境复杂度调整信息素权重
    environmentComplexity = calculateEnvironmentComplexity(planner, start, goal);
    planner.adaptiveWeights.astar = min(0.8, planner.astarPheromoneWeight * (1 + environmentComplexity * 0.5));
    planner.adaptiveWeights.dwa = min(0.6, planner.dwaPheromoneWeight * (1 + environmentComplexity * 0.3));
    
    fprintf('环境复杂度: %.2f, A*信息素权重: %.2f, DWA信息素权重: %.2f\n', ...
            environmentComplexity, planner.adaptiveWeights.astar, planner.adaptiveWeights.dwa);
    
    % 生成全局路径 - 使用信息素加权A*
    fprintf('执行信息素加权A*全局路径规划...\n');
    globalPath = pheromoneWeightedAStar(planner, start, goal);
    
    if isempty(globalPath)
        disp('A*无法找到路径，创建简单路径');
        midZ = max(start(3), goal(3)) + 15;
        midPoint = [(start(1)+goal(1))/2, (start(2)+goal(2))/2, midZ];
        globalPath = [start; midPoint; goal];
    end
    
    % 提取关键节点
    fprintf('提取路径关键节点...\n');
    keyPoints = extractKeyPoints(planner, globalPath);
    
    % 计算全局路径指标
    globalMetrics = calculatePathMetrics(planner, globalPath);
    
    % 沉积全局路径上的信息素
    depositPheromones(planner.pheromoneManager, globalPath, globalMetrics);
    
    % 更新全局路径信息素
    for i = 1:size(globalPath, 1)-1
        segment = [globalPath(i,:); globalPath(i+1,:)];
        segmentMetrics = calculatePathMetrics(planner, segment);
        depositPheromones(planner.pheromoneManager, segment, segmentMetrics);
    end
    
    % 障碍物周围沉积负信息素
    if isfield(planner.obstacles, 'dynamic')
        for i = 1:size(planner.obstacles.dynamic, 1)
            obsPos = planner.obstacles.dynamic(i, 1:3);
            obsRadius = planner.obstacles.dynamic(i, 4);
            depositNegativePheromones(planner.pheromoneManager, obsPos, obsRadius * 2.5);
        end
    end

    % 保存历史数据
    planner.pathHistory{end+1} = globalPath;
    planner.pathMetrics = [planner.pathMetrics; globalMetrics];

    % 平滑路径
    smoothedPath = smoothPath(globalPath, planner.obstacles);
    fprintf('路径平滑后，节点数从%d减少到%d\n', size(globalPath,1), size(smoothedPath,1));

    % 沿平滑路径添加中间点以确保最大间距
    densePath = safeAddIntermediatePoints(smoothedPath, planner.obstacles, 4.0);

    % 蒸发信息素
    evaporateAndDiffusePheromones(planner.pheromoneManager);

    % 返回路径
    path = densePath;
end

%% 信息素管理器创建函数
function manager = createPheromoneManager(mapSize, gridSize, params)
    % 创建信息素管理器
    manager = struct();
    
    % 基本参数
    manager.mapSize = mapSize;
    manager.gridSize = gridSize;
    manager.evaporationRate = params.evaporationRate;
    manager.diffusionRate = params.diffusionRate;
    manager.pheromoneQ = params.pheromoneQ;
    manager.initialPheromone = params.initialPheromone;
    manager.maxPheromone = params.maxPheromone;
    manager.isVisualized = params.isVisualized;
    manager.lengthWeight = params.lengthWeight;
    manager.smoothnessWeight = params.smoothnessWeight;
    manager.energyWeight = params.energyWeight;
    
    % 添加分层权重
    manager.pathLayerWeight = params.pathLayerWeight;
    manager.avoidanceLayerWeight = params.avoidanceLayerWeight;
    manager.energyLayerWeight = params.energyLayerWeight;
    
    % 计算网格尺寸
    dims = ceil(mapSize / gridSize);
    
    % 主信息素矩阵和多层信息素
    manager.pheromoneMatrix = ones(dims(1), dims(2), dims(3)) * manager.initialPheromone;
    manager.pathPheromones = ones(dims(1), dims(2), dims(3)) * manager.initialPheromone;
    manager.avoidancePheromones = zeros(dims(1), dims(2), dims(3));
    manager.energyPheromones = ones(dims(1), dims(2), dims(3)) * manager.initialPheromone;
    
    % 添加长短期记忆信息素
    manager.shortTermPheromones = zeros(dims(1), dims(2), dims(3));
    manager.longTermPheromones = zeros(dims(1), dims(2), dims(3));
    
    % 缓存与可视化
    manager.queryCache = containers.Map('KeyType', 'char', 'ValueType', 'double');
    manager.cacheHits = 0;
    manager.cacheMisses = 0;
    manager.visualHandle = [];
    
    obj = manager;
end

%% P-G-DWA*算法实现
function planner = createPGDWAStar(obstacles, mapSize, gridSize, minHeight, params, pheromoneManager)
    % 创建P-G-DWA*规划器
    planner = struct();
    
    % 保存参数
    planner.obstacles = obstacles;
    planner.mapSize = mapSize;
    planner.gridSize = gridSize;
    planner.minHeight = minHeight;
    planner.astarPheromoneWeight = getfield_default(params, 'astarPheromoneWeight', 0.4);
    planner.dwaPheromoneWeight = getfield_default(params, 'dwaPheromoneWeight', 0.5);
    planner.diffusionRate = getfield_default(params, 'diffusionRate', 0.05);
    planner.pheromoneManager = pheromoneManager;
    
    % 初始化历史数据
    planner.pathHistory = {};
    planner.pathMetrics = [];
    planner.iteration = 0;
    
    % 自适应参数控制
    planner.adaptiveWeights = struct('astar', planner.astarPheromoneWeight, 'dwa', planner.dwaPheromoneWeight);
end

%% 信息素管理函数
    function evaporateAndDiffusePheromones(manager)
    % 信息素蒸发和扩散，分层处理

    % 路径层信息素蒸发
    manager.pathPheromones = (1 - manager.evaporationRate*0.8) * manager.pathPheromones;

    % 避障层信息素蒸发(更快)
    manager.avoidancePheromones = (1 - manager.evaporationRate*1.2) * manager.avoidancePheromones;

    % 能耗层信息素蒸发
    manager.energyPheromones = (1 - manager.evaporationRate*0.7) * manager.energyPheromones;

    % 集成层蒸发
    manager.pheromoneMatrix = (1 - manager.evaporationRate) * manager.pheromoneMatrix;

    % 信息素扩散处理
    if manager.diffusionRate > 0
        kernel = generateDiffusionKernel(1.0);
        
        % 各层扩散
        manager.pathPheromones = applyDiffusion(manager.pathPheromones, kernel, manager.diffusionRate*0.9);
        manager.avoidancePheromones = applyDiffusion(manager.avoidancePheromones, kernel, manager.diffusionRate*1.5);
        manager.energyPheromones = applyDiffusion(manager.energyPheromones, kernel, manager.diffusionRate*0.8);
        manager.pheromoneMatrix = applyDiffusion(manager.pheromoneMatrix, kernel, manager.diffusionRate);
    end

    % 更新集成信息素矩阵
    updateIntegratedPheromones(manager);

    % 更新长期记忆
    updateLongTermMemory(manager);

    % 清除查询缓存
    if manager.queryCache.Count > 1000
        manager.queryCache = containers.Map('KeyType', 'char', 'ValueType', 'double');
    end
end
function pheromones = applyDiffusion(pheromones, kernel, rate)
    % 应用扩散到信息素层
    diffused = convn(pheromones, kernel, 'same');
    pheromones = pheromones * (1.0 - rate) + diffused * rate;
end

function updateIntegratedPheromones(manager)
    % 更新集成信息素矩阵 - 自适应加权融合
    manager.pheromoneMatrix = manager.pathPheromones * manager.pathLayerWeight - ...
                              manager.avoidancePheromones * manager.avoidanceLayerWeight + ...
                              manager.energyPheromones * manager.energyLayerWeight;

    % 确保值在有效范围内
    manager.pheromoneMatrix = min(manager.maxPheromone, max(manager.initialPheromone, manager.pheromoneMatrix));
end

function updateLongTermMemory(manager)
    % 更新长期记忆信息素

    % 短期信息素自然衰减
    manager.shortTermPheromones = manager.shortTermPheromones * 0.95;

    % 将高于阈值的短期信息素转移到长期记忆
    threshold = manager.initialPheromone * 3;
    transferMask = manager.shortTermPheromones > threshold;
    transferAmount = manager.shortTermPheromones .* transferMask * 0.1;

    % 更新长期记忆
    manager.longTermPheromones = manager.longTermPheromones + transferAmount;
    manager.longTermPheromones = min(manager.longTermPheromones, manager.maxPheromone);

    % 从短期记忆中移除已转移部分
    manager.shortTermPheromones = manager.shortTermPheromones - transferAmount;
end

function kernel = generateDiffusionKernel(sigma)
    % 生成3D高斯扩散核
    size = 3;
    kernel = zeros(size, size, size);
    center = ceil(size/2);

    for i = 1:size
        for j = 1:size
            for k = 1:size
                dist = sqrt((i-center)^2 + (j-center)^2 + (k-center)^2);
                kernel(i,j,k) = exp(-dist^2/(2*sigma^2));
            end
        end
    end

    % 归一化
    kernel = kernel / sum(kernel(:));
end

function depositPheromones(manager, path, metrics)
    % 信息素沉积 - 为不同层分别沉积
    if size(path, 1) < 2
        return;
    end

    % 基础沉积量
    baseAmount = manager.pheromoneQ;

    % 计算沉积因子
    qualityFactor = calculateQualityFactor(metrics, manager);

    % 计算实际沉积量
    depositAmount = baseAmount * qualityFactor;

    % 沿路径沉积信息素
    for i = 1:size(path, 1)
        gridIdx = worldToGrid(manager, path(i,:));
        
        if isValidGrid(manager, gridIdx)
            % 更新路径层信息素
            currentValue = manager.pathPheromones(gridIdx(1), gridIdx(2), gridIdx(3));
            newValue = min(manager.maxPheromone, currentValue + depositAmount);
            manager.pathPheromones(gridIdx(1), gridIdx(2), gridIdx(3)) = newValue;
            
            % 如果有能耗信息，更新能耗层
            if isfield(metrics, 'energy_efficiency') && metrics.energy_efficiency > 0
                energyDeposit = depositAmount * metrics.energy_efficiency * 0.8;
                manager.energyPheromones(gridIdx(1), gridIdx(2), gridIdx(3)) = ...
                    min(manager.maxPheromone, manager.energyPheromones(gridIdx(1), gridIdx(2), gridIdx(3)) + energyDeposit);
            end
            
            % 更新集成矩阵
            manager.pheromoneMatrix(gridIdx(1), gridIdx(2), gridIdx(3)) = ...
                min(manager.maxPheromone, manager.pheromoneMatrix(gridIdx(1), gridIdx(2), gridIdx(3)) + depositAmount);
            
            % 更新短期记忆
            manager.shortTermPheromones(gridIdx(1), gridIdx(2), gridIdx(3)) = ...
                min(manager.maxPheromone, manager.shortTermPheromones(gridIdx(1), gridIdx(2), gridIdx(3)) + depositAmount * 0.9);
        end
    end

    % 更新集成信息素
    updateIntegratedPheromones(manager);
end

function qualityFactor = calculateQualityFactor(metrics, manager)
    % 计算路径质量因子
    if isfield(metrics, 'path_length') && metrics.path_length > 0
        lengthFactor = min(1.0, 100.0 / metrics.path_length);
    else
        lengthFactor = 0.5;
    end

    if isfield(metrics, 'smoothness')
        smoothnessFactor = metrics.smoothness;
    else
        smoothnessFactor = 0.5;
    end

    if isfield(metrics, 'energy_efficiency')
        energyFactor = metrics.energy_efficiency;
    else
        energyFactor = 0.5;
    end

    % 计算综合质量因子
    qualityFactor = manager.lengthWeight * lengthFactor + ...
                   manager.smoothnessWeight * smoothnessFactor + ...
                   (1 - manager.lengthWeight - manager.smoothnessWeight) * energyFactor;
end

function depositNegativePheromones(manager, obstaclePos, radius)
    % 避障信息素沉积
    centerIdx = worldToGrid(manager, obstaclePos);
    radiusGrid = ceil(radius / manager.gridSize);

    % 对影响范围内的每个栅格应用负信息素
    for dx = -radiusGrid:radiusGrid
        for dy = -radiusGrid:radiusGrid
            for dz = -radiusGrid:radiusGrid
                idx = centerIdx + [dx, dy, dz];
                if isValidGrid(manager, idx)
                    dist = sqrt(dx^2 + dy^2 + dz^2) * manager.gridSize;
                    if dist <= radius
                        % 计算基于距离的影响强度
                        factor = exp(-dist^2/(2*(radius/2.0)^2));
                        reduction = manager.maxPheromone * factor * 3.0;
                        
                        % 更新避障层信息素(作为负信息素)
                        manager.avoidancePheromones(idx(1), idx(2), idx(3)) = ...
                            manager.avoidancePheromones(idx(1), idx(2), idx(3)) + reduction * 3.0;
                        
                        % 减少路径层信息素
                        current = manager.pathPheromones(idx(1), idx(2), idx(3));
                        manager.pathPheromones(idx(1), idx(2), idx(3)) = ...
                            max(manager.initialPheromone * 0.1, current - reduction * 2.5);
                        
                        % 减少集成信息素
                        current = manager.pheromoneMatrix(idx(1), idx(2), idx(3));
                        manager.pheromoneMatrix(idx(1), idx(2), idx(3)) = ...
                            max(manager.initialPheromone * 0.1, current - reduction * 2.5);
                            
                        % 更新短期记忆
                        manager.shortTermPheromones(idx(1), idx(2), idx(3)) = ...
                            max(0, manager.shortTermPheromones(idx(1), idx(2), idx(3)) - reduction * 2.0);
                    end
                end
            end
        end
    end

    % 更新集成信息素
    updateIntegratedPheromones(manager);
end

function [pheromoneValue, gradient] = getPheromoneValue(manager, worldPos)
    % 获取指定世界坐标处的信息素值及其梯度
    
    % 使用缓存加速重复查询
    posKey = sprintf('%.2f,%.2f,%.2f', worldPos(1), worldPos(2), worldPos(3));
    if isKey(manager.queryCache, posKey)
        manager.cacheHits = manager.cacheHits + 1;
        pheromoneValue = manager.queryCache(posKey);
        
        % 计算梯度 - 不缓存梯度，总是重新计算以保证准确性
        gradient = calculatePheromoneGradient(manager, worldPos);
        return;
    end
    
    % 将世界坐标转换为网格索引
    gridIdx = worldToGrid(manager, worldPos);
    
    % 检查索引是否有效
    if isValidGrid(manager, gridIdx)
        % 获取信息素值 - 使用多层融合矩阵
        pheromoneValue = manager.pheromoneMatrix(gridIdx(1), gridIdx(2), gridIdx(3));
        
        % 加入长期记忆影响
        pheromoneValue = pheromoneValue + manager.longTermPheromones(gridIdx(1), gridIdx(2), gridIdx(3)) * 0.5;
    else
        % 无效索引，返回初始值
        pheromoneValue = manager.initialPheromone;
    end
    
    % 计算梯度
    gradient = calculatePheromoneGradient(manager, worldPos);
    
    % 更新缓存
    manager.queryCache(posKey) = pheromoneValue;
    manager.cacheMisses = manager.cacheMisses + 1;
end

function gradient = calculatePheromoneGradient(manager, worldPos)
    % 计算信息素梯度 - 使用中心差分法
    h = manager.gridSize * 0.5; % 半步长
    
    % X方向梯度
    posX1 = worldPos - [h, 0, 0];
    posX2 = worldPos + [h, 0, 0];
    [valX1, ~] = getPheromoneValueNoCache(manager, posX1);
    [valX2, ~] = getPheromoneValueNoCache(manager, posX2);
    gradX = (valX2 - valX1) / (2*h);
    
    % Y方向梯度
    posY1 = worldPos - [0, h, 0];
    posY2 = worldPos + [0, h, 0];
    [valY1, ~] = getPheromoneValueNoCache(manager, posY1);
    [valY2, ~] = getPheromoneValueNoCache(manager, posY2);
    gradY = (valY2 - valY1) / (2*h);
    
    % Z方向梯度
    posZ1 = worldPos - [0, 0, h];
    posZ2 = worldPos + [0, 0, h];
    [valZ1, ~] = getPheromoneValueNoCache(manager, posZ1);
    [valZ2, ~] = getPheromoneValueNoCache(manager, posZ2);
    gradZ = (valZ2 - valZ1) / (2*h);
    
    % 合成梯度向量
    gradient = [gradX, gradY, gradZ];
    
    % 归一化梯度（如果梯度不为零）
    gradNorm = norm(gradient);
    if gradNorm > 0
        gradient = gradient / gradNorm;
    end
end

function [pheromoneValue, isValid] = getPheromoneValueNoCache(manager, worldPos)
    % 无缓存版本的信息素查询，避免在梯度计算中递归调用缓存
    
    % 将世界坐标转换为网格索引
    gridIdx = worldToGrid(manager, worldPos);
    
    % 检查索引是否有效
    if isValidGrid(manager, gridIdx)
        % 获取信息素值 - 使用多层融合矩阵
        pheromoneValue = manager.pheromoneMatrix(gridIdx(1), gridIdx(2), gridIdx(3));
        isValid = true;
    else
        % 无效索引，返回初始值
        pheromoneValue = manager.initialPheromone;
        isValid = false;
    end
end

function gridIdx = worldToGrid(manager, worldPos)
    % 将世界坐标转换为网格索引
    gridIdx = max(1, min(ceil(worldPos / manager.gridSize), ceil(manager.mapSize / manager.gridSize)));
    gridIdx = round(gridIdx); % 确保索引是整数
end

function valid = isValidGrid(manager, gridIdx)
    % 检查网格索引是否有效
    dims = size(manager.pheromoneMatrix);
    valid = all(gridIdx >= 1) && all(gridIdx <= dims);
end

function curvature = analyzePheromoneDistributionCurvature(manager, pos)
    % 分析信息素分布曲率 - 使用Hessian矩阵的迹
    h = manager.gridSize;
    
    % 计算二阶导数
    % XX方向
    posXX1 = pos - [h, 0, 0];
    posXX2 = pos;
    posXX3 = pos + [h, 0, 0];
    [valXX1, ~] = getPheromoneValueNoCache(manager, posXX1);
    [valXX2, ~] = getPheromoneValueNoCache(manager, posXX2);
    [valXX3, ~] = getPheromoneValueNoCache(manager, posXX3);
    dxx = (valXX1 - 2*valXX2 + valXX3) / (h*h);
    
    % YY方向
    posYY1 = pos - [0, h, 0];
    posYY2 = pos;
    posYY3 = pos + [0, h, 0];
    [valYY1, ~] = getPheromoneValueNoCache(manager, posYY1);
    [valYY2, ~] = getPheromoneValueNoCache(manager, posYY2);
    [valYY3, ~] = getPheromoneValueNoCache(manager, posYY3);
    dyy = (valYY1 - 2*valYY2 + valYY3) / (h*h);
    
    % ZZ方向
    posZZ1 = pos - [0, 0, h];
    posZZ2 = pos;
    posZZ3 = pos + [0, 0, h];
    [valZZ1, ~] = getPheromoneValueNoCache(manager, posZZ1);
    [valZZ2, ~] = getPheromoneValueNoCache(manager, posZZ2);
    [valZZ3, ~] = getPheromoneValueNoCache(manager, posZZ3);
    dzz = (valZZ1 - 2*valZZ2 + valZZ3) / (h*h);
    
    % 曲率估计 - 使用Hessian矩阵的迹
    curvature = abs(dxx) + abs(dyy) + abs(dzz);
    
    % 归一化到[0,1]区间
    curvature = min(1.0, curvature / 10.0);
end

function visualizePheromoneSlices(manager, figHandle, queryPos)
    % 信息素可视化函数 - 支持实时交互式查询
    % queryPos: 可选参数,指定要查询的世界坐标 [x, y, z]
    
    if nargin < 2 || ~ishandle(figHandle); figHandle = gcf; end
    if nargin < 3; queryPos = []; end

    figure(figHandle);
    clf(figHandle);

    % ✅ 直接使用主矩阵（障碍物为负值）
    combinedPheromones = manager.pheromoneMatrix;
    
    % 获取数据维度
    dims = size(combinedPheromones);

    % 如果提供了查询点,使用该点;否则使用地图中心
    if ~isempty(queryPos)
        % 转换世界坐标到网格索引
        gridIdx = ceil(queryPos / manager.gridSize);
        % 确保索引在有效范围内
        gridIdx = max([1,1,1], min(gridIdx, dims));
        midX = gridIdx(1);
        midY = gridIdx(2);
        midZ = gridIdx(3);
        titleSuffix = sprintf(' (查询点: [%.1f, %.1f, %.1f])', queryPos(1), queryPos(2), queryPos(3));
    else
        midX = ceil(dims(1)/2);
        midY = ceil(dims(2)/2);
        midZ = ceil(dims(3)/2);
        titleSuffix = ' (地图中心)';
    end

    % ========== 关键修改：计算全局颜色范围 ==========
    % 计算整个信息素矩阵的最小值和最大值
    minVal = min(combinedPheromones(:));
    maxVal = max(combinedPheromones(:));
    
    % 如果所有值都相同，设置一个小的范围
    if minVal == maxVal
        minVal = minVal - 0.1;
        maxVal = maxVal + 0.1;
    end
    
    % 创建统一的颜色映射
    colormap(jet);
    
    % 创建子图 - 3个主切片,1个3D视图
    subplot(2, 2, 1); % XY平面切片
    xySlice = squeeze(combinedPheromones(:, :, midZ));
    imagesc([0 manager.mapSize(1)], [0 manager.mapSize(2)], xySlice);
    
    % ========== 关键修改：设置统一的颜色范围 ==========
    caxis([minVal, maxVal]);
    
    title(sprintf('XY平面信息素分布 (Z=%d)%s', midZ*manager.gridSize, titleSuffix));
    xlabel('X轴'); ylabel('Y轴');
    axis([0 manager.mapSize(1) 0 manager.mapSize(2)]);
    axis manual;
    set(gca, 'YDir', 'normal');

    % 标记查询点
    if ~isempty(queryPos)
        hold on;
        plot(queryPos(1), queryPos(2), 'w*', 'MarkerSize', 15, 'LineWidth', 2);
        plot(queryPos(1), queryPos(2), 'ko', 'MarkerSize', 8, 'LineWidth', 2);
        hold off;
    end

    % 添加 XY平面上的路径投影
    global originalPath finalPath
    hold on;
    if ~isempty(originalPath) 
        plot(originalPath(:,1), originalPath(:,2), 'k-', 'LineWidth', 1.5);
    end
    if ~isempty(finalPath)
        plot(finalPath(:,1), finalPath(:,2), 'w-', 'LineWidth', 1.5);
    end
    hold off;

    subplot(2, 2, 2); % XZ平面切片
    xzSlice = squeeze(combinedPheromones(:, midY, :));
    imagesc([0 manager.mapSize(1)], [0 manager.mapSize(3)], xzSlice');
    
    % ========== 关键修改：设置统一的颜色范围 ==========
    caxis([minVal, maxVal]);
    
    title(sprintf('XZ平面信息素分布 (Y=%d)%s', midY*manager.gridSize, titleSuffix));
    xlabel('X轴'); ylabel('Z轴');
    axis([0 manager.mapSize(1) 0 manager.mapSize(3)]);
    axis manual;
    set(gca, 'YDir', 'normal');

    % 标记查询点
    if ~isempty(queryPos)
        hold on;
        plot(queryPos(1), queryPos(3), 'w*', 'MarkerSize', 15, 'LineWidth', 2);
        plot(queryPos(1), queryPos(3), 'ko', 'MarkerSize', 8, 'LineWidth', 2);
        hold off;
    end

    % 添加 XZ平面上的路径投影
    hold on;
    if ~isempty(originalPath)
        plot(originalPath(:,1), originalPath(:,3), 'k-', 'LineWidth', 1.5);
    end
    if ~isempty(finalPath)
        plot(finalPath(:,1), finalPath(:,3), 'w-', 'LineWidth', 1.5);
    end
    hold off;

    subplot(2, 2, 3); % YZ平面切片
    yzSlice = squeeze(combinedPheromones(midX, :, :));
    imagesc([0 manager.mapSize(2)], [0 manager.mapSize(3)], yzSlice');
    
    % ========== 关键修改：设置统一的颜色范围 ==========
    caxis([minVal, maxVal]);
    
    title(sprintf('YZ平面信息素分布 (X=%d)%s', midX*manager.gridSize, titleSuffix));
    xlabel('Y轴'); ylabel('Z轴');
    axis([0 manager.mapSize(2) 0 manager.mapSize(3)]);
    axis manual;
    set(gca, 'YDir', 'normal');

    % 标记查询点
    if ~isempty(queryPos)
        hold on;
        plot(queryPos(2), queryPos(3), 'w*', 'MarkerSize', 15, 'LineWidth', 2);
        plot(queryPos(2), queryPos(3), 'ko', 'MarkerSize', 8, 'LineWidth', 2);
        hold off;
    end

    % 添加 YZ平面上的路径投影
    hold on;
    if ~isempty(originalPath)
        plot(originalPath(:,2), originalPath(:,3), 'k-', 'LineWidth', 1.5);
    end
    if ~isempty(finalPath)
        plot(finalPath(:,2), finalPath(:,3), 'w-', 'LineWidth', 1.5);
    end
    hold off;

    % ========== 关键修改：创建共享的颜色条 ==========
    % 调整子图位置，为颜色条腾出空间
    subplot(2, 2, 4);
    pos = get(gca, 'Position');
    
    % 创建颜色条，放在右侧
    c = colorbar('Location', 'eastoutside');
    c.Label.String = '信息素浓度';
    c.Label.FontSize = 12;
    c.Label.FontWeight = 'bold';
    
    % 设置颜色条的范围
    caxis([minVal, maxVal]);
    
    % 恢复子图位置
    set(gca, 'Position', pos);

    % 可选的3D视图
    % 创建世界坐标网格
    [X, Y, Z] = meshgrid(1:dims(2), 1:dims(1), 1:dims(3));
    X = X * manager.gridSize;
    Y = Y * manager.gridSize;
    Z = Z * manager.gridSize;

    % 只显示高于阈值的值 - 使用等值面
    meanVal = mean(combinedPheromones(:));
    stdVal = std(combinedPheromones(:));

    % 定义高阈值
    threshold = meanVal + 0.6 * stdVal;

    % 绘制等值面
    p = patch(isosurface(X, Y, Z, combinedPheromones, threshold));
    isonormals(X, Y, Z, combinedPheromones, p);
    set(p, 'FaceColor', 'red', 'EdgeColor', 'none', 'FaceAlpha', 0.5);

    % 添加路径
    hold on;
    if ~isempty(originalPath)
        plot3(originalPath(:,1), originalPath(:,2), originalPath(:,3), 'k-', 'LineWidth', 1.5);
    end
    if ~isempty(finalPath)
        plot3(finalPath(:,1), finalPath(:,2), finalPath(:,3), 'w-', 'LineWidth', 1.5);
    end

    % 标记查询点
    if ~isempty(queryPos)
        plot3(queryPos(1), queryPos(2), queryPos(3), 'w*', 'MarkerSize', 20, 'LineWidth', 3);
        plot3(queryPos(1), queryPos(2), queryPos(3), 'ko', 'MarkerSize', 10, 'LineWidth', 2);
        
        % 添加查询点标签
        text(queryPos(1), queryPos(2), queryPos(3)+5, '查询点', ...
            'Color', 'w', 'FontWeight', 'bold', 'FontSize', 12, ...
            'HorizontalAlignment', 'center', 'BackgroundColor', [0 0 0 0.5]);
    end

    title(sprintf('3D信息素分布 (红色 = 高浓度区域)%s', titleSuffix));
    xlabel('X轴'); ylabel('Y轴'); zlabel('Z轴');
    axis([0 manager.mapSize(1) 0 manager.mapSize(2) 0 manager.mapSize(3)]);
    grid on;
    view(3);
    camlight;
    lighting gouraud;

    % 添加信息素统计信息
    annotation('textbox', [0.1, 0.01, 0.8, 0.04], 'String', ...
        sprintf('信息素统计: 最小=%.2f, 最大=%.2f, 平均=%.2f, 标准差=%.2f', ...
                minVal, maxVal, meanVal, stdVal), ...
        'FontSize', 9, 'FitBoxToText', 'on', 'BackgroundColor', [1 1 1 0.7], ...
        'HorizontalAlignment', 'center');
    
    % ═══════════════════════════════════════════════
    % 添加实时交互式查询功能
    % ═══════════════════════════════════════════════
    
    % 设置窗口的ButtonDownFcn
    set(figHandle, 'WindowButtonDownFcn', @(src, event) clickQueryCallback(src, event, manager));
    
    % 添加说明文本
    annotation('textbox', [0.02, 0.95, 0.3, 0.04], 'String', ...
        '💡 点击任意切片图查询该位置信息素', ...
        'FontSize', 10, 'FontWeight', 'bold', 'EdgeColor', 'none', ...
        'BackgroundColor', [1 1 0.8 0.8], 'HorizontalAlignment', 'left');
end
function path = smoothPath(path, obstacles)
    % 路径平滑处理 - 改进算法减少不必要的转弯
    if size(path, 1) <= 2
        return;
    end

    smoothed = path(1,:); % 起点
    i = 1;

    while i < size(path, 1)
        current = path(i,:);
        
        % 尝试跳过中间点直接连接
        for j = min(i+3, size(path, 1)):-1:i+1
            target = path(j,:);
            
            % 检查直线路径是否无碰撞
            collision = checkPathCollision(current, target, obstacles);
            
            if ~collision
                % 可以直接连接，跳过中间点
                i = j-1;
                break;
            end
        end
        
        i = i + 1;
        if i < size(path, 1)
            smoothed = [smoothed; path(i,:)];
        end
    end

    % 确保终点包含在内
    if ~isequal(smoothed(end,:), path(end,:))
        smoothed = [smoothed; path(end,:)];
    end

    path = smoothed;
end

function smoothedPath = smoothPathPoints(path, obstacles)
    % 增强路径平滑处理
    smoothedPath = enhancedSmoothPath(path, obstacles, 0.6, 5);
end

function smoothedPath = enhancedSmoothPath(path, obstacles, smoothWeight, iterations)
    % 如果路径点太少，直接返回
    if size(path, 1) <= 2
        smoothedPath = path;
        return;
    end
    
    % 创建工作副本
    smoothedPath = path;
    
    % 定义平滑参数
    alpha = 0.1;  % 原路径权重 - 较小值使平滑更明显
    beta = 1.0 - alpha;  % 直线拉伸权重
    
    % 执行迭代平滑
    for iter = 1:iterations
        % 从第二个点到倒数第二个点进行平滑
        for i = 2:size(smoothedPath, 1)-1
            % 保存原始点
            oldPoint = smoothedPath(i, :);
            
            % 计算相邻点的中间位置 (直线拉伸)
            prev = smoothedPath(i-1, :);
            next = smoothedPath(i+1, :);
            
            % 直线拉伸 - 将点向相邻点的连线移动
            linePoint = (prev + next) / 2;
            
            % 加权平均原点和直线点
            newPoint = oldPoint * alpha + linePoint * beta;
            
            % 检查新点是否会导致碰撞
            if ~checkCollision(newPoint, obstacles)
                smoothedPath(i, :) = newPoint;
            end
        end
    end
end

function densePath = safeAddIntermediatePoints(path, obstacles, maxSpacing)
    % 安全添加中间点函数，确保插入的点不会与障碍物碰撞
    if size(path, 1) < 2
        densePath = path;
        return;
    end

    densePath = path(1,:); % 第一个点

    for i = 1:size(path,1)-1
        currentPoint = path(i,:);
        nextPoint = path(i+1,:);
        
        % 计算两点之间的距离
        segmentLength = norm(nextPoint - currentPoint);
        
        % 如果距离大于最大间距，则添加中间点
        if segmentLength > maxSpacing
            % 计算需要添加的点数
            numPoints = ceil(segmentLength / maxSpacing) - 1;
            
            % 添加中间点
            for j = 1:numPoints
                t = j / (numPoints + 1);
                interpolatedPoint = currentPoint + t * (nextPoint - currentPoint);
                
                % 确保插入点不会与障碍物碰撞
                if ~checkCollision(interpolatedPoint, obstacles)
                    densePath = [densePath; interpolatedPoint];
                else
                    % 如果点在障碍物内部，尝试修改高度
                    adjustedPoint = interpolatedPoint;
                    adjustedPoint(3) = adjustedPoint(3) + 2.0;
                    
                    if ~checkCollision(adjustedPoint, obstacles)
                        densePath = [densePath; adjustedPoint];
                    end
                    % 如果调整后仍有碰撞，则跳过该点
                end
            end
        end
        
        % 添加下一个原始点
        densePath = [densePath; nextPoint];
    end
end

function segments = calculatePathSegmentInfo(path)
    % 预计算路径段信息，用于计算曲率
    segments = struct('startIdx', {}, 'endIdx', {}, 'length', {}, 'curvature', {}, 'direction', {});

    % 路径段至少需要两个点
    if size(path, 1) < 2
        return;
    end

    % 计算每段路径的信息
    for i = 1:size(path, 1)-1
        segment = struct();
        segment.startIdx = i;
        segment.endIdx = i+1;
        
        % 计算路径段长度和方向
        segment.direction = path(i+1,:) - path(i,:);
        segment.length = norm(segment.direction);
        
        if segment.length > 0
            segment.direction = segment.direction / segment.length; % 归一化
        end
        
        % 计算曲率 - 如果有足够的点
        if i > 1 && i < size(path, 1)-1
            % 使用相邻三个点估计曲率
            p1 = path(i-1,:);
            p2 = path(i,:);
            p3 = path(i+1,:);
            
            % 计算两个方向向量
            v1 = p2 - p1;
            v2 = p3 - p2;
            
            % 向量夹角的变化率可以反映曲率
            v1_norm = norm(v1);
            v2_norm = norm(v2);
            
            if v1_norm > 0 && v2_norm > 0
                v1 = v1 / v1_norm;
                v2 = v2 / v2_norm;
                dot_product = dot(v1, v2);
                % 防止数值误差导致的问题
                dot_product = min(1, max(-1, dot_product));
                angle = acos(dot_product);
                
                % 曲率与转弯角度和路径长度有关
                segment.curvature = angle / segment.length;
            else
                segment.curvature = 0;
            end
        else
            segment.curvature = 0;
        end
        
        segments(end+1) = segment;
    end
end

function path = pheromoneWeightedAStar(planner, start, goal)
    % 使用信息素加权的A*算法进行三维路径规划

    % 先进行直接路径检查
    directPath = checkDirectPath(planner, start, goal);
    if ~isempty(directPath)
        disp('找到直接路径，跳过A*搜索');
        path = directPath;
        return;
    end

    % 使用优先队列实现开放列表
    openList = createPriorityQueue();
    closedList = containers.Map('KeyType', 'char', 'ValueType', 'logical');
    nodes = containers.Map('KeyType', 'char', 'ValueType', 'any');

    % 添加起点到开放列表
    startNode = struct('position', start, 'g', 0, 'h', 0, 'f', 0, 'parent', '', 'direction', [0,0,0]);

    % 计算改进的启发式值
    startNode.h = calculateEnhancedHeuristic(start, goal, 0, 0);
    startNode.f = startNode.g + startNode.h;

    startKey = sprintf('%.0f,%.0f,%.0f', start(1), start(2), start(3));
    nodes(startKey) = startNode;
    openList = queueInsert(openList, startKey, startNode.f);

    % 定义方向集合 - 26连通性
    directions = [
        % 6个基本方向
        planner.gridSize, 0, 0;     % 东
        -planner.gridSize, 0, 0;    % 西
        0, planner.gridSize, 0;     % 北
        0, -planner.gridSize, 0;    % 南
        0, 0, planner.gridSize;     % 上
        0, 0, -planner.gridSize;    % 下
        
        % 平面对角线方向
        planner.gridSize, planner.gridSize, 0;      % 东北
        planner.gridSize, -planner.gridSize, 0;     % 东南
        -planner.gridSize, planner.gridSize, 0;     % 西北
        -planner.gridSize, -planner.gridSize, 0;    % 西南
        
        % 垂直对角线方向
        planner.gridSize, 0, planner.gridSize;      % 东上
        -planner.gridSize, 0, planner.gridSize;     % 西上
        0, planner.gridSize, planner.gridSize;      % 北上
        0, -planner.gridSize, planner.gridSize;     % 南上
        planner.gridSize, 0, -planner.gridSize;     % 东下
        -planner.gridSize, 0, -planner.gridSize;    % 西下
        0, planner.gridSize, -planner.gridSize;     % 北下
        0, -planner.gridSize, -planner.gridSize;    % 南下
        
        % 完整的体对角线方向
        planner.gridSize, planner.gridSize, planner.gridSize;       % 东北上
        planner.gridSize, planner.gridSize, -planner.gridSize;      % 东北下
        planner.gridSize, -planner.gridSize, planner.gridSize;      % 东南上
        planner.gridSize, -planner.gridSize, -planner.gridSize;     % 东南下
        -planner.gridSize, planner.gridSize, planner.gridSize;      % 西北上
        -planner.gridSize, planner.gridSize, -planner.gridSize;     % 西北下
        -planner.gridSize, -planner.gridSize, planner.gridSize;     % 西南上
        -planner.gridSize, -planner.gridSize, -planner.gridSize;    % 西南下
    ];

    % 获取方向的代价
    costs = zeros(size(directions, 1), 1);
    for i = 1:size(directions, 1)
        costs(i) = norm(directions(i, :));
    end

    % 放宽目标到达条件
    goalReachedThreshold = planner.gridSize * 1.5;

    % A*搜索循环
    iterations = 0;
    maxIterations = 5000; % 限制最大迭代次数以避免无限循环

    while ~queueIsEmpty(openList) && iterations < maxIterations
        iterations = iterations + 1;
        
        % 获取f值最小的节点
        [openList, currentKey] = queuePop(openList);
        if isempty(currentKey)
            continue; % 跳过空键
        end
        
        % 检查当前键是否存在
        if ~isKey(nodes, currentKey)
            continue; % 如果键不存在，跳过此次循环
        end
        
        currentNode = nodes(currentKey);
        
        % 检查是否到达目标
        if norm(currentNode.position - goal) < goalReachedThreshold
            % 重建路径
            path = rebuildPath(nodes, currentKey);
            
            % 确保路径包含精确的终点
            if ~isequal(path(end,:), goal)
                path = [path; goal];
            end
            
            return;
        end
        
        % 将当前节点加入关闭列表
        closedList(currentKey) = true;
        
        % 扩展相邻节点
        for i = 1:size(directions, 1)
            dir = directions(i, :);
            cost = costs(i);
            
            % 计算新位置
            neighborPos = currentNode.position + dir;
            
            % 在评估邻居节点时，额外考虑高度
            if neighborPos(3) < planner.minHeight
                continue; % 低于最小高度，不考虑
            end
            
            % 检查是否在地图范围内
            if neighborPos(1) < 1 || neighborPos(1) > planner.mapSize(1) || ...
               neighborPos(2) < 1 || neighborPos(2) > planner.mapSize(2) || ...
               neighborPos(3) > planner.mapSize(3)
                continue;
            end
            
            % 改进: 方向惩罚，避免回头和多余转弯
            if ~isempty(currentNode.direction) && norm(currentNode.direction) > 0
                % 计算方向变化的角度
                if norm(dir) > 0 && norm(currentNode.direction) > 0
                    dirUnit = dir / norm(dir);
                    currentDirUnit = currentNode.direction / norm(currentNode.direction);
                    dotProduct = dot(dirUnit, currentDirUnit);
                    
                    % 避免回头（方向夹角>135度）
                    if dotProduct < -0.7
                        continue; % 跳过该方向
                    end
                    
                    % 避免大角度转弯（90-135度），除非必要
                    if dotProduct < 0
                        toGoal = goal - currentNode.position;
                        if norm(toGoal) > 0
                            toGoalUnit = toGoal / norm(toGoal);
                            % 计算新方向与目标方向的夹角
                            goalAlignment = dot(dirUnit, toGoalUnit);
                            
                            % 如果新方向不朝向目标，则跳过
                            if goalAlignment < 0.3
                                continue;
                            end
                        end
                    end
                end
            end
            
            % 检查是否是障碍物 - 增强的碰撞检测
            if checkCollision(neighborPos, planner.obstacles)
                continue;
            end
            
            neighborKey = sprintf('%.0f,%.0f,%.0f', neighborPos(1), neighborPos(2), neighborPos(3));
            
            % 如果在关闭列表中，跳过
            if isKey(closedList, neighborKey) && closedList(neighborKey)
                continue;
            end
            
            % 计算到邻居的新g值
            newG = currentNode.g + cost;
            
            % 路径平滑度考虑 - 对方向变化进行惩罚
            directionChangePenalty = 0;
            if ~isempty(currentNode.direction) && norm(currentNode.direction) > 0
                currentDir = currentNode.direction / norm(currentNode.direction);
                newDir = dir / norm(dir);
                % 计算方向变化惩罚（基于夹角）
                dirCos = dot(currentDir, newDir);
                directionChangePenalty = (1 - dirCos) * cost * 0.5;
            end
            
            % 应用平滑度惩罚
            newG = newG + directionChangePenalty;
            
            % 获取信息素值和梯度
            [pheromoneValue, pheromoneGradient] = getPheromoneValue(planner.pheromoneManager, neighborPos);
            pheromoneWeight = planner.adaptiveWeights.astar;
            
            % 检查是否已经在开放列表中
            if ~isKey(nodes, neighborKey)
                % 计算信息素增强的启发式值
                adjustedH = calculateEnhancedHeuristic(neighborPos, goal, pheromoneValue, pheromoneWeight);
                
                % 考虑信息素梯度方向
                if norm(pheromoneGradient) > 0.1
                    toGoal = goal - neighborPos;
                    if norm(toGoal) > 0
                        toGoalUnit = toGoal / norm(toGoal);
                        gradientAlignment = dot(pheromoneGradient, toGoalUnit);
                        
                        % 如果梯度与目标方向一致，降低代价
                        if gradientAlignment > 0
                            adjustedH = adjustedH * (1 - gradientAlignment * 0.2 * pheromoneWeight);
                        end
                    end
                end
                
                % 创建新节点
                neighborNode = struct('position', neighborPos, ...
                                     'g', newG, ...
                                     'h', adjustedH, ...
                                     'f', newG + adjustedH, ...
                                     'parent', currentKey, ...
                                     'direction', dir);
                nodes(neighborKey) = neighborNode;
                openList = queueInsert(openList, neighborKey, neighborNode.f);
            else
                % 节点已经在开放列表中，检查是否找到了更好的路径
                neighborNode = nodes(neighborKey);
                if newG < neighborNode.g
                    % 更新节点
                    neighborNode.g = newG;
                    neighborNode.parent = currentKey;
                    neighborNode.direction = dir; % 更新方向
                    
                    % 更新启发式 - 可选，通常不需要更新
                    adjustedH = calculateEnhancedHeuristic(neighborPos, goal, pheromoneValue, pheromoneWeight);
                    neighborNode.h = adjustedH;
                    
                    neighborNode.f = newG + neighborNode.h;
                    nodes(neighborKey) = neighborNode;
                    
                    % 如果已在开放列表中，更新优先级
                    openList = queueUpdatePriority(openList, neighborKey, neighborNode.f);
                end
            end
        end
    end

    % 超出迭代限制或未找到路径
    disp(['A*搜索未找到路径，迭代次数: ', num2str(iterations)]);
    path = [];
end

function hValue = calculateEnhancedHeuristic(pos, goal, pheromoneValue, pheromoneWeight)
    % 增强型启发式函数计算

    % 基础启发式 - 曼哈顿距离与欧式距离的混合
    distance = norm(goal - pos);
    manhattan = sum(abs(goal - pos));
    baseH = 0.7 * distance + 0.3 * manhattan;

    % 添加非线性缩放以改善大距离表现
    sigmoid = 2/(1 + exp(-distance/20)) - 1;
    scaledBaseH = baseH * (1.5 * sigmoid + 0.8);

    % 信息素加权调整 - 增强信息素影响
    if pheromoneValue > 0.05 && pheromoneWeight > 0
        pheromoneRatio = min(1.0, pheromoneValue / 100.0);
        pheromoneInfluence = 1.0 - pheromoneWeight * pheromoneRatio * 1.2; % 增强影响
        
        % 应用信息素影响
        hValue = scaledBaseH * pheromoneInfluence;
    else
        hValue = scaledBaseH;
    end
end

function path = checkDirectPath(planner, start, goal)
    % 检查起点和终点之间是否有直接无碰撞路径
    path = [];

    % 生成直线上的点
    steps = max(20, ceil(norm(goal - start) / 2));
    directPath = zeros(steps, 3);

    for i = 1:steps
        t = (i-1) / (steps-1);
        directPath(i,:) = start * (1-t) + goal * t;
    end

    % 检查每个点是否有碰撞
    for i = 1:steps
        if checkCollision(directPath(i,:), planner.obstacles)
            return; % 发现碰撞，返回空路径
        end
    end

    % 无碰撞，返回直接路径
    path = directPath;
end

function path = rebuildPath(nodes, goalKey)
    % 重建路径
    path = [];
    currentKey = goalKey;

    % 检查键是否存在
    if ~isKey(nodes, currentKey)
        disp('警告: 路径重建失败 - 键不存在');
        return;
    end

    while isKey(nodes, currentKey) && ~strcmp(currentKey, '')
        currentNode = nodes(currentKey);
        path = [currentNode.position; path];
        currentKey = currentNode.parent;
    end
end

function keyPoints = extractKeyPoints(planner, path)
    % 提取路径中的关键节点
    if size(path, 1) <= 2
        keyPoints = path;
        return;
    end

    % 使用角度变化提取关键点
    keyPoints = path(1,:);  % 起点始终是关键点

    % 角度阈值
    angleThreshold = 20 * pi/180;  % 20度

    for i = 2:size(path,1)-1
        % 计算前后方向向量
        prevVec = path(i,:) - path(i-1,:);
        nextVec = path(i+1,:) - path(i,:);
        
        % 归一化
        if norm(prevVec) > 0
            prevVec = prevVec / norm(prevVec);
        end
        if norm(nextVec) > 0
            nextVec = nextVec / norm(nextVec);
        end
        
        % 计算夹角
        cosAngle = dot(prevVec, nextVec);
        cosAngle = min(1, max(-1, cosAngle));  % 确保在[-1,1]范围内
        angle = acos(cosAngle);
        
        % 如果角度变化大，则为关键点
        % 或者如果与上一个关键点距离太远
        lastKeyPoint = keyPoints(end,:);
        distToLast = norm(path(i,:) - lastKeyPoint);
        
        if angle > angleThreshold || distToLast > 20
            keyPoints = [keyPoints; path(i,:)];
        end
        
        % 信息素浓度异常点也可以考虑为关键点 - 增强信息素导航
        pheromoneValue = getPheromoneValue(planner.pheromoneManager, path(i,:));
        avgPheromone = planner.pheromoneManager.initialPheromone * 2;
        
        if pheromoneValue > avgPheromone * 2.5 || pheromoneValue < avgPheromone * 0.4 % 调整阈值
            % 显著高或低的信息素点
            if size(keyPoints, 1) < 12 % 增加关键点数量上限
                % 避免添加太接近的点
                if i > 1 && norm(path(i,:) - keyPoints(end,:)) > 4.5
                    keyPoints = [keyPoints; path(i,:)];
                end
            end
        end
    end

    % 终点始终是关键点
    keyPoints = [keyPoints; path(end,:)];
end

function metrics = calculatePathMetrics(planner, path)
    % 计算路径评估指标
    metrics = struct();

    if size(path, 1) < 2
        metrics.path_length = 0;
        metrics.smoothness = 0;
        metrics.avg_pheromone = 0;
        metrics.max_altitude = 0;
        metrics.min_altitude = 0;
        metrics.energy_efficiency = 0.5; % 默认能效
        return;
    end

    % 路径长度
    path_length = 0;
    for i = 1:size(path,1)-1
        path_length = path_length + norm(path(i+1,:) - path(i,:));
    end
    metrics.path_length = path_length;

    % 平滑度 (基于转弯角度)
    total_angle = 0;
    angle_count = 0;

    for i = 2:size(path,1)-1
        v1 = path(i,:) - path(i-1,:);
        v2 = path(i+1,:) - path(i,:);
        
        if norm(v1) > 0 && norm(v2) > 0
            v1 = v1 / norm(v1);
            v2 = v2 / norm(v2);
            
            cos_angle = dot(v1, v2);
            cos_angle = max(-1, min(1, cos_angle));  % 确保范围在[-1,1]内
            
            % 角度变化 (0表示180度转弯，1表示不转弯)
            smoothness = (cos_angle + 1) / 2;
            total_angle = total_angle + smoothness;
            angle_count = angle_count + 1;
        end
    end

    if angle_count > 0
        metrics.smoothness = total_angle / angle_count;
    else
        metrics.smoothness = 1;  % 默认最大平滑度
    end

    % 平均信息素
    pheromones = zeros(size(path, 1), 1);
    for i = 1:size(path, 1)
        pheromones(i) = getPheromoneValue(planner.pheromoneManager, path(i,:));
    end
    metrics.avg_pheromone = mean(pheromones);

    % 最大和最小高度
    metrics.max_altitude = max(path(:,3));
    metrics.min_altitude = min(path(:,3));

    % 能量效率估计 - 基于高度变化和转弯
    heightChanges = 0;
    for i = 1:size(path,1)-1
        heightChanges = heightChanges + abs(path(i+1,3) - path(i,3));
    end

    % 高度变化越大，效率越低
    heightEfficiency = max(0.1, 1 - heightChanges / (path_length + 0.001));

    % 平滑度越高，效率越高
    metrics.energy_efficiency = 0.7 * heightEfficiency + 0.3 * metrics.smoothness;
end

%% 避障和控制函数
function [pheromoneInfo, adaptiveWeight] = analyzeLocalPheromoneSpace(manager, pos, state, pathDist, baseWeight)
    % 创新分析：多维信息素空间分析
    pheromoneInfo = struct();

    % 获取当前位置信息素和梯度
    [pheromoneInfo.currentValue, pheromoneInfo.gradient] = getPheromoneValue(manager, pos);
    pheromoneInfo.pheromoneManager = manager; % 保存管理器引用

    % 获取各层信息素
    gridIdx = worldToGrid(manager, pos);
    if isValidGrid(manager, gridIdx)
        pheromoneInfo.pathLayer = manager.pathPheromones(gridIdx(1), gridIdx(2), gridIdx(3));
        pheromoneInfo.avoidanceLayer = manager.avoidancePheromones(gridIdx(1), gridIdx(2), gridIdx(3));
        pheromoneInfo.energyLayer = manager.energyPheromones(gridIdx(1), gridIdx(2), gridIdx(3));
    else
        pheromoneInfo.pathLayer = 0;
        pheromoneInfo.avoidanceLayer = 0;
        pheromoneInfo.energyLayer = 0;
    end

    % 信息素曲率分析
    pheromoneInfo.curvature = analyzePheromoneDistributionCurvature(manager, pos);

    % 自适应权重计算
    pathDeviation = min(1.0, pathDist / 3.0);
    pheromoneStrength = min(1.0, pheromoneInfo.currentValue / 50.0);

    % 自适应权重，对信息素敏感度更高
    adaptiveWeight = baseWeight * (1.0 + pathDeviation) * (1.0 + pheromoneStrength * 1.2); 
    adaptiveWeight = min(0.8, max(0.2, adaptiveWeight));

    pheromoneInfo.adaptiveWeight = adaptiveWeight;
end

function blendedDir = calculateBlendedDirection(pathDir, pathWeight, obstacleDir, avoidWeight, pheromoneInfo)
    % 获取信息素梯度增强导向
    pheromoneGradient = pheromoneInfo.gradient;
    pathPheromone = pheromoneInfo.pathLayer;
    avoidancePheromone = pheromoneInfo.avoidanceLayer;

    % 基础混合
    blendedDir = pathDir * pathWeight + obstacleDir * avoidWeight;

    % 添加信息素梯度影响 - 增强影响
    if norm(pheromoneGradient) > 0.05
        % 计算梯度与路径方向的一致性
        gradientAlignment = dot(pheromoneGradient, pathDir);
        
        % 如果在路径信息素高的区域，梯度与路径一致时加强影响
        if pathPheromone > 5.0 && gradientAlignment > 0
            gradientFactor = min(0.4, (pathPheromone / 50.0) * 0.4); 
            blendedDir = blendedDir + pheromoneGradient * gradientFactor;
        end
        
        % 如果在避障信息素高的区域，梯度与避障方向一致时加强影响
        if avoidancePheromone > 5.0
            gradientVsAvoid = dot(pheromoneGradient, -obstacleDir);
            if gradientVsAvoid > 0
                avoidGradientFactor = min(0.4, (avoidancePheromone / 50.0) * 0.4);
                blendedDir = blendedDir + pheromoneGradient * avoidGradientFactor;
            end
        end
    end

    if norm(blendedDir) > 0
        blendedDir = blendedDir / norm(blendedDir);
    else
        blendedDir = pathDir;
    end

    % 确保方向向量永远不为零
    if norm(blendedDir) < 0.01
        blendedDir = [0, 0, 1]; % 默认向上
    end
end

function pheromoneFactor = calculateEnhancedPheromoneFactor(pheromoneInfo)
    % 增强的信息素影响因子计算

    pheromoneValue = pheromoneInfo.currentValue;
    pathPheromone = pheromoneInfo.pathLayer;
    avoidancePheromone = pheromoneInfo.avoidanceLayer;

    baseFactor = 1.0;

    % 基于信息素值的基本调整 - 增强信息素影响
    if pheromoneValue > 1.0
        pheromoneRatio = min(1.0, pheromoneValue / 50.0);
        factorAdjustment = pheromoneRatio * 0.6;
        
        % 是否是路径上的高信息素
        if pathPheromone > avoidancePheromone
            baseFactor = baseFactor + factorAdjustment;
        else
            baseFactor = baseFactor - factorAdjustment * 0.5;
        end
    end

    % 避障信息素特别处理
    if avoidancePheromone > 10.0
        avoidRatio = min(1.0, avoidancePheromone / 30.0);
        baseFactor = baseFactor * (1.0 - avoidRatio * 0.3);
    end

    % 确保因子在合理范围
    pheromoneFactor = max(0.7, min(1.5, baseFactor));
end

function [omega, pitchRate, v, vz] = calculatePheromoneAwareControlOutputs(...
    state, targetYaw, targetPitch, maxOmega, maxSpeed, maxVz, ...
    targetPoint, currentPos, obstacleDistance, pheromoneInfo)

    % 信息素感知的控制输出计算
    yaw = state(4);
    pitch = state(5);

    % 信息素影响 - 增强信息素影响
    pheromoneFactor = calculateEnhancedPheromoneFactor(pheromoneInfo);
    pheromoneFactor = pheromoneFactor * 1.2; 

    % 角速度计算
    yawError = wrapToPi(targetYaw - yaw);
    omega = min(maxOmega, max(-maxOmega, yawError * 2.5 * pheromoneFactor));

    % 俯仰角速度
    pitchError = targetPitch - pitch;
    pitchRate = min(maxOmega, max(-maxOmega, pitchError * 2.0 * pheromoneFactor));

    % 速度计算 - 考虑信息素影响
    baseSpeed = calculatePheromoneAdaptiveSpeed(state, pheromoneInfo, maxSpeed);

    % 当前信息素值影响速度
    pheromoneValue = pheromoneInfo.currentValue;
    if pheromoneValue > 0.5
        pheromoneRatio = min(1.0, (pheromoneValue - 0.5) / 50.0);
        speedAdjustment = 1.0 + pheromoneRatio * 0.4; 
        v = min(maxSpeed, baseSpeed * speedAdjustment);
    else
        v = baseSpeed;
    end

    % 垂直速度
    heightDiff = targetPoint(3) - currentPos(3);
    vz = min(maxVz, max(-maxVz, heightDiff * pheromoneFactor));

    % 障碍物距离调整
    if obstacleDistance < 8.0
        v = v * max(0.4, min(1.0, obstacleDistance / 8.0));
    end
end

function baseSpeed = calculatePheromoneAdaptiveSpeed(state, pheromoneInfo, maxSpeed)
    % 信息素自适应速度计算

    % 信息素场的速度影响
    pheromoneValue = pheromoneInfo.currentValue;
    gradientStrength = norm(pheromoneInfo.gradient);
    curvature = pheromoneInfo.curvature;

    % 基础速度 (范围0.35-0.8 maxSpeed)
    baseSpeed = maxSpeed * 0.6;

    % 信息素梯度强度调整 - 梯度大表示变化快，需要减速
    if gradientStrength > 0.1
        gradientFactor = max(0.7, 1.0 - gradientStrength);
        baseSpeed = baseSpeed * gradientFactor;
    end

    % 信息素曲率调整 - 曲率大表示转弯，需要减速
    if curvature > 0.1
        curvatureFactor = max(0.5, 1.0 - curvature * 2.0);
        baseSpeed = baseSpeed * curvatureFactor;
    end

    % 信息素强度调整 - 高信息素区域可以稍微加速
    if pheromoneValue > 5.0 && gradientStrength < 0.05 && curvature < 0.1
        pheromoneBoost = min(1.2, 1.0 + (pheromoneValue - 5.0) / 50.0 * 0.25); 
        baseSpeed = baseSpeed * pheromoneBoost;
    end

    % 确保最小速度
    baseSpeed = max(maxSpeed * 0.35, min(maxSpeed * 0.8, baseSpeed));
end

function baseSpeed = calculatePrecisePathSpeed(pos, flightPath, closestIdx, maxSpeed, state)
    % 根据路径曲率计算精确的自适应速度 - 严格路径跟随用
    baseSpeed = maxSpeed * 0.5;

    % 确保索引有效
    if isempty(flightPath) || size(flightPath, 1) < 3 || closestIdx >= size(flightPath, 1)
        return;
    end

    % 计算局部曲率
    nextIdx = min(closestIdx+1, size(flightPath, 1));
    prevIdx = max(closestIdx-1, 1);

    if nextIdx == closestIdx || prevIdx == closestIdx
        return;
    end

    % 计算向量
    v1 = flightPath(closestIdx,:) - flightPath(prevIdx,:);
    v2 = flightPath(nextIdx,:) - flightPath(closestIdx,:);

    % 计算向量长度
    len1 = norm(v1);
    len2 = norm(v2);

    if len1 < 0.01 || len2 < 0.01
        return;
    end

    % 归一化向量
    v1 = v1 / len1;
    v2 = v2 / len2;

    % 计算曲率估计
    dot_product = dot(v1, v2);
    dot_product = min(1, max(-1, dot_product));
    angle = acos(dot_product);

    % 根据曲率调整速度
    curvatureEffect = exp(-angle * 2.5);
    baseSpeed = baseSpeed * (0.45 + 0.55 * curvatureEffect);

    % 确保最小速度
    baseSpeed = max(maxSpeed * 0.3, min(maxSpeed * 0.8, baseSpeed));
end

function [collisionDetected, avoidance] = detectFrontCollision(...
    state, obstacles, v, omega, vz, maxSpeed, maxOmega, maxVz, pheromoneInfo)

    % 初始化返回值
    collisionDetected = false;
    avoidance = struct('direction', [0,0,0], 'distance', inf, 'v', v, 'vz', vz, 'omega', omega, 'pitchRate', 0);

    currentPos = state(1:3);

    % 紧急避障检测
    detectionDistance = 7.0;
    global dynObsRadius;

    for i = 1:size(obstacles.dynamic, 1)
        obsPos = obstacles.dynamic(i, 1:3);
        obsRadius = obstacles.dynamic(i, 4);
        
        % 计算到障碍物中心的距离
        distToObs = norm(currentPos - obsPos);
        
        % 紧急避障阈值 - 距离小于障碍物半径+安全距离
        if distToObs < (obsRadius + detectionDistance)
            collisionDetected = true;
            
            % 计算远离方向
            avoidDir = currentPos - obsPos;
            if norm(avoidDir) > 0
                avoidDir = avoidDir / norm(avoidDir);
            else
                % 如果位置重合，默认向上避开
                avoidDir = [0, 0, 1];
            end
            
            % 添加随机扰动避免对称困境
            randDist = [rand()*0.2-0.1, rand()*0.2-0.1, rand()*0.4-0.1];
            avoidDir = avoidDir + randDist;
            if norm(avoidDir) > 0
                avoidDir = avoidDir / norm(avoidDir);
            end
            
            avoidance.distance = distToObs - obsRadius;
            avoidance.direction = avoidDir;
            
            % 计算紧急程度因子
            urgencyFactor = max(2.0, 10.0/((distToObs - obsRadius)+0.1));
            
            % 紧急转向
            yawError = atan2(avoidDir(2), avoidDir(1)) - state(4);
            yawError = wrapToPi(yawError);
            avoidance.omega = min(maxOmega*2.0, max(-maxOmega*2.0, yawError * 4.0 * urgencyFactor));
            
            % 垂直避障
            if avoidDir(3) > 0
                % 向上避障
                avoidance.vz = min(maxVz, max(0, maxVz * avoidDir(3) * urgencyFactor));
                avoidance.pitchRate = min(maxOmega, max(-maxOmega, 0.2 * urgencyFactor));
            elseif avoidDir(3) < 0
                % 向下避障
                avoidance.vz = max(-maxVz, min(0, -maxVz * abs(avoidDir(3)) * urgencyFactor));
                avoidance.pitchRate = max(-maxOmega, min(maxOmega, -0.2 * urgencyFactor));
            end
            
            % 紧急制动
            brakeIntensity = max(0.3, min(0.8, 1.2/((distToObs - obsRadius)+0.1)));
            avoidance.v = v * (1.0 - brakeIntensity);
            
            % 确保信息素抑制
            if distToObs < 6.0
                manager = pheromoneInfo.pheromoneManager;
obstaclePos = estimatedObsPos;
radius = estimatedRadius * 2.5;

centerIdx = worldToGrid(manager, obstaclePos);
radiusGrid = ceil(radius / manager.gridSize);

for dx = -radiusGrid:radiusGrid
    for dy = -radiusGrid:radiusGrid
        for dz = -radiusGrid:radiusGrid
            idx = centerIdx + [dx, dy, dz];
            if isValidGrid(manager, idx)
                dist = sqrt(dx^2 + dy^2 + dz^2) * manager.gridSize;
                if dist <= radius
                    factor = exp(-dist^2/(2*(radius/2.0)^2));
                    reduction = manager.maxPheromone * factor * 3.0;
                    
                    manager.avoidancePheromones(idx(1), idx(2), idx(3)) = ...
                        manager.avoidancePheromones(idx(1), idx(2), idx(3)) + reduction * 3.0;
                    
                    current = manager.pathPheromones(idx(1), idx(2), idx(3));
                    manager.pathPheromones(idx(1), idx(2), idx(3)) = ...
                        max(manager.initialPheromone * 0.1, current - reduction * 2.5);
                    
                    current = manager.pheromoneMatrix(idx(1), idx(2), idx(3));
                    manager.pheromoneMatrix(idx(1), idx(2), idx(3)) = ...
                        max(manager.initialPheromone * 0.1, current - reduction * 2.5);
                    
                    manager.shortTermPheromones(idx(1), idx(2), idx(3)) = ...
                        max(0, manager.shortTermPheromones(idx(1), idx(2), idx(3)) - reduction * 2.0);
                end
            end
        end
    end
end
updateIntegratedPheromones(manager);
            end
            
            % 检查每个方向是否会导致与静态障碍物碰撞
            % 首先检查原始避障方向是否会导致碰撞
            safeDir = ensureSafeAvoidanceDirection(currentPos, avoidDir, obstacles, 6);
            if ~isequal(safeDir, avoidDir)
                avoidance.direction = safeDir;
                
                % 更新避障控制
                yawError = atan2(safeDir(2), safeDir(1)) - state(4);
                yawError = wrapToPi(yawError);
                avoidance.omega = min(maxOmega*2.0, max(-maxOmega*2.0, yawError * 4.0 * urgencyFactor));
                
                if safeDir(3) > 0
                    avoidance.vz = min(maxVz, max(0, maxVz * safeDir(3) * urgencyFactor));
                    avoidance.pitchRate = min(maxOmega, max(-maxOmega, 0.2 * urgencyFactor));
                elseif safeDir(3) < 0
                    avoidance.vz = max(-maxVz, min(0, -maxVz * abs(safeDir(3)) * urgencyFactor));
                    avoidance.pitchRate = max(-maxOmega, min(maxOmega, -0.2 * urgencyFactor));
                end
            end
        end
    end

    % 检查静态障碍物前向碰撞
    staticCollision = checkStaticObstacleCollision(state, obstacles, v, maxSpeed);
    if staticCollision.detected && (~collisionDetected || staticCollision.distance < avoidance.distance)
        collisionDetected = true;
        avoidance.distance = staticCollision.distance;
        avoidance.direction = staticCollision.direction;
        
        % 计算紧急程度因子
        urgencyFactor = max(2.0, 10.0/(staticCollision.distance+0.1));
        
        % 紧急转向
        yawError = atan2(staticCollision.direction(2), staticCollision.direction(1)) - state(4);
        yawError = wrapToPi(yawError);
        avoidance.omega = min(maxOmega*2.0, max(-maxOmega*2.0, yawError * 4.0 * urgencyFactor));
        
        % 垂直避障
        if staticCollision.direction(3) > 0
            % 向上避障
            avoidance.vz = min(maxVz, max(0, maxVz * staticCollision.direction(3) * urgencyFactor));
            avoidance.pitchRate = min(maxOmega, max(-maxOmega, 0.2 * urgencyFactor));
        elseif staticCollision.direction(3) < 0
            % 向下避障
            avoidance.vz = max(-maxVz, min(0, -maxVz * abs(staticCollision.direction(3)) * urgencyFactor));
            avoidance.pitchRate = max(-maxOmega, min(maxOmega, -0.2 * urgencyFactor));
        end
        
        % 紧急制动
        brakeIntensity = max(0.3, min(0.8, 1.2/(staticCollision.distance+0.1)));
        avoidance.v = v * (1.0 - brakeIntensity);
    end
end

function safeDir = ensureSafeAvoidanceDirection(currentPos, initialDir, obstacles, checkDistance)
    % 确保避障方向不会导致与静态障碍物碰撞
    safeDir = initialDir;

    % 检查初始方向是否安全
    testPoint = currentPos + initialDir * checkDistance;
    if ~checkCollision(testPoint, obstacles)
        return; % 初始方向安全
    end

    % 生成候选方向
    candidateDirections = [
        [0, 0, 1],    % 上
        [0, 0, -1],   % 下
        [1, 0, 0],    % 右
        [-1, 0, 0],   % 左
        [0, 1, 0],    % 前
        [0, -1, 0],   % 后
        [1, 1, 0],    % 右前
        [1, -1, 0],   % 右后
        [-1, 1, 0],   % 左前
        [-1, -1, 0],  % 左后
        [1, 0, 1],    % 右上
        [-1, 0, 1],   % 左上
        [0, 1, 1],    % 前上
        [0, -1, 1],   % 后上
    ];

    % 找到最安全的方向
    bestScore = -inf;
    for i = 1:size(candidateDirections, 1)
        dir = candidateDirections(i, :);
        if norm(dir) > 0
            dir = dir / norm(dir);
        else
            continue;
        end
        
        % 计算与原始方向的相似度
        similarity = dot(dir, initialDir);
        
        % 检查安全性
        testPoint = currentPos + dir * checkDistance;
        if ~checkCollision(testPoint, obstacles)
            % 安全方向的得分
            score = similarity;
            
            if score > bestScore
                bestScore = score;
                safeDir = dir;
            end
        end
    end
end

function collision = checkStaticObstacleCollision(state, obstacles, v, maxSpeed)
    % 检查前方是否有静态障碍物

    % 初始化返回值
    collision = struct('detected', false, 'distance', inf, 'direction', [0,0,0]);

    % 获取当前位置和航向
    pos = state(1:3);
    yaw = state(4);
    pitch = state(5);

    % 前向检测距离
    lookAheadDist = 10.0 * (v / maxSpeed + 0.3);

    % 生成前向检测向量
    headingVec = [cos(yaw)*cos(pitch), sin(yaw)*cos(pitch), sin(pitch)];

    % 前向检测点
    testPoint = pos + headingVec * lookAheadDist;

    % 检查建筑物
    for i = 1:size(obstacles.buildings, 1)
        x = obstacles.buildings(i, 1);
        y = obstacles.buildings(i, 2);
        width = obstacles.buildings(i, 3);
        depth = obstacles.buildings(i, 4);
        height = obstacles.buildings(i, 5);

        % 简化的碰撞检测 - 检查直线是否与建筑物相交
        % 建筑物边界
        minX = x - width/2;
        maxX = x + width/2;
        minY = y - depth/2;
        maxY = y + depth/2;
        minZ = 0;
        maxZ = height;

        % 使用线段与AABB相交测试
        [intersect, t] = lineAABBIntersection(pos, testPoint, [minX, minY, minZ], [maxX, maxY, maxZ]);

        if intersect && t < 1
            distance = t * lookAheadDist;
            
            if distance < collision.distance
                collision.detected = true;
                collision.distance = distance;
                
                % 计算避障方向 - 远离建筑物
                avoidDir = pos - [x, y, pos(3)];
                if norm(avoidDir) > 0
                    avoidDir = avoidDir / norm(avoidDir);
                    
                    % 添加上升分量
                    avoidDir = avoidDir + [0, 0, 0.5];
                    avoidDir = avoidDir / norm(avoidDir);
                else
                    avoidDir = [0, 0, 1]; % 默认向上
                end
                
                collision.direction = avoidDir;
            end
        end
    end

    % 检查树木
    for i = 1:size(obstacles.trees, 1)
        x = obstacles.trees(i, 1);
        y = obstacles.trees(i, 2);
        radius = obstacles.trees(i, 3);
        height = obstacles.trees(i, 4);
        
        % 将树木简化为圆柱体
        % 计算点到圆柱体中心线的最短距离
        hDist = norm([pos(1)-x, pos(2)-y]); % 水平距离
        
        % 检查是否在树干内部
        trunkRadius = radius/3;
        trunkHeight = height * 0.6;
        
        if hDist <= trunkRadius && pos(3) <= trunkHeight
            collision.detected = true;
            distance = norm(pos - [x, y, pos(3)]);
            
            if distance < collision.distance
                collision.distance = distance;
                
                % 计算避障方向 - 远离树干
                avoidDir = [pos(1) - x, pos(2) - y, 0];
                if norm(avoidDir) > 0
                    avoidDir = avoidDir / norm(avoidDir);
                else
                    avoidDir = [1, 0, 0]; % 默认水平方向
                end
                
                collision.direction = avoidDir;
            end
        end
        
        % 检查是否在树冠内部
        treeTop = trunkHeight + radius;
        if hDist <= radius && pos(3) > trunkHeight && pos(3) <= treeTop
            collision.detected = true;
            distance = norm(pos - [x, y, pos(3)]);
            
            if distance < collision.distance
                collision.distance = distance;
                
                % 计算避障方向 - 远离树冠或从上方绕过
                if pos(3) > treeTop * 0.8
                    collision.direction = [0, 0, 1]; % 向上绕过
                else
                    avoidDir = [pos(1) - x, pos(2) - y, 0];
                    if norm(avoidDir) > 0
                        collision.direction = avoidDir / norm(avoidDir);
                    else
                        collision.direction = [1, 0, 0];
                    end
                end
            end
        end
    end
end

function [isBlocking, obstacleInfo] = predictDynamicObstacles(state, obstacles, lookAheadDist)
    % 专门针对动态障碍物的障碍物预测函数
    global enableDynamicObstacles;
    global originalPath;
    global debugMode;

    % 默认结果
    isBlocking = false;
    obstacleInfo = struct('distance', inf, 'position', [0,0,0], 'height', 0, 'avoidDirection', [0,0,0]);

    % 禁用动态障碍物避障时退出
    if ~enableDynamicObstacles
        return;
    end

    % 安全检查: 确保originalPath有效
    if isempty(originalPath) || ~isnumeric(originalPath) || size(originalPath, 2) < 3
        return;
    end

    % 当前位置
    pos = state(1:3);

    % 找到最近的路径点索引
    try
        [~, currentIdx] = findClosestPointOnPath(pos, originalPath, 1);
    catch
        currentIdx = 1;
    end

    % 创建严格沿A*路径的预测点
    predictSteps = 30;
    predictPoints = zeros(predictSteps, 3);

    % 生成沿路径的预测点（精确沿路径）
    for i = 1:predictSteps
        % 计算路径参数t (当前位置 + 预测距离)
        t = currentIdx + (i-1) * 0.5; % 以0.5为步长沿路径移动
        
        % 确保t在有效范围内
        if t < 1
            t = 1;
        end
        
        % 确保t不超出路径长度
        if t >= size(originalPath, 1)
            t = size(originalPath, 1);
        end
        
        % 插值获取精确的路径点
        predictPoints(i,:) = getPointOnPath(t, originalPath);
    end

    % 检查动态障碍物
    minDist = inf;
    closestObsIdx = 0;

    % 检查障碍物
    if ~isfield(obstacles, 'dynamic') || isempty(obstacles.dynamic)
        return;
    end

    % 检查所有动态障碍物
    for i = 1:size(obstacles.dynamic, 1)
        % 确保obstacles.dynamic不为空且行数足够
        if isempty(obstacles.dynamic) || i > size(obstacles.dynamic, 1)
            continue;
        end
        
        % 提取障碍物属性
        if size(obstacles.dynamic, 2) >= 4
            obsPos = obstacles.dynamic(i, 1:3);
            obsRadius = obstacles.dynamic(i, 4);
        else
            % 如果数据不足，使用默认值
            continue;
        end
        
        % 计算到障碍物中心的向量
        toObs = obsPos - pos;
        distToObs = norm(toObs);
        
        % 只考虑前方一定距离内的障碍物
        if distToObs <= lookAheadDist
            % 检查预测路径是否会与障碍物碰撞
            willCollide = false;
            minPathObsDist = inf;
            
            for j = 1:predictSteps
                % 跳过无效的预测点
                if j > size(predictPoints, 1) || all(predictPoints(j,:) == 0)
                    continue;
                end
                
                % 计算预测点到障碍物中心的距离
                predPos = predictPoints(j,:);
                distPredToObs = norm(predPos - obsPos);
                
                % 如果距离小于障碍物半径+安全距离，认为会碰撞
                if distPredToObs < (obsRadius + 3.0)
                    willCollide = true;
                    minPathObsDist = 0;
                    break;
                end
                
                % 更新最小距离
                if distPredToObs < minPathObsDist
                    minPathObsDist = distPredToObs - obsRadius;
                    
                    % 如果距离小于安全阈值，认为可能会碰撞
                    if minPathObsDist < 3.0
                        willCollide = true;
                        break;
                    end
                end
            end
            
            if willCollide || distToObs < minDist
                minDist = distToObs;
                closestObsIdx = i;
                
                % 计算最佳避障方向
                % 计算相对方向 - 使用路径方向为参考
                pathDir = [];
                if currentIdx < size(originalPath, 1) - 1
                    pathDir = originalPath(currentIdx+1,:) - originalPath(currentIdx,:);
                    if norm(pathDir) > 0
                        pathDir = pathDir / norm(pathDir);
                    else
                        pathDir = [0, 0, 0];
                    end
                else
                    pathDir = [0, 0, 0];
                end
                
                rightVec = [];
                if norm(pathDir) > 0
                    rightVec = cross(pathDir, [0, 0, 1]); % 右向量
                    if norm(rightVec) > 0
                        rightVec = rightVec / norm(rightVec);
                    else
                        rightVec = [1, 0, 0]; % 默认右向量
                    end
                else
                    rightVec = [1, 0, 0]; % 默认右向量
                end
                
                % 确定绕行方向 - 向左还是向右
                leftDist = norm(obsPos - obsRadius - pos);
                rightDist = norm(obsPos + obsRadius - pos);
                
                % 选择远离障碍物的方向绕行
                if leftDist > rightDist
                    horizontalDir = [-rightVec(1), -rightVec(2), 0]; % 向左绕
                else
                    horizontalDir = [rightVec(1), rightVec(2), 0]; % 向右绕
                end
                
                % 垂直避障方向 - 根据相对高度判断向上或向下
                if pos(3) < obsPos(3)
                    verticalDir = [0, 0, -1]; % 向下绕行
                else
                    verticalDir = [0, 0, 1]; % 向上绕行
                end
                
                % 根据障碍物的形状和相对位置，动态选择避障方向
                avoidDir = [];
                if abs(pos(3) - obsPos(3)) < obsRadius
                    % 如果高度重叠较大，优先水平避障
                    avoidDir = horizontalDir * 0.8 + verticalDir * 0.2;
                else
                    % 优先垂直避障
                    avoidDir = horizontalDir * 0.2 + verticalDir * 0.8;
                end
                
                % 添加一个小的前进分量
                if norm(pathDir) > 0
                    avoidDir = avoidDir + pathDir * 0.1;
                end
                
                % 添加小随机扰动以打破对称性
                randDisturb = [rand()*0.05-0.025, rand()*0.05-0.025, rand()*0.05-0.025];
                avoidDir = avoidDir + randDisturb;
                
                % 归一化避障方向
                if norm(avoidDir) > 0
                    avoidDir = avoidDir / norm(avoidDir);
                else
                    % 防止零向量
                    avoidDir = [0, 0, 1];
                end
                
                % 设置障碍物信息
                obstacleInfo.distance = minDist - obsRadius;
                obstacleInfo.position = obsPos;
                obstacleInfo.height = obsRadius;
                obstacleInfo.avoidDirection = avoidDir;
                
                if willCollide
                    isBlocking = true;
                    
                    % 调试输出
                    if debugMode
                        disp(['检测到障碍物将阻挡路径，距离: ', num2str(minDist - obsRadius), ' 避障方向: [', ...
                            num2str(avoidDir(1), '%.2f'), ', ', ...
                            num2str(avoidDir(2), '%.2f'), ', ', ...
                            num2str(avoidDir(3), '%.2f'), ']']);
                    end
                    
                    return; % 发现碰撞立即返回
                end
            end
        end
    end

    % 如果找到了最近障碍物但不会立即碰撞
    if closestObsIdx > 0
        isBlocking = minDist < 15.0;
    end
end

function point = getPointOnPath(t, path)
    % 获取路径上参数t对应的点（支持非整数t）
    if t <= 1
        point = path(1,:);
        return;
    end

    if t >= size(path, 1)
        point = path(end,:);
        return;
    end

    % 将t分解为整数部分和小数部分
    idx = floor(t);
    frac = t - idx;

    % 线性插值
    if idx < size(path, 1)
        point = path(idx,:) * (1-frac) + path(idx+1,:) * frac;
    else
        point = path(end,:);
    end
end

%% 碰撞检测和路径工具
function isCollision = checkCollision(position, obstacles)
    % 检查位置是否与障碍物碰撞
    global collisionSafetyMargin;
    global allowPathCollisions;
    global flightPath;

    % 默认安全边距
    if isempty(collisionSafetyMargin) || ~isnumeric(collisionSafetyMargin) || collisionSafetyMargin <= 0
        safetyMargin = 1.5; % 减小碰撞检测安全边距
    else
        safetyMargin = collisionSafetyMargin;
    end

    isCollision = false;

    % 如果允许路径上的碰撞检测，检查该位置是否接近路径
    if allowPathCollisions && ~isempty(flightPath)
        [minPathDist, ~] = findClosestPointOnPath(position, flightPath, 1);
        if minPathDist < 1.0
            % 位置接近规划路径，不视为碰撞
            isCollision = false;
            return;
        end
    end

    % 检查建筑物
    for i = 1:size(obstacles.buildings, 1)
        x = obstacles.buildings(i, 1);
        y = obstacles.buildings(i, 2);
        width = obstacles.buildings(i, 3) * safetyMargin;
        depth = obstacles.buildings(i, 4) * safetyMargin;
        height = obstacles.buildings(i, 5) * safetyMargin;
        
        % 检查点是否在建筑物内部
        if position(1) >= x-width/2 && position(1) <= x+width/2 && ...
           position(2) >= y-depth/2 && position(2) <= y+depth/2 && ...
           position(3) >= 0 && position(3) <= height
            isCollision = true;
            return;
        end
    end

    % 检查树木
    for i = 1:size(obstacles.trees, 1)
        x = obstacles.trees(i, 1);
        y = obstacles.trees(i, 2);
        radius = obstacles.trees(i, 3) * safetyMargin;
        height = obstacles.trees(i, 4) * safetyMargin;
        
        % 检查树干
        trunkRadius = radius/3;
        trunkHeight = height * 0.6;
        
        % 计算水平距离
        hDist = sqrt((position(1) - x)^2 + (position(2) - y)^2);
        
        % 检查是否在树干内部
        if hDist <= trunkRadius && position(3) <= trunkHeight
            isCollision = true;
            return;
        end
        
        % 检查是否在树冠内部
        treeTop = trunkHeight + radius;
        if hDist <= radius && position(3) > trunkHeight && position(3) <= treeTop
            isCollision = true;
            return;
        end
    end

    % 检查电线杆和电线
    for i = 1:size(obstacles.powerLines, 1)
        x1 = obstacles.powerLines(i, 1);
        y1 = obstacles.powerLines(i, 2);
        x2 = obstacles.powerLines(i, 3);
        y2 = obstacles.powerLines(i, 4);
        height = obstacles.powerLines(i, 5);
        
        if position(3) <= height+0.5 && position(3) >= height-1.5
            % 计算点到线段的距离
            v = [x2-x1, y2-y1, 0];
            w = [position(1)-x1, position(2)-y1, 0];
            
            c1 = dot(w,v);
            c2 = dot(v,v);
            
            if c1 <= 0
                dist = norm([position(1)-x1, position(2)-y1]);
            elseif c2 <= c1
                dist = norm([position(1)-x2, position(2)-y2]);
            else
                b = c1/c2;
                pb = [x1, y1, 0] + b * v;
                dist = norm([position(1)-pb(1), position(2)-pb(2), 0]);
            end
            
            % 检查是否在电线附近
            if dist <= 1.0 * safetyMargin
                isCollision = true;
                return;
            end
        end
    end

    % 检查动态障碍物
    for i = 1:size(obstacles.dynamic, 1)
        obsPos = obstacles.dynamic(i, 1:3);
        obsRadius = obstacles.dynamic(i, 4) * safetyMargin;
        
        % 计算到障碍物中心的距离
        dist = norm(position - obsPos);
        
        % 检查是否在障碍物内部
        if dist <= obsRadius
            isCollision = true;
            return;
        end
    end
end

function collision = checkPathCollision(start, endpoint, obstacles)
    % 检查两点之间的路径是否有碰撞
    collision = false;
    global trajectoryCheckDivisions;

    % 确保分段数有效
    if isempty(trajectoryCheckDivisions) || ~isnumeric(trajectoryCheckDivisions) || trajectoryCheckDivisions < 2
        steps = 6; % 降低分段数以减少误报
    else
        steps = trajectoryCheckDivisions;
    end

    % 增加路径插值点数，提高检测精度
    for i = 1:steps
        t = i / steps;
        point = start + t * (endpoint - start);
        
        if checkCollision(point, obstacles)
            collision = true;
            return;
        end
    end
end

function [closestIdx, minDist, lookAheadIdx] = enhancedPathFollowing(pos, flightPath, droneState)
    % 严格路径跟踪 - 优化前瞻点选择策略，更精确跟踪路径
    % 初始化返回值
    minDist = inf;
    closestIdx = 1;
    lookAheadIdx = 1;

    % 安全检查
    if isempty(flightPath) || size(flightPath, 1) < 1
        return;
    end

    % 精确寻找最近的路径点和投影点
    [minDist, projIdx] = findClosestPointOnPath(pos, flightPath, 1);
    closestIdx = projIdx;

    % 使用更短的前瞻距离，确保精确跟踪
    lookAheadDist = 2.0;

    % 安全检查: 确保droneState有效
    if isempty(droneState) || length(droneState) < 7
        currentVel = 1.5; % 默认速度
    else
        currentVel = sqrt(droneState(7)^2 + droneState(8)^2); % 当前速度
    end

    % 根据速度调整前瞻距离 - 更保守的调整
    adaptiveLookAhead = max(1.0, min(2.0, lookAheadDist * (0.5 + currentVel/8)));

    lookAheadIdx = closestIdx;
    accumulatedDist = 0;

    for i = closestIdx:size(flightPath, 1)-1
        segmentDist = norm(flightPath(i+1,:) - flightPath(i,:));
        accumulatedDist = accumulatedDist + segmentDist;
        
        if accumulatedDist >= adaptiveLookAhead || i == size(flightPath, 1)-1
            lookAheadIdx = i + 1;
            break;
        end
    end

    % 安全检查：确保索引不会超出范围
    lookAheadIdx = min(lookAheadIdx, size(flightPath, 1));
    closestIdx = min(closestIdx, size(flightPath, 1));
end

function [closestIdx, minDist, lookAheadIdx] = standardPathFollowing(pos, flightPath, droneState)
    % 标准路径跟踪 - 优化前瞻点选择策略
    % 初始化返回值
    minDist = inf;
    closestIdx = 1;
    lookAheadIdx = 1;

    % 安全检查
    if isempty(flightPath) || size(flightPath, 1) < 1
        return;
    end

    % 寻找最近的路径点
    for i = 1:size(flightPath, 1)
        dist = norm(flightPath(i,:) - pos);
        if dist < minDist
            minDist = dist;
            closestIdx = i;
        end
    end

    % 确定前瞻点 - 更短的前瞻距离确保紧密跟踪
    lookAheadDist = 2.5;

    % 安全检查: 确保droneState有效
    if isempty(droneState) || length(droneState) < 7
        currentVel = 2.0; % 默认速度
    else
        currentVel = sqrt(droneState(7)^2 + droneState(8)^2); % 当前速度
    end

    % 根据速度调整前瞻距离
    adaptiveLookAhead = max(1.5, min(3.0, lookAheadDist * (0.6 + currentVel/6)));

    lookAheadIdx = closestIdx;
    accumulatedDist = 0;

    for i = closestIdx:size(flightPath, 1)-1
        segmentDist = norm(flightPath(i+1,:) - flightPath(i,:));
        accumulatedDist = accumulatedDist + segmentDist;
        
        if accumulatedDist >= adaptiveLookAhead || i == size(flightPath, 1)-1
            lookAheadIdx = i + 1;
            break;
        end
    end

    % 安全检查：确保索引不会超出范围
    lookAheadIdx = min(lookAheadIdx, size(flightPath, 1));
    closestIdx = min(closestIdx, size(flightPath, 1));
end

function [distance, projIdx] = findClosestPointOnPath(pos, path, startIdx)
    % 找到当前位置在路径上的投影点
    distance = inf;
    projIdx = 1;

    % 安全检查：确保startIdx有效
    if isempty(startIdx) || ~isnumeric(startIdx) || ~isscalar(startIdx) || isnan(startIdx) || startIdx < 1
        startIdx = 1;
    end

    % 如果路径为空或只有一个点，直接返回
    if isempty(path) || size(path, 1) < 2
        if isempty(path)
            projIdx = 1;
            distance = inf;
        else
            projIdx = 1;
            distance = norm(pos - path(1,:));
        end
        return;
    end

    % 确保startIdx在有效范围内
    startIdx = max(1, min(startIdx, size(path, 1)));

    for i = startIdx:size(path,1)-1
        % 计算线段
        p1 = path(i,:);
        p2 = path(i+1,:);
        segment = p2 - p1;
        segmentLength = norm(segment);
        
        if segmentLength < 0.001
            continue; % 跳过极短线段
        end
        
        % 单位向量
        segmentDir = segment / segmentLength;
        
        % 计算投影
        v = pos - p1;
        proj = dot(v, segmentDir);
        
        % 计算最近点
        if proj < 0
            closestPoint = p1;
        elseif proj > segmentLength
            closestPoint = p2;
        else
            closestPoint = p1 + proj * segmentDir;
        end
        
        % 计算距离
        dist = norm(pos - closestPoint);
        
        if dist < distance
            distance = dist;
            projIdx = i;
            
            % 如果投影在线段结束点附近，选择下一个点
            if proj > segmentLength * 0.8
                projIdx = min(i + 1, size(path, 1));
            end
        end
    end
end

function newState = updateDroneState3D(state, v, vz, omega, pitchRate, dt)
    % 更新无人机状态 
    global mapSize;

    % 当前状态
    x = state(1);
    y = state(2);
    z = state(3);
    yaw = state(4);
    pitch = state(5);
    roll = state(6);

    % 对运动指令添加平滑处理
    vFiltered = v;
    vzFiltered = vz;
    omegaFiltered = omega;
    pitchRateFiltered = pitchRate;

    % 更新姿态角度
    newYaw = yaw + omegaFiltered * dt;
    newPitch = pitch + pitchRateFiltered * dt;
    newRoll = roll; % 不考虑滚转

    % 限制俯仰角范围
    newPitch = max(-pi/3, min(pi/3, newPitch));

    % 考虑俯仰角影响水平和垂直运动
    newX = x + vFiltered * cos(newYaw) * cos(newPitch) * dt;
    newY = y + vFiltered * sin(newYaw) * cos(newPitch) * dt;
    newZ = z + vzFiltered * dt + vFiltered * sin(newPitch) * dt; % 考虑俯仰对高度的影响

    % 计算当前速度分量
    vx = vFiltered * cos(newYaw) * cos(newPitch);
    vy = vFiltered * sin(newYaw) * cos(newPitch);
    vz = vzFiltered + vFiltered * sin(newPitch);

    % 地图边界安全裕度
    margin = 1.5;

    % 检查并限制位置在地图范围内
    if newX < margin
        newX = margin;
        vx = max(0, vx);  % 防止继续向边界外移动
    elseif newX > mapSize(1) - margin
        newX = mapSize(1) - margin;
        vx = min(0, vx);
    end

    if newY < margin
        newY = margin;
        vy = max(0, vy);
    elseif newY > mapSize(2) - margin
        newY = mapSize(2) - margin;
        vy = min(0, vy);
    end

    if newZ < margin
        newZ = margin;
        vz = max(0, vz);
    elseif newZ > mapSize(3) - margin
        newZ = mapSize(3) - margin;
        vz = min(0, vz);
    end

    % 返回新状态
    newState = [newX, newY, newZ, newYaw, newPitch, newRoll, vx, vy, vz];
end

function energyConsumption = calculateEnergyConsumption(v, vz, omega, baseRate, speedFactor, climbFactor, turnFactor, dt)
    % 能量消耗计算函数
    % 基础能耗
    baseConsumption = baseRate * dt;

    % 速度消耗（二次关系）
    speedConsumption = speedFactor * v^2 * dt;

    % 垂直速度消耗（爬升消耗更大）
    if vz > 0
        climbConsumption = climbFactor * vz^2 * dt;
    else
        climbConsumption = climbFactor * 0.3 * abs(vz) * dt;
    end

    % 转向消耗
    turnConsumption = turnFactor * abs(omega) * dt;

    % 总能耗 - 确保至少有最小能耗
    minConsumption = baseRate * 0.5 * dt;
    energyConsumption = max(minConsumption, baseConsumption + speedConsumption + climbConsumption + turnConsumption);
end

%% 线段与AABB相交测试
function [intersect, t] = lineAABBIntersection(lineStart, lineEnd, boxMin, boxMax)
    % 线段与轴对齐包围盒(AABB)相交测试
    intersect = false;
    t = inf;
    
    % 线段方向
    dir = lineEnd - lineStart;
    
    % 参数化t的范围
    tmin = 0;
    tmax = 1;
    
    % 测试x轴方向
    if abs(dir(1)) < 1e-6
        % 线段平行于x轴
        if lineStart(1) < boxMin(1) || lineStart(1) > boxMax(1)
            % 线段在盒子外部
            return;
        end
    else
        % 计算t的范围
        tx1 = (boxMin(1) - lineStart(1)) / dir(1);
        tx2 = (boxMax(1) - lineStart(1)) / dir(1);
        
        % 确保tx1 <= tx2
        if tx1 > tx2
            temp = tx1;
            tx1 = tx2;
            tx2 = temp;
        end
        
        % 更新t范围
        tmin = max(tmin, tx1);
        tmax = min(tmax, tx2);
        
        % 如果t范围无效，则无交点
        if tmin > tmax
            return;
        end
    end
    
    % 测试y轴方向
    if abs(dir(2)) < 1e-6
        % 线段平行于y轴
        if lineStart(2) < boxMin(2) || lineStart(2) > boxMax(2)
            % 线段在盒子外部
            return;
        end
    else
        % 计算t的范围
        ty1 = (boxMin(2) - lineStart(2)) / dir(2);
        ty2 = (boxMax(2) - lineStart(2)) / dir(2);
        
        % 确保ty1 <= ty2
        if ty1 > ty2
            temp = ty1;
            ty1 = ty2;
            ty2 = temp;
        end
        
        % 更新t范围
        tmin = max(tmin, ty1);
        tmax = min(tmax, ty2);
        
        % 如果t范围无效，则无交点
        if tmin > tmax
            return;
        end
    end
    
    % 测试z轴方向
    if abs(dir(3)) < 1e-6
        % 线段平行于z轴
        if lineStart(3) < boxMin(3) || lineStart(3) > boxMax(3)
            % 线段在盒子外部
            return;
        end
    else
        % 计算t的范围
        tz1 = (boxMin(3) - lineStart(3)) / dir(3);
        tz2 = (boxMax(3) - lineStart(3)) / dir(3);
        
        % 确保tz1 <= tz2
        if tz1 > tz2
            temp = tz1;
            tz1 = tz2;
            tz2 = temp;
        end
        
        % 更新t范围
        tmin = max(tmin, tz1);
        tmax = min(tmax, tz2);
        
        % 如果t范围无效，则无交点
        if tmin > tmax
            return;
        end
    end
    
    % 如果到这里，有交点
    intersect = true;
    t = tmin; % 返回最早的交点参数
end

%% 优先队列实现
function queue = createPriorityQueue()
    % 创建优先队列
    queue = struct();
    queue.keys = {};
    queue.priorities = [];
    queue.map = containers.Map('KeyType', 'char', 'ValueType', 'double');
    queue.size = 0;
end

function queue = queueInsert(queue, key, priority)
    % 向队列中插入元素
    if queueContains(queue, key)
        queue = queueUpdatePriority(queue, key, priority);
        return;
    end

    queue.size = queue.size + 1;
    queue.keys{queue.size} = key;
    queue.priorities(queue.size) = priority;
    queue.map(key) = queue.size;

    % 执行上浮操作
    queue = queueBubbleUp(queue, queue.size);
end

function [queue, key] = queuePop(queue)
    % 弹出最高优先级元素
    key = '';
    if queueIsEmpty(queue)
        return;
    end

    key = queue.keys{1};
    remove(queue.map, key);

    if queue.size > 1
        queue.keys{1} = queue.keys{queue.size};
        queue.priorities(1) = queue.priorities(queue.size);
        queue.map(queue.keys{1}) = 1;
    end

    queue.keys(queue.size) = [];
    queue.priorities(queue.size) = [];
    queue.size = queue.size - 1;

    if queue.size > 0
        queue = queueBubbleDown(queue, 1);
    end
end
function pheromoneInfo = analyzePheromoneAroundPoint(pheromoneManager, worldPos, radius, gridSize)
    % 分析指定世界坐标点周围的信息素浓度
    % 
    % 输入:
    %   pheromoneManager - 信息素管理器
    %   worldPos - 世界坐标 [x, y, z]
    %   radius - 检测半径（网格单位）
    %   gridSize - 网格大小
    % 输出:
    %   pheromoneInfo - 包含信息素统计信息的结构体
    
    % 转换为网格坐标
    centerGridIdx = ceil(worldPos / gridSize);
    dims = size(pheromoneManager.pheromoneMatrix);
    
    % 初始化输出
    pheromoneInfo = struct();
    pheromoneInfo.centerPos = worldPos;
    pheromoneInfo.centerGridIdx = centerGridIdx;
    pheromoneInfo.radius = radius;
    pheromoneInfo.values = [];
    pheromoneInfo.positions = [];
    
    % 检查中心点是否有效
    if ~all(centerGridIdx > 0) || ~all(centerGridIdx <= dims)
        pheromoneInfo.valid = false;
        pheromoneInfo.message = '中心点超出地图范围';
        return;
    end
    
    pheromoneInfo.valid = true;
    pheromoneInfo.centerValue = pheromoneManager.pheromoneMatrix(centerGridIdx(1), centerGridIdx(2), centerGridIdx(3));
    
    % 收集周围区域的信息素值
    values = [];
    positions = [];
    distances = [];
    
    for i = -radius:radius
        for j = -radius:radius
            for k = -radius:radius
                idx = centerGridIdx + [i, j, k];
                
                % 检查索引有效性
                if all(idx > 0) && all(idx <= dims)
                    % 计算距离
                    distance = sqrt(i^2 + j^2 + k^2);
                    
                    if distance <= radius
                        % 获取信息素值
                        phValue = pheromoneManager.pheromoneMatrix(idx(1), idx(2), idx(3));
                        values = [values; phValue];
                        positions = [positions; idx];
                        distances = [distances; distance];
                    end
                end
            end
        end
    end
    
    % 统计信息
    pheromoneInfo.values = values;
    pheromoneInfo.positions = positions;
    pheromoneInfo.distances = distances;
    pheromoneInfo.count = length(values);
    
    if ~isempty(values)
        pheromoneInfo.max = max(values);
        pheromoneInfo.min = min(values);
        pheromoneInfo.mean = mean(values);
        pheromoneInfo.median = median(values);
        pheromoneInfo.std = std(values);
        
        % 按距离分层统计
        pheromoneInfo.layer1 = mean(values(distances <= 1));  % 最近层
        if radius > 1
            pheromoneInfo.layer2 = mean(values(distances > 1 & distances <= 2));
        end
        if radius > 2
            pheromoneInfo.layer3 = mean(values(distances > 2));
        end
        
        % 各层信息素
        pheromoneInfo.pathLayer = [];
        pheromoneInfo.avoidanceLayer = [];
        pheromoneInfo.energyLayer = [];
        
        for m = 1:size(positions, 1)
            idx = positions(m, :);
            pheromoneInfo.pathLayer = [pheromoneInfo.pathLayer; pheromoneManager.pathPheromones(idx(1), idx(2), idx(3))];
            pheromoneInfo.avoidanceLayer = [pheromoneInfo.avoidanceLayer; pheromoneManager.avoidancePheromones(idx(1), idx(2), idx(3))];
            pheromoneInfo.energyLayer = [pheromoneInfo.energyLayer; pheromoneManager.energyPheromones(idx(1), idx(2), idx(3))];
        end
        
        pheromoneInfo.pathLayerMean = mean(pheromoneInfo.pathLayer);
        pheromoneInfo.avoidanceLayerMean = mean(pheromoneInfo.avoidanceLayer);
        pheromoneInfo.energyLayerMean = mean(pheromoneInfo.energyLayer);
    else
        pheromoneInfo.message = '未找到有效数据';
    end
end
function queue = queueUpdatePriority(queue, key, priority)
    % 更新优先级
    if ~queueContains(queue, key)
        queue = queueInsert(queue, key, priority);
        return;
    end

    index = queue.map(key);
    oldPriority = queue.priorities(index);
    queue.priorities(index) = priority;

    if priority < oldPriority
        queue = queueBubbleUp(queue, index);
    else
        queue = queueBubbleDown(queue, index);
    end
end

function contains = queueContains(queue, key)
    % 检查队列是否包含元素
    contains = isKey(queue.map, key);
end

function empty = queueIsEmpty(queue)
    % 检查队列是否为空
    empty = (queue.size == 0);
end

function queue = queueBubbleUp(queue, index)
    % 上浮操作
    while index > 1
        parentIndex = floor(index / 2);
        if queue.priorities(parentIndex) <= queue.priorities(index)
            break;
        end
        
        % 交换元素
        queue = queueSwap(queue, parentIndex, index);
        index = parentIndex;
    end
end

function queue = queueBubbleDown(queue, index)
    % 下沉操作
    while true
        smallest = index;
        leftChild = 2 * index;
        rightChild = 2 * index + 1;
        
        if leftChild <= queue.size && queue.priorities(leftChild) < queue.priorities(smallest)
            smallest = leftChild;
        end
        
        if rightChild <= queue.size && queue.priorities(rightChild) < queue.priorities(smallest)
            smallest = rightChild;
        end
        
        if smallest == index
            break;
        end
        
        % 交换元素
        queue = queueSwap(queue, index, smallest);
        index = smallest;
    end
end

function queue = queueSwap(queue, i, j)
    % 交换队列中的两个元素
    % 交换键
    tempKey = queue.keys{i};
    queue.keys{i} = queue.keys{j};
    queue.keys{j} = tempKey;

    % 交换优先级
    tempPriority = queue.priorities(i);
    queue.priorities(i) = queue.priorities(j);
    queue.priorities(j) = tempPriority;

    % 更新映射
    queue.map(queue.keys{i}) = i;
    queue.map(queue.keys{j}) = j;
end

%% 可视化和UI函数
function plotBuilding(position, width, depth, height, color, alpha)
    % 绘制建筑物
    if nargin < 6
        alpha = 0.7;
    end

    % 顶点坐标
    vertices = [
        0, 0, 0;
        width, 0, 0;
        width, depth, 0;
        0, depth, 0;
        0, 0, height;
        width, 0, height;
        width, depth, height;
        0, depth, height
    ];

    % 移动到指定位置
    vertices = vertices + repmat(position, 8, 1);

    % 定义面
    faces = [
        1, 2, 6, 5;    % 前面
        2, 3, 7, 6;    % 右面
        3, 4, 8, 7;    % 后面
        4, 1, 5, 8;    % 左面
        5, 6, 7, 8;    % 顶面
        1, 2, 3, 4     % 底面
    ];

    % 绘制建筑物
    patch('Vertices', vertices, 'Faces', faces, ...
          'FaceColor', color, 'EdgeColor', 'k', 'FaceAlpha', alpha);
end

function handle = plotCube(position, width, depth, height, color, alpha, tag)
    % 绘制立方体函数
    if nargin < 6
        alpha = 0.7;
    end
    if nargin < 7
        tag = '';
    end

    % 确保颜色值在有效范围内
    if ~isempty(color) && isnumeric(color) && length(color) == 3
        % 确保RGB值在0-1范围内
        color = max(0, min(1, color));
    else
        % 默认颜色
        color = [0.7, 0.7, 0.7];
    end

    % 顶点坐标
    vertices = [
        0, 0, 0;
        width, 0, 0;
        width, depth, 0;
        0, depth, 0;
        0, 0, height;
        width, 0, height;
        width, depth, height;
        0, depth, height
    ];

    % 移动到指定位置
    vertices = vertices + repmat(position, 8, 1);

    % 定义面
    faces = [
        1, 2, 6, 5;    % 前面
        2, 3, 7, 6;    % 右面
        3, 4, 8, 7;    % 后面
        4, 1, 5, 8;    % 左面
        5, 6, 7, 8;    % 顶面
        1, 2, 3, 4     % 底面
    ];

    % 绘制立方体
    handle = patch('Vertices', vertices, 'Faces', faces, ...
          'FaceColor', color, 'EdgeColor', 'k', 'FaceAlpha', alpha, 'Tag', tag);
end

function closeCallback(src, ~)
    % 窗口关闭回调
    % 默认关闭行为
    delete(src);
end

function pauseButtonCallback(~, ~)
    % 暂停按钮回调
    global isPaused pauseTextHandle;
    isPaused = ~isPaused;

    if isPaused
        disp('仿真已暂停');
        if ishandle(pauseTextHandle)
            set(pauseTextHandle, 'Visible', 'on');
        end
    else
        disp('仿真已继续');
        if ishandle(pauseTextHandle)
            set(pauseTextHandle, 'Visible', 'off');
        end
    end
end

function replanPathCallback(~, ~)
    % 路径重规划按钮回调
    global droneState goal flightPath originalPath pathSegments currentGoalIdx;
    global flightFig planningFig simulationTime isReplanning replanTextHandle;
    global lastReplanTime pgdwaStar pathAdvancementStuckCounter currentTargetHandle;
    global planPathHandle; % 新增：规划路径句柄

    % 防止重复触发
    if simulationTime - lastReplanTime < 2.0
        disp('请等待几秒后再次尝试重规划');
        return;
    end

    % 显示重规划状态
    isReplanning = true;
    if ishandle(replanTextHandle)
        set(replanTextHandle, 'Visible', 'on');
    end

    % 触发路径重规划
    disp('手动触发路径重规划...');

    % 创建新路径
    newStart = droneState(1:3);
    newPath = planPath(pgdwaStar, newStart, goal);

    % 如果找到有效路径，应用它
    if ~isempty(newPath) && size(newPath, 1) > 1
        % 更新路径
        flightPath = newPath;
        originalPath = newPath;
        pathSegments = calculatePathSegmentInfo(flightPath);
        currentGoalIdx = 2; % 重置为新路径的第二个点
        pathAdvancementStuckCounter = 0; % 重置卡住计数器
        
        % 更新飞行图中的路径显示
        if ishandle(flightFig)
            figure(flightFig);
            % 删除旧路径显示
            if ishandle(planPathHandle)
                delete(planPathHandle);
            end
            % 绘制新路径
            planPathHandle = plot3(flightPath(:,1), flightPath(:,2), flightPath(:,3), 'b-', 'LineWidth', 2);
        end
        
        % 更新规划图上的路径
        if ishandle(planningFig)
            figure(planningFig);
            oldPathHandles = findobj(planningFig, 'Type', 'line', 'Color', 'b');
            delete(oldPathHandles);
            plot3(flightPath(:,1), flightPath(:,2), flightPath(:,3), 'b-', 'LineWidth', 2);
        end
        
        % 更新当前目标点
        if ishandle(currentTargetHandle)
            delete(currentTargetHandle);
        end
        
        if ishandle(flightFig)
            figure(flightFig);
            currentTargetHandle = plot3(flightPath(currentGoalIdx,1), flightPath(currentGoalIdx,2), flightPath(currentGoalIdx,3), ...
                                    'mo', 'MarkerSize', 6, 'MarkerFaceColor', 'm');
        end
        
        disp(['路径重规划完成，新路径包含 ', num2str(size(flightPath, 1)), ' 个点']);
        lastReplanTime = simulationTime;
    else
        disp('路径重规划失败，继续使用当前路径');
    end

    isReplanning = false;
    if ishandle(replanTextHandle)
        set(replanTextHandle, 'Visible', 'off');
    end
    if ishandle(flightFig)
        figure(flightFig); % 确保返回飞行图
    end
end

function obstacleInterferenceCallback(~, ~)
    % 障碍物干扰按钮回调 - 优化预测与放置逻辑
    global droneState globalObstacles dynObsHandles flightPath flightFig planningFig;
    global planning_dynObsHandles pheromoneManager pheromoneFig mapSize;
    global dynObsRadius;

    % 确保路径存在
    if ~isempty(flightPath) && size(flightPath, 1) > 5
        % 计算无人机当前速度和方向
        currentSpeed = sqrt(droneState(7)^2 + droneState(8)^2);
        yaw = droneState(4);
        directionVec = [cos(yaw), sin(yaw), 0];
        
        if currentSpeed < 0.5
            currentSpeed = 2.0; % 如果速度太低，使用默认值
        end
        
        % 预测无人机更远位置
        predictTime = 6.0;
        predictDist = currentSpeed * predictTime;
        
        % 计算预测位置 - 在无人机前方放置障碍物
        targetPoint = droneState(1:3) + directionVec * predictDist;
        
        % 添加随机偏移，确保障碍物更准确地拦截路径
        randomOffset = [rand()*1.0-0.5, rand()*1.0-0.5, rand()*3.0-1.5];
        targetPoint = targetPoint + randomOffset;
        
        % 确保目标点在地图范围内
        targetPoint = max([1, 1, 1], min(targetPoint, mapSize));
        
        % 确保与无人机有一定距离
        minDistToUAV = 10.0; % 最小距离
        actualDist = norm(targetPoint - droneState(1:3));
        
        if actualDist < minDistToUAV
            % 调整位置以保持最小距离
            dirToTarget = (targetPoint - droneState(1:3)) / actualDist;
            targetPoint = droneState(1:3) + dirToTarget * minDistToUAV;
        end
        
        % 更新障碍物位置
        if size(globalObstacles.dynamic, 1) > 0
            % 修改第一个障碍物位置
            globalObstacles.dynamic(1, 1:3) = targetPoint;
            
            % 重绘障碍物
            if ishandle(dynObsHandles{1})
                delete(dynObsHandles{1});
            end
            
            % 绘制新障碍物 - 使用更明显的红色和球体
            if ishandle(flightFig)
                figure(flightFig);
                [sx, sy, sz] = sphere(20);
                dynObsHandles{1} = surf(dynObsRadius*sx+targetPoint(1), dynObsRadius*sy+targetPoint(2), dynObsRadius*sz+targetPoint(3), ...
                                    'FaceColor', [1 0.2 0.2], 'EdgeColor', 'none', 'FaceAlpha', 0.9);
                
                % 更新编号文本标签
                textH = findobj(flightFig, 'Type', 'text', 'String', '1');
                if ~isempty(textH)
                    set(textH, 'Position', [targetPoint(1), targetPoint(2), targetPoint(3)+dynObsRadius+1]);
                else
                    text(targetPoint(1), targetPoint(2), targetPoint(3)+dynObsRadius+1, '1', ...
                        'Color', 'k', 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
                end
            end
            
            % 更新规划图中的障碍物
            if ishandle(planningFig)
                figure(planningFig);
                if ishandle(planning_dynObsHandles{1})
                    delete(planning_dynObsHandles{1});
                end
                
                [sx, sy, sz] = sphere(20);
                planning_dynObsHandles{1} = surf(dynObsRadius*sx+targetPoint(1), dynObsRadius*sy+targetPoint(2), dynObsRadius*sz+targetPoint(3), ...
                                           'FaceColor', [1 0.2 0.2], 'EdgeColor', 'none', 'FaceAlpha', 0.5);
            end
            
            % 创建避障用的负信息素
            manager = pheromoneInfo.pheromoneManager;
obstaclePos = estimatedObsPos;
radius = estimatedRadius * 2.5;

centerIdx = worldToGrid(manager, obstaclePos);
radiusGrid = ceil(radius / manager.gridSize);

for dx = -radiusGrid:radiusGrid
    for dy = -radiusGrid:radiusGrid
        for dz = -radiusGrid:radiusGrid
            idx = centerIdx + [dx, dy, dz];
            if isValidGrid(manager, idx)
                dist = sqrt(dx^2 + dy^2 + dz^2) * manager.gridSize;
                if dist <= radius
                    factor = exp(-dist^2/(2*(radius/2.0)^2));
                    reduction = manager.maxPheromone * factor * 3.0;
                    
                    manager.avoidancePheromones(idx(1), idx(2), idx(3)) = ...
                        manager.avoidancePheromones(idx(1), idx(2), idx(3)) + reduction * 3.0;
                    
                    current = manager.pathPheromones(idx(1), idx(2), idx(3));
                    manager.pathPheromones(idx(1), idx(2), idx(3)) = ...
                        max(manager.initialPheromone * 0.1, current - reduction * 2.5);
                    
                    current = manager.pheromoneMatrix(idx(1), idx(2), idx(3));
                    manager.pheromoneMatrix(idx(1), idx(2), idx(3)) = ...
                        max(manager.initialPheromone * 0.1, current - reduction * 2.5);
                    
                    manager.shortTermPheromones(idx(1), idx(2), idx(3)) = ...
                        max(0, manager.shortTermPheromones(idx(1), idx(2), idx(3)) - reduction * 2.0);
                end
            end
        end
    end
end
updateIntegratedPheromones(manager);
            
            % 更新信息素可视化
            if ishandle(pheromoneFig)
                figure(pheromoneFig);
                visualizePheromoneSlices(pheromoneManager, pheromoneFig);
            end
            
            % 返回到飞行图
            figure(flightFig);
            
            disp(['已将障碍物1放置在无人机前方 ', num2str(predictDist), ' 米处']);
        end
    else
        disp('无法放置障碍物: 路径不存在或太短');
    end
end

function togglePathFollowingMode(~, ~)
    % 切换路径跟随模式
    global strictPathFollowing pathFollowModeTextHandle;

    strictPathFollowing = ~strictPathFollowing;

    if strictPathFollowing
        disp('已切换到严格路径跟随模式');
        if ishandle(pathFollowModeTextHandle)
            set(pathFollowModeTextHandle, 'String', '严格路径跟随模式', 'Color', 'b', 'Visible', 'on');
        end
    else
        disp('已切换到普通路径跟随模式');
        if ishandle(pathFollowModeTextHandle)
            set(pathFollowModeTextHandle, 'String', '普通路径跟随模式', 'Color', [0.5 0.5 0], 'Visible', 'on');
        end
    end
end

function toggleObstacleMovement(~, ~)
    % 停止/启动障碍物移动
    global isObstacleMoving;

    isObstacleMoving = ~isObstacleMoving;

    if isObstacleMoving
        disp('已启动障碍物移动');
    else
        disp('已停止障碍物移动');
    end
end

function keyPressCallback(src, event)
    % 键盘回调函数
    global selectedObstacle globalObstacles dynObsHandles lastKeyPressTime lastMoveDirection;
    global planning_dynObsHandles planningFig flightFig dynObsRadius;
    global isObstacleMoving;

    % 获取当前时间，用于平滑移动
    currentTime = tic;
    timeSinceLastKey = toc(lastKeyPressTime);
    lastKeyPressTime = currentTime;

    % 移动速度 - 与时间间隔相关
    moveSpeed = 2.0;
    if timeSinceLastKey < 0.1
        moveSpeed = moveSpeed * 2; % 快速连续按键加速移动
    end

    % 根据按键处理操作
    switch event.Key
        case {'1', '2', '3'}
            % 选择障碍物
            selectedObstacle = str2double(event.Key);
            if selectedObstacle > size(globalObstacles.dynamic, 1)
                selectedObstacle = 0;
                disp('无效的障碍物索引');
            else
                disp(['已选中障碍物 ', num2str(selectedObstacle)]);
            end
            
        case 'escape'
            % 取消选择障碍物
            selectedObstacle = 0;
            disp('已取消选择障碍物');
            
        case 'space'
            % 暂停/继续
            pauseButtonCallback([], []);
            
        case {'leftarrow', 'rightarrow', 'uparrow', 'downarrow', 'pageup', 'pagedown'}
            % 移动选中的障碍物
            if selectedObstacle > 0 && selectedObstacle <= size(globalObstacles.dynamic, 1)
                obsPos = globalObstacles.dynamic(selectedObstacle, 1:3);
                
                % 确定移动方向
                moveDir = [0, 0, 0];
                
                switch event.Key
                    case 'leftarrow'
                        moveDir = [-moveSpeed, 0, 0];
                    case 'rightarrow'
                        moveDir = [moveSpeed, 0, 0];
                    case 'uparrow'
                        moveDir = [0, moveSpeed, 0];
                    case 'downarrow'
                        moveDir = [0, -moveSpeed, 0];
                    case 'pageup'
                        moveDir = [0, 0, moveSpeed];
                    case 'pagedown'
                        moveDir = [0, 0, -moveSpeed];
                end
                
                % 平滑移动 - 与上次方向结合
                if timeSinceLastKey < 0.1 && norm(lastMoveDirection) > 0
                    moveDir = 0.7 * moveDir + 0.3 * lastMoveDirection;
                end
                lastMoveDirection = moveDir;
                
                % 计算新位置
                newPos = obsPos + moveDir;
                
                % 确保在地图范围内
                global mapSize;
                newPos = max([dynObsRadius, dynObsRadius, dynObsRadius], ...
                          min(newPos, [mapSize(1)-dynObsRadius, mapSize(2)-dynObsRadius, mapSize(3)-dynObsRadius]));
                
                % 更新障碍物位置
                globalObstacles.dynamic(selectedObstacle, 1:3) = newPos;
                
                % 获取障碍物属性
                obsRadius = globalObstacles.dynamic(selectedObstacle, 4);
                obsColor = globalObstacles.dynamic(selectedObstacle, 5:7);
                
                % 更新飞行图中的障碍物
                if ishandle(flightFig)
                    figure(flightFig);
                    if ishandle(dynObsHandles{selectedObstacle})
                        delete(dynObsHandles{selectedObstacle});
                    end
                    
                    % 使用球体替代立方体
                    [sx, sy, sz] = sphere(20);
                    dynObsHandles{selectedObstacle} = surf(obsRadius*sx+newPos(1), obsRadius*sy+newPos(2), obsRadius*sz+newPos(3), ...
                                                     'FaceColor', obsColor, 'EdgeColor', 'none', 'FaceAlpha', 0.7);
                    
                    % 更新编号文本标签
                    textH = findobj(flightFig, 'Type', 'text', 'String', num2str(selectedObstacle));
                    if ~isempty(textH)
                        set(textH, 'Position', [newPos(1), newPos(2), newPos(3)+obsRadius+1]);
                    end
                end
                
                % 更新规划图中的障碍物
                if ishandle(planningFig)
                    figure(planningFig);
                    if ishandle(planning_dynObsHandles{selectedObstacle})
                        delete(planning_dynObsHandles{selectedObstacle});
                    end
                    
                    % 使用球体替代立方体
                    [sx, sy, sz] = sphere(20);
                    planning_dynObsHandles{selectedObstacle} = surf(obsRadius*sx+newPos(1), obsRadius*sy+newPos(2), obsRadius*sz+newPos(3), ...
                                                               'FaceColor', obsColor, 'EdgeColor', 'none', 'FaceAlpha', 0.5);
                end
                
                % 保存最后的移动方向
                lastMoveDirection = moveDir;
                
                % 返回到飞行图
                figure(flightFig);
            end
            
        case 'r'
            % 手动触发路径重规划
            replanPathCallback([], []);
            
        case 'i'
            % 手动触发障碍物干扰
            obstacleInterferenceCallback([], []);
            
        case 'o'
            % 停止/启动障碍物移动
            toggleObstacleMovement([], []);
            
        case 's'
            % 保存当前状态
            global droneState finalPath energy simulationTime flightPath;
            saveSimulationState(droneState, finalPath, globalObstacles, energy, simulationTime, flightPath);
            disp('已保存当前仿真状态');
            
        case 'd'
            % 切换调试模式
            global debugMode;
            debugMode = ~debugMode;
            if debugMode
                disp('调试模式已启用');
            else
                disp('调试模式已禁用');
            end
            
        case 't'
            % 切换路径跟随模式
            togglePathFollowingMode([], []);
    end
end

function val = getfield_default(struct, field, default)
    % 获取结构体字段值，如果不存在则使用默认值
    if isfield(struct, field)
        val = struct.(field);
    else
        val = default;
    end
end

function [loaded, droneState, path, obstacles, energy, time, originalPath] = loadSimulationState()
% 加载仿真状态
loaded = false;
droneState = [];
path = [];
obstacles = struct();
energy = 1000;
time = 0;
originalPath = [];

try
    loadedData = load('drone_simulation_state.mat');
    if isfield(loadedData, 'saveData')
        saveData = loadedData.saveData;
        droneState = saveData.droneState;
        path = saveData.path;
        obstacles = saveData.obstacles;
        energy = saveData.energy;
        time = saveData.time;
        originalPath = saveData.originalPath;
        loaded = true;
        disp('成功加载仿真状态');
    else
        disp('无法加载仿真状态：文件格式不正确');
    end
catch ME
    disp(['加载仿真状态失败: ', ME.message]);
end
end

function saveSimulationState(droneState, path, obstacles, energy, time, originalPath)
% 保存仿真状态
saveData = struct();
saveData.droneState = droneState;
saveData.path = path;
saveData.obstacles = obstacles;
saveData.energy = energy;
saveData.time = time;
saveData.originalPath = originalPath;

try
    save('drone_simulation_state.mat', 'saveData');
    disp('仿真状态已保存');
catch ME
    disp(['保存仿真状态失败: ', ME.message]);
end
end

function updateAutoMovingObstacles()
% 更新自动移动的障碍物 - 确保所有障碍物都在移动
global globalObstacles autoObsDirections autoObsSpeeds mapSize;
global dynObsHandles planning_dynObsHandles flightFig planningFig;
global dynObsRadius;

% 确保数据结构初始化
if isempty(autoObsDirections) || isempty(autoObsSpeeds)
    return;
end

% 处理所有动态障碍物
for i = 1:size(globalObstacles.dynamic, 1)
    % 获取当前障碍物属性
    obsPos = globalObstacles.dynamic(i, 1:3);
    obsRadius = globalObstacles.dynamic(i, 4);
    obsColor = globalObstacles.dynamic(i, 5:7);
    
    % 处理自动移动障碍物
    if i > 1 || (i == 1 && size(autoObsDirections, 1) >= 1)
        dirIdx = min(i, size(autoObsDirections, 1));
        speedIdx = min(i, size(autoObsSpeeds, 1));
        
        % 获取移动方向和速度
        moveDir = autoObsDirections(dirIdx, :);
        speed = autoObsSpeeds(speedIdx);
        
        % 计算新位置
        newPos = obsPos + moveDir * speed;
        
        % 检查是否到达边界，如果是则改变方向
        bounceX = false;
        bounceY = false;
        bounceZ = false;
        
        % X方向边界检查
        if newPos(1) - obsRadius < 0 || newPos(1) + obsRadius > mapSize(1)
            moveDir(1) = -moveDir(1);
            bounceX = true;
        end
        
        % Y方向边界检查
        if newPos(2) - obsRadius < 0 || newPos(2) + obsRadius > mapSize(2)
            moveDir(2) = -moveDir(2);
            bounceY = true;
        end
        
        % Z方向边界检查
        if newPos(3) - obsRadius < 0 || newPos(3) + obsRadius > mapSize(3)
            moveDir(3) = -moveDir(3);
            bounceZ = true;
        end
        
        % 如果碰到边界，重新计算新位置
        if bounceX || bounceY || bounceZ
            % 保存更新后的方向
            autoObsDirections(dirIdx, :) = moveDir;
            
            % 重新计算新位置
            newPos = obsPos + moveDir * speed;
        end
        
        % 更新障碍物位置
        globalObstacles.dynamic(i, 1:3) = newPos;
        
        % 更新飞行图中的障碍物
        if ishandle(flightFig)
            figure(flightFig);
            if ishandle(dynObsHandles{i})
                delete(dynObsHandles{i});
            end
            
            % 绘制球体障碍物
            [sx, sy, sz] = sphere(20);
            dynObsHandles{i} = surf(obsRadius*sx+newPos(1), obsRadius*sy+newPos(2), obsRadius*sz+newPos(3), ...
                                  'FaceColor', obsColor, 'EdgeColor', 'none', 'FaceAlpha', 0.7);
            
            % 更新编号文本标签
            textH = findobj(flightFig, 'Type', 'text', 'String', num2str(i));
            if ~isempty(textH)
                set(textH, 'Position', [newPos(1), newPos(2), newPos(3)+obsRadius+1]);
            end
        end
        
        % 更新规划图中的障碍物
        if ishandle(planningFig)
            figure(planningFig);
            if ishandle(planning_dynObsHandles{i})
                delete(planning_dynObsHandles{i});
            end
            
            % 绘制球体障碍物
            [sx, sy, sz] = sphere(20);
            planning_dynObsHandles{i} = surf(obsRadius*sx+newPos(1), obsRadius*sy+newPos(2), obsRadius*sz+newPos(3), ...
                                          'FaceColor', obsColor, 'EdgeColor', 'none', 'FaceAlpha', 0.5);
        end
    end
end
end

function dirText = getDirectionDescription(direction)
    % 获取方向描述文本
    if isempty(direction) || all(direction == 0)
        dirText = '无方向';
        return;
    end
    
    % 找出主要方向
    [~, maxIdx] = max(abs(direction));
    
    switch maxIdx
        case 1  % X轴
            if direction(1) > 0
                dirText = '向右';
            else
                dirText = '向左';
            end
        case 2  % Y轴
            if direction(2) > 0
                dirText = '向前';
            else
                dirText = '向后';
            end
        case 3  % Z轴
            if direction(3) > 0
                dirText = '向上';
            else
                dirText = '向下';
            end
        otherwise
            dirText = '未知';
    end
end
function queryPheromone(worldPos, radius)
    % 查询指定位置的信息素
    % 用法: queryPheromone([50, 50, 20], 2)
    
    global pheromoneManager gridSize;
    
    if nargin < 2
        radius = 2;
    end
    
    if isempty(pheromoneManager)
        fprintf('信息素管理器未初始化\n');
        return;
    end
    
    gridIdx = round(ceil(worldPos / gridSize));
    dims = size(pheromoneManager.pheromoneMatrix);
    
    fprintf('\n========================================\n');
    fprintf('信息素查询\n');
    fprintf('========================================\n');
    fprintf('查询位置: [%.2f, %.2f, %.2f]\n', worldPos(1), worldPos(2), worldPos(3));
    fprintf('网格坐标: [%d, %d, %d]\n', gridIdx(1), gridIdx(2), gridIdx(3));
    fprintf('查询半径: %d 格\n', radius);
    
    % 检查边界
    if gridIdx(1) < 1 || gridIdx(1) > dims(1) || ...
       gridIdx(2) < 1 || gridIdx(2) > dims(2) || ...
       gridIdx(3) < 1 || gridIdx(3) > dims(3)
        fprintf('❌ 位置超出范围！\n');
        return;
    end
    
    % 中心点
    fprintf('\n--- 中心点信息素 ---\n');
    fprintf('主矩阵: %.4f\n', pheromoneManager.pheromoneMatrix(gridIdx(1), gridIdx(2), gridIdx(3)));
    fprintf('路径层: %.4f\n', pheromoneManager.pathPheromones(gridIdx(1), gridIdx(2), gridIdx(3)));
    fprintf('避障层: %.4f\n', pheromoneManager.avoidancePheromones(gridIdx(1), gridIdx(2), gridIdx(3)));
    fprintf('能耗层: %.4f\n', pheromoneManager.energyPheromones(gridIdx(1), gridIdx(2), gridIdx(3)));
    
    % 周围统计
    fprintf('\n--- 周围区域统计 ---\n');
    values = [];
    for i = -radius:radius
        for j = -radius:radius
            for k = -radius:radius
                idx = gridIdx + [i, j, k];
                if idx(1) >= 1 && idx(1) <= dims(1) && ...
                   idx(2) >= 1 && idx(2) <= dims(2) && ...
                   idx(3) >= 1 && idx(3) <= dims(3)
                    dist = sqrt(i^2 + j^2 + k^2);
                    if dist <= radius
                        values = [values; pheromoneManager.pheromoneMatrix(idx(1), idx(2), idx(3))];
                    end
                end
            end
        end
    end
    
    fprintf('检测点数: %d\n', length(values));
    fprintf('平均值:   %.4f\n', mean(values));
    fprintf('最大值:   %.4f\n', max(values));
    fprintf('最小值:   %.4f\n', min(values));
    fprintf('中位数:   %.4f\n', median(values));
    fprintf('标准差:   %.4f\n', std(values));
    fprintf('========================================\n\n');
end
% ═══════════════════════════════════════════════
% 修改现有的更新回调函数
% ═══════════════════════════════════════════════
function updatePheromoneVisualizationCallback()
    % 更新信息素可视化回调
    global pheromoneManager pheromoneFig queryPoint;
    if ishandle(pheromoneFig)
        figure(pheromoneFig);
        if ~isempty(queryPoint)
            visualizePheromoneSlices(pheromoneManager, pheromoneFig, queryPoint);
            % 同时显示查询信息
            comprehensiveQueryPheromone(queryPoint, 3);
        else
            visualizePheromoneSlices(pheromoneManager, pheromoneFig);
        end
    end
end

function setQueryPointCallback()
    % 设置查询点回调
    global queryPoint droneState mapSize pheromoneManager pheromoneFig;
    
    % 提供默认值
    if isempty(droneState) || length(droneState) < 3
        defaultX = num2str(mapSize(1)/2);
        defaultY = num2str(mapSize(2)/2);
        defaultZ = num2str(mapSize(3)/2);
    else
        defaultX = num2str(droneState(1));
        defaultY = num2str(droneState(2));
        defaultZ = num2str(droneState(3));
    end
    
    % 提示用户输入
    answer = inputdlg({'X坐标:', 'Y坐标:', 'Z坐标:'}, ...
                      '设置查询点坐标', 1, ...
                      {defaultX, defaultY, defaultZ});
    
    if ~isempty(answer)
        x = str2double(answer{1});
        y = str2double(answer{2});
        z = str2double(answer{3});
        
        if ~isnan(x) && ~isnan(y) && ~isnan(z)
            % 确保坐标在地图范围内
            x = max(1, min(x, mapSize(1)));
            y = max(1, min(y, mapSize(2)));
            z = max(1, min(z, mapSize(3)));
            
            queryPoint = [x, y, z];
            fprintf('\n查询点已设置为: [%.2f, %.2f, %.2f]\n', x, y, z);
            
            % 执行查询
            comprehensiveQueryPheromone(queryPoint, 3);
            
            % 立即更新可视化
            if ishandle(pheromoneFig)
                figure(pheromoneFig);
                visualizePheromoneSlices(pheromoneManager, pheromoneFig, queryPoint);
            end
        else
            fprintf('无效的坐标输入\n');
        end

        
    end
end

function queryCurrentPositionCallback()
    % 查询当前无人机位置的信息素回调
    global queryPoint droneState pheromoneManager pheromoneFig;
    
    if ~isempty(droneState) && length(droneState) >= 3
        queryPoint = droneState(1:3);
        fprintf('\n已设置查询点为当前无人机位置: [%.2f, %.2f, %.2f]\n', ...
                queryPoint(1), queryPoint(2), queryPoint(3));
        
        % 执行查询
        comprehensiveQueryPheromone(queryPoint, 3);
        
        % 立即更新可视化
        if ishandle(pheromoneFig)
            figure(pheromoneFig);
            visualizePheromoneSlices(pheromoneManager, pheromoneFig, queryPoint);
        end
    else
        fprintf('无人机位置尚未初始化\n');
    end
end
function comprehensiveQueryPheromone(worldPos, radius)
    % 综合查询指定位置的信息素 - 包括初始、动态障碍物、静态障碍物
    % 增强版：实时可视化障碍物信息素影响
    
    global pheromoneManager gridSize globalObstacles pheromoneFig;
    
    if nargin < 1
        fprintf('请提供查询位置: comprehensiveQueryPheromone([x, y, z], radius)\n');
        return;
    end
    
    if nargin < 2
        radius = 3;
    end
    
    if isempty(pheromoneManager)
        fprintf('信息素管理器未初始化\n');
        return;
    end
    
    gridIdx = ceil(worldPos / gridSize);
    dims = size(pheromoneManager.pheromoneMatrix);
    
    fprintf('\n╔════════════════════════════════════════════════════════════╗\n');
    fprintf('║           信息素综合查询系统 (实时障碍物可视化)           ║\n');
    fprintf('╚════════════════════════════════════════════════════════════╝\n');
    fprintf('查询位置: [%.2f, %.2f, %.2f]\n', worldPos(1), worldPos(2), worldPos(3));
    fprintf('网格坐标: [%d, %d, %d]\n', gridIdx(1), gridIdx(2), gridIdx(3));
    fprintf('查询半径: %d 格\n', radius);
    
    % 边界检查
    if gridIdx(1) < 1 || gridIdx(1) > dims(1) || ...
       gridIdx(2) < 1 || gridIdx(2) > dims(2) || ...
       gridIdx(3) < 1 || gridIdx(3) > dims(3)
        fprintf('❌ 位置超出范围!\n');
        return;
    end
    
    % ═══════════════════════════════════════════════
    % 1. 中心点详细信息
    % ═══════════════════════════════════════════════
    fprintf('\n┌─────────────────────────────────────────┐\n');
    fprintf('│ 1️⃣  中心点信息素详情                    │\n');
    fprintf('└─────────────────────────────────────────┘\n');
    
    centerMainValue = pheromoneManager.pheromoneMatrix(gridIdx(1), gridIdx(2), gridIdx(3));
    centerPathValue = pheromoneManager.pathPheromones(gridIdx(1), gridIdx(2), gridIdx(3));
    centerAvoidValue = pheromoneManager.avoidancePheromones(gridIdx(1), gridIdx(2), gridIdx(3));
    centerEnergyValue = pheromoneManager.energyPheromones(gridIdx(1), gridIdx(2), gridIdx(3));
    
    fprintf('  主矩阵:   %.4f  %s\n', centerMainValue, getPheromoneBar(centerMainValue));
    fprintf('  路径层:   %.4f  %s\n', centerPathValue, getPheromoneBar(centerPathValue));
    fprintf('  避障层:   %.4f  %s\n', centerAvoidValue, getPheromoneBar(centerAvoidValue));
    fprintf('  能耗层:   %.4f  %s\n', centerEnergyValue, getPheromoneBar(centerEnergyValue));
    
    % ═══════════════════════════════════════════════
    % 2. 周围区域统计
    % ═══════════════════════════════════════════════
    fprintf('\n┌─────────────────────────────────────────┐\n');
    fprintf('│ 2️⃣  周围区域统计 (半径=%d格)            │\n', radius);
    fprintf('└─────────────────────────────────────────┘\n');
    
    allValues = [];
    pathValues = [];
    avoidValues = [];
    energyValues = [];
    positions = [];
    distances = [];
    
    for i = -radius:radius
        for j = -radius:radius
            for k = -radius:radius
                idx = gridIdx + [i, j, k];
                if all(idx >= 1) && all(idx <= dims)
                    dist = sqrt(i^2 + j^2 + k^2);
                    if dist <= radius
                        allValues = [allValues; pheromoneManager.pheromoneMatrix(idx(1), idx(2), idx(3))];
                        pathValues = [pathValues; pheromoneManager.pathPheromones(idx(1), idx(2), idx(3))];
                        avoidValues = [avoidValues; pheromoneManager.avoidancePheromones(idx(1), idx(2), idx(3))];
                        energyValues = [energyValues; pheromoneManager.energyPheromones(idx(1), idx(2), idx(3))];
                        positions = [positions; idx];
                        distances = [distances; dist];
                    end
                end
            end
        end
    end
    
    fprintf('  检测点数: %d\n', length(allValues));
    fprintf('  ─────────────────────────────────────\n');
    fprintf('  主矩阵 - 平均:%.4f 最大:%.4f 最小:%.4f\n', mean(allValues), max(allValues), min(allValues));
    fprintf('  路径层 - 平均:%.4f 最大:%.4f 最小:%.4f\n', mean(pathValues), max(pathValues), min(pathValues));
    fprintf('  避障层 - 平均:%.4f 最大:%.4f 最小:%.4f\n', mean(avoidValues), max(avoidValues), min(avoidValues));
    fprintf('  能耗层 - 平均:%.4f 最大:%.4f 最小:%.4f\n', mean(energyValues), max(energyValues), min(energyValues));
    
    % ═══════════════════════════════════════════════
    % 3. 动态障碍物信息素影响 (增强版)
    % ═══════════════════════════════════════════════
    fprintf('\n┌─────────────────────────────────────────┐\n');
    fprintf('│ 3️⃣  动态障碍物信息素影响 (实时可视化)  │\n');
    fprintf('└─────────────────────────────────────────┘\n');
    
    dynamicObstacleData = [];  % 存储动态障碍物数据用于可视化
    
    if isfield(globalObstacles, 'dynamic') && ~isempty(globalObstacles.dynamic)
        foundDynamic = false;
        visualRange = 30.0;  % 可视化范围
        
        for i = 1:size(globalObstacles.dynamic, 1)
            obsPos = globalObstacles.dynamic(i, 1:3);
            obsRadius = globalObstacles.dynamic(i, 4);
            distToObs = norm(worldPos - obsPos);
            
            if distToObs <= visualRange
                foundDynamic = true;
                fprintf('  🔴 动态障碍物 #%d:\n', i);
                fprintf('     位置: [%.1f, %.1f, %.1f]\n', obsPos(1), obsPos(2), obsPos(3));
                fprintf('     半径: %.1f 距离: %.2f\n', obsRadius, distToObs);
                
                % 分析该障碍物周围的信息素
                obsInfo = analyzePheromoneAroundPoint(pheromoneManager, obsPos, ceil(obsRadius*2.0/gridSize), gridSize);
                if obsInfo.valid
                    fprintf('     中心避障信息素: %.4f\n', obsInfo.centerValue);
                    fprintf('     周围避障层均值: %.4f\n', obsInfo.avoidanceLayerMean);
                    fprintf('     影响强度: %s\n', getInfluenceLevel(obsInfo.avoidanceLayerMean));
                    
                    % 计算理论影响区域
                    effectiveRadius = obsRadius * 2.5;
                    fprintf('     理论影响半径: %.2f米\n', effectiveRadius);
                    
                    % 存储障碍物数据用于可视化
                    obstacleData = struct();
                    obstacleData.position = obsPos;
                    obstacleData.radius = obsRadius;
                    obstacleData.effectiveRadius = effectiveRadius;
                    obstacleData.distance = distToObs;
                    obstacleData.pheromoneInfo = obsInfo;
                    obstacleData.type = 'dynamic';
                    obstacleData.index = i;
                    dynamicObstacleData = [dynamicObstacleData; obstacleData];
                end
            end
        end
        
        if ~foundDynamic
            fprintf('  ✓ 查询范围内无动态障碍物影响 (搜索半径:%.1f米)\n', visualRange);
        end
    else
        fprintf('  ℹ️  无动态障碍物\n');
    end
    
    % ═══════════════════════════════════════════════
    % 4. 静态障碍物信息素影响 (增强版)
    % ═══════════════════════════════════════════════
    fprintf('\n┌─────────────────────────────────────────┐\n');
    fprintf('│ 4️⃣  静态障碍物信息素影响 (实时可视化)  │\n');
    fprintf('└─────────────────────────────────────────┘\n');
    
    staticObstacleData = [];  % 存储静态障碍物数据用于可视化
    foundStatic = false;
    staticSearchRange = 30.0;
    
    % 检查建筑物
    if isfield(globalObstacles, 'buildings') && ~isempty(globalObstacles.buildings)
        for i = 1:size(globalObstacles.buildings, 1)
            x = globalObstacles.buildings(i, 1);
            y = globalObstacles.buildings(i, 2);
            width = globalObstacles.buildings(i, 3);
            depth = globalObstacles.buildings(i, 4);
            height = globalObstacles.buildings(i, 5);
            
            % 计算到建筑物中心的距离
            buildingCenter = [x, y, height/2];
            distToBuilding = norm(worldPos - buildingCenter);
            
            % 检查是否在搜索范围内
            if distToBuilding <= staticSearchRange
                foundStatic = true;
                fprintf('  🏢 建筑物 #%d:\n', i);
                fprintf('     中心: [%.1f, %.1f] 尺寸: %.1fx%.1f 高度: %.1f\n', ...
                        x, y, width, depth, height);
                
                % 计算到建筑物边缘的最短距离
                distX = max(0, max(x-width/2 - worldPos(1), worldPos(1) - (x+width/2)));
                distY = max(0, max(y-depth/2 - worldPos(2), worldPos(2) - (y+depth/2)));
                distZ = max(0, worldPos(3) - height);
                edgeDist = sqrt(distX^2 + distY^2 + distZ^2);
                
                fprintf('     到边缘距离: %.2f米\n', edgeDist);
                fprintf('     到中心距离: %.2f米\n', distToBuilding);
                
                % 分析建筑物周围信息素
                buildingInfo = analyzePheromoneAroundPoint(pheromoneManager, buildingCenter, 3, gridSize);
                if buildingInfo.valid
                    fprintf('     周围避障层均值: %.4f\n', buildingInfo.avoidanceLayerMean);
                end
                
                % 存储建筑物数据
                obstacleData = struct();
                obstacleData.position = buildingCenter;
                obstacleData.size = [width, depth, height];
                obstacleData.edgeDistance = edgeDist;
                obstacleData.centerDistance = distToBuilding;
                obstacleData.type = 'building';
                obstacleData.index = i;
                staticObstacleData = [staticObstacleData; obstacleData];
            end
        end
    end
    
    % 检查树木
    if isfield(globalObstacles, 'trees') && ~isempty(globalObstacles.trees)
        for i = 1:size(globalObstacles.trees, 1)
            x = globalObstacles.trees(i, 1);
            y = globalObstacles.trees(i, 2);
            treeRadius = globalObstacles.trees(i, 3);
            treeHeight = globalObstacles.trees(i, 4);
            
            treeCenter = [x, y, treeHeight/2];
            distToTree = norm(worldPos - treeCenter);
            
            if distToTree <= staticSearchRange
                foundStatic = true;
                fprintf('  🌳 树木 #%d:\n', i);
                fprintf('     位置: [%.1f, %.1f] 半径: %.1f 高度: %.1f\n', ...
                        x, y, treeRadius, treeHeight);
                
                horizontalDist = sqrt((worldPos(1)-x)^2 + (worldPos(2)-y)^2);
                fprintf('     水平距离: %.2f米\n', horizontalDist);
                fprintf('     3D距离: %.2f米\n', distToTree);
                
                % 分析树木周围信息素
                treeInfo = analyzePheromoneAroundPoint(pheromoneManager, treeCenter, 2, gridSize);
                if treeInfo.valid
                    fprintf('     周围避障层均值: %.4f\n', treeInfo.avoidanceLayerMean);
                end
                
                % 存储树木数据
                obstacleData = struct();
                obstacleData.position = treeCenter;
                obstacleData.radius = treeRadius;
                obstacleData.height = treeHeight;
                obstacleData.distance = distToTree;
                obstacleData.type = 'tree';
                obstacleData.index = i;
                staticObstacleData = [staticObstacleData; obstacleData];
            end
        end
    end
    
    if ~foundStatic
        fprintf('  ✓ 查询范围内无静态障碍物影响 (搜索半径:%.1f米)\n', staticSearchRange);
    end
    
    % ═══════════════════════════════════════════════
    % 5. 初始地图信息素基准
    % ═══════════════════════════════════════════════
    fprintf('\n┌─────────────────────────────────────────┐\n');
    fprintf('│ 5️⃣  初始地图信息素基准                  │\n');
    fprintf('└─────────────────────────────────────────┘\n');
    
    fprintf('  初始值: %.4f\n', pheromoneManager.initialPheromone);
    fprintf('  最大值: %.4f\n', pheromoneManager.maxPheromone);
    fprintf('  全局平均: %.4f\n', mean(pheromoneManager.pheromoneMatrix(:)));
    fprintf('  全局标准差: %.4f\n', std(pheromoneManager.pheromoneMatrix(:)));
    
    % 计算偏离初始值的程度
    deviation = centerMainValue - pheromoneManager.initialPheromone;
    deviationPercent = (deviation / pheromoneManager.initialPheromone) * 100;
    
    fprintf('\n  当前位置偏离初始值: %.4f (%.1f%%)\n', deviation, deviationPercent);
    if abs(deviationPercent) < 10
        fprintf('  状态: ✓ 接近初始状态\n');
    elseif deviationPercent > 0
        fprintf('  状态: ↑ 正信息素积累 (路径优化区域)\n');
    else
        fprintf('  状态: ↓ 负信息素积累 (障碍区域)\n');
    end
    
    % ═══════════════════════════════════════════════
    % 6. 信息素梯度分析
    % ═══════════════════════════════════════════════
    fprintf('\n┌─────────────────────────────────────────┐\n');
    fprintf('│ 6️⃣  信息素梯度分析                      │\n');
    fprintf('└─────────────────────────────────────────┘\n');
    
    [~, gradient] = getPheromoneValue(pheromoneManager, worldPos);
    gradientMag = norm(gradient);
    
    fprintf('  梯度向量: [%.4f, %.4f, %.4f]\n', gradient(1), gradient(2), gradient(3));
    fprintf('  梯度强度: %.4f %s\n', gradientMag, getGradientBar(gradientMag));
    
    if gradientMag > 0.1
        fprintf('  主导方向: %s\n', getDirectionFromVector(gradient));
        fprintf('  含义: 信息素变化剧烈,可能接近障碍物或路径边界\n');
    else
        fprintf('  含义: 信息素分布平缓,区域较为均匀\n');
    end
    
    fprintf('\n╚════════════════════════════════════════════════════════════╝\n');
    
    % ═══════════════════════════════════════════════
    % 7. 创建实时可视化 - 障碍物信息素影响图
    % ═══════════════════════════════════════════════
    if ~isempty(dynamicObstacleData) || ~isempty(staticObstacleData)
        fprintf('\n🎨 正在生成障碍物信息素影响可视化...\n');
        visualizeObstaclePheromoneInfluence(worldPos, dynamicObstacleData, staticObstacleData, pheromoneManager);
    end
    
    fprintf('\n╔════════════════════════════════════════════════════════════╗\n');
    fprintf('║           查询完成                                         ║\n');
    fprintf('╚════════════════════════════════════════════════════════════╝\n\n');
end

% ═══════════════════════════════════════════════
% 新增：障碍物信息素影响可视化函数
% ═══════════════════════════════════════════════
function visualizeObstaclePheromoneInfluence(queryPos, dynamicObstacles, staticObstacles, manager)
    % 创建独立的可视化窗口显示障碍物信息素影响
    
    % 创建新窗口
    obstacleFig = figure('Name', '障碍物信息素影响分析', 'Position', [100, 100, 1200, 800]);
    
    % ═══════════════════════════════════════════════
    % 子图1: XY平面 - 查询点周围的信息素分布
    % ═══════════════════════════════════════════════
    subplot(2, 2, 1);
    hold on;
    title('XY平面 - 避障信息素分布');
    xlabel('X轴'); ylabel('Y轴');
    
    % 绘制查询点周围的信息素热图
    plotRange = 40;  % 绘制范围
    gridRes = 1;     % 网格分辨率
    
    [X, Y] = meshgrid(max(1, queryPos(1)-plotRange):gridRes:min(manager.mapSize(1), queryPos(1)+plotRange), ...
                      max(1, queryPos(2)-plotRange):gridRes:min(manager.mapSize(2), queryPos(2)+plotRange));
    
    Z_avoid = zeros(size(X));
    for i = 1:size(X, 1)
        for j = 1:size(X, 2)
            gridIdx = ceil([X(i,j), Y(i,j), queryPos(3)] / manager.gridSize);
            dims = size(manager.avoidancePheromones);
            if all(gridIdx >= 1) && all(gridIdx <= dims)
                Z_avoid(i,j) = manager.avoidancePheromones(gridIdx(1), gridIdx(2), gridIdx(3));
            end
        end
    end
    
    % 绘制热图
    contourf(X, Y, Z_avoid, 20);
    colormap(jet);
    colorbar;
    
    % 绘制动态障碍物
    for i = 1:length(dynamicObstacles)
        obs = dynamicObstacles(i);
        viscircles(obs.position(1:2), obs.radius, 'Color', 'r', 'LineWidth', 2);
        viscircles(obs.position(1:2), obs.effectiveRadius, 'Color', 'r', 'LineStyle', '--', 'LineWidth', 1);
        text(obs.position(1), obs.position(2), sprintf('D%d', obs.index), ...
             'Color', 'w', 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    end
    
    % 绘制静态障碍物
    for i = 1:length(staticObstacles)
        obs = staticObstacles(i);
        if strcmp(obs.type, 'building')
            width = obs.size(1);
            depth = obs.size(2);
            rectangle('Position', [obs.position(1)-width/2, obs.position(2)-depth/2, width, depth], ...
                     'EdgeColor', 'k', 'LineWidth', 2);
            text(obs.position(1), obs.position(2), sprintf('B%d', obs.index), ...
                 'Color', 'k', 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
        elseif strcmp(obs.type, 'tree')
            viscircles(obs.position(1:2), obs.radius, 'Color', 'g', 'LineWidth', 2);
            text(obs.position(1), obs.position(2), sprintf('T%d', obs.index), ...
                 'Color', 'g', 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
        end
    end
    
    % 标记查询点
    plot(queryPos(1), queryPos(2), 'w*', 'MarkerSize', 20, 'LineWidth', 3);
    plot(queryPos(1), queryPos(2), 'ko', 'MarkerSize', 10, 'LineWidth', 2);
    
    axis equal;
    grid on;
    
    % ═══════════════════════════════════════════════
    % 子图2: 3D视图 - 障碍物和信息素云
    % ═══════════════════════════════════════════════
    subplot(2, 2, 2);
    hold on;
    title('3D视图 - 障碍物信息素影响范围');
    xlabel('X轴'); ylabel('Y轴'); zlabel('Z轴');
    
    % 绘制动态障碍物及其影响范围
    for i = 1:length(dynamicObstacles)
        obs = dynamicObstacles(i);
        
        % 绘制障碍物实体
        [sx, sy, sz] = sphere(20);
        surf(obs.radius*sx+obs.position(1), obs.radius*sy+obs.position(2), ...
             obs.radius*sz+obs.position(3), 'FaceColor', [1 0.2 0.2], ...
             'EdgeColor', 'none', 'FaceAlpha', 0.8);
        
        % 绘制影响范围（半透明球体）
        surf(obs.effectiveRadius*sx+obs.position(1), ...
             obs.effectiveRadius*sy+obs.position(2), ...
             obs.effectiveRadius*sz+obs.position(3), ...
             'FaceColor', [1 0.5 0.5], 'EdgeColor', 'none', 'FaceAlpha', 0.2);
        
        % 标签
        text(obs.position(1), obs.position(2), obs.position(3)+obs.effectiveRadius+2, ...
             sprintf('动态#%d\n影响:%.1fm', obs.index, obs.effectiveRadius), ...
             'Color', 'r', 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    end
    
    % 绘制静态障碍物
    for i = 1:length(staticObstacles)
        obs = staticObstacles(i);
        if strcmp(obs.type, 'building')
            width = obs.size(1);
            depth = obs.size(2);
            height = obs.size(3);
            plotBuilding([obs.position(1)-width/2, obs.position(2)-depth/2, 0], ...
                        width, depth, height, [0.7 0.7 0.7], 0.5);
        elseif strcmp(obs.type, 'tree')
            % 树干
            [cx, cy, cz] = cylinder(obs.radius/3, 8);
            cz = cz * obs.height * 0.6;
            surf(cx+obs.position(1), cy+obs.position(2), cz, ...
                'FaceColor', [0.6 0.3 0], 'EdgeColor', 'none', 'FaceAlpha', 0.7);
            
            % 树冠
            [sx, sy, sz] = sphere(8);
            surf(obs.radius*sx+obs.position(1), obs.radius*sy+obs.position(2), ...
                 obs.radius*sz+obs.height*0.6+obs.radius, ...
                 'FaceColor', [0.1 0.6 0.1], 'EdgeColor', 'none', 'FaceAlpha', 0.7);
        end
    end
    
    % 标记查询点
    plot3(queryPos(1), queryPos(2), queryPos(3), 'y*', 'MarkerSize', 25, 'LineWidth', 4);
    
    % 绘制查询点到各障碍物的连线
    for i = 1:length(dynamicObstacles)
        obs = dynamicObstacles(i);
        plot3([queryPos(1), obs.position(1)], [queryPos(2), obs.position(2)], ...
              [queryPos(3), obs.position(3)], 'r--', 'LineWidth', 1);
    end
    
    grid on;
    view(3);
    axis equal;
    camlight;
    lighting gouraud;
    
    % ═══════════════════════════════════════════════
    % 子图3: 信息素强度随距离变化曲线
    % ═══════════════════════════════════════════════
    subplot(2, 2, 3);
    hold on;
    title('信息素强度 vs 距离障碍物');
    xlabel('距离障碍物(米)'); ylabel('避障信息素强度');
    grid on;
    
    % 对每个动态障碍物绘制曲线
    for i = 1:length(dynamicObstacles)
        obs = dynamicObstacles(i);
        
        % 生成从障碍物中心到查询点方向上的采样点
        direction = queryPos - obs.position;
        if norm(direction) > 0
            direction = direction / norm(direction);
        end
        
        distances = 0:0.5:min(obs.distance+5, 50);
        pheromoneValues = zeros(size(distances));
        
        for j = 1:length(distances)
            samplePos = obs.position + direction * distances(j);
            gridIdx = ceil(samplePos / manager.gridSize);
            dims = size(manager.avoidancePheromones);
            if all(gridIdx >= 1) && all(gridIdx <= dims)
                pheromoneValues(j) = manager.avoidancePheromones(gridIdx(1), gridIdx(2), gridIdx(3));
            end
        end
        
        plot(distances, pheromoneValues, 'LineWidth', 2, 'DisplayName', sprintf('动态#%d', obs.index));
        
        % 标记查询点位置
        plot(obs.distance, pheromoneValues(find(distances >= obs.distance, 1)), ...
             'o', 'MarkerSize', 10, 'LineWidth', 2);
    end
    
    legend('Location', 'best');
    
    % ═══════════════════════════════════════════════
    % 子图4: 障碍物信息统计表
    % ═══════════════════════════════════════════════
    subplot(2, 2, 4);
    axis off;
    
    % 创建表格数据
    tableData = {};
    tableHeaders = {'类型', '编号', '距离(m)', '影响强度', '信息素值'};
    
    % 添加动态障碍物数据
    for i = 1:length(dynamicObstacles)
        obs = dynamicObstacles(i);
        influenceLevel = getInfluenceLevel(obs.pheromoneInfo.avoidanceLayerMean);
        tableData{end+1, 1} = '🔴动态';
        tableData{end, 2} = sprintf('#%d', obs.index);
        tableData{end, 3} = sprintf('%.2f', obs.distance);
        tableData{end, 4} = influenceLevel;
        tableData{end, 5} = sprintf('%.2f', obs.pheromoneInfo.avoidanceLayerMean);
    end
    
    % 添加静态障碍物数据
    for i = 1:length(staticObstacles)
        obs = staticObstacles(i);
        if strcmp(obs.type, 'building')
            tableData{end+1, 1} = '🏢建筑';
            tableData{end, 2} = sprintf('#%d', obs.index);
            tableData{end, 3} = sprintf('%.2f', obs.edgeDistance);
            tableData{end, 4} = '中';
            tableData{end, 5} = '-';
        elseif strcmp(obs.type, 'tree')
            tableData{end+1, 1} = '🌳树木';
            tableData{end, 2} = sprintf('#%d', obs.index);
            tableData{end, 3} = sprintf('%.2f', obs.distance);
            tableData{end, 4} = '弱';
            tableData{end, 5} = '-';
        end
    end
    
    % 显示表格
    if ~isempty(tableData)
        % 计算表格位置
        numRows = size(tableData, 1);
        yPos = 0.9;
        yStep = 0.8 / (numRows + 2);
        
        % 绘制表头
        text(0.05, yPos, tableHeaders{1}, 'FontWeight', 'bold', 'FontSize', 10);
        text(0.25, yPos, tableHeaders{2}, 'FontWeight', 'bold', 'FontSize', 10);
        text(0.40, yPos, tableHeaders{3}, 'FontWeight', 'bold', 'FontSize', 10);
        text(0.60, yPos, tableHeaders{4}, 'FontWeight', 'bold', 'FontSize', 10);
        text(0.80, yPos, tableHeaders{5}, 'FontWeight', 'bold', 'FontSize', 10);
        yPos = yPos - yStep;
        
        % 绘制分隔线
        line([0.02, 0.98], [yPos, yPos], 'Color', 'k', 'LineWidth', 2);
        yPos = yPos - yStep;
        
        % 绘制数据行
        for i = 1:size(tableData, 1)
            text(0.05, yPos, tableData{i, 1}, 'FontSize', 9);
            text(0.25, yPos, tableData{i, 2}, 'FontSize', 9);
            text(0.40, yPos, tableData{i, 3}, 'FontSize', 9);
            text(0.60, yPos, tableData{i, 4}, 'FontSize', 9);
            text(0.80, yPos, tableData{i, 5}, 'FontSize', 9);
            yPos = yPos - yStep;
        end
        
        % 添加总结信息
        yPos = 0.15;
        text(0.05, yPos, sprintf('查询点: [%.1f, %.1f, %.1f]', queryPos(1), queryPos(2), queryPos(3)), ...
             'FontSize', 10, 'FontWeight', 'bold');
        yPos = yPos - 0.08;
        text(0.05, yPos, sprintf('检测到 %d 个障碍物影响', length(dynamicObstacles) + length(staticObstacles)), ...
             'FontSize', 10);
        yPos = yPos - 0.08;
        text(0.05, yPos, sprintf('最近障碍物距离: %.2f 米', getMinDistance(dynamicObstacles, staticObstacles)), ...
             'FontSize', 10);
    else
        text(0.5, 0.5, '查询范围内无障碍物', 'FontSize', 14, ...
             'HorizontalAlignment', 'center', 'FontWeight', 'bold');
    end
    
    title('障碍物统计表', 'FontSize', 12, 'FontWeight', 'bold');
    
    % 添加总标题
    sgtitle(sprintf('障碍物信息素影响分析 - 查询点: [%.1f, %.1f, %.1f]', ...
                    queryPos(1), queryPos(2), queryPos(3)), ...
            'FontSize', 14, 'FontWeight', 'bold');
end

% ═══════════════════════════════════════════════
% 辅助函数：获取最近障碍物距离
% ═══════════════════════════════════════════════
function minDist = getMinDistance(dynamicObs, staticObs)
    minDist = inf;
    
    for i = 1:length(dynamicObs)
        if dynamicObs(i).distance < minDist
            minDist = dynamicObs(i).distance;
        end
    end
    
    for i = 1:length(staticObs)
        if strcmp(staticObs(i).type, 'building')
            dist = staticObs(i).edgeDistance;
        else
            dist = staticObs(i).distance;
        end
        
        if dist < minDist
            minDist = dist;
        end
    end
    
    if isinf(minDist)
        minDist = 0;
    end
end

% ═══════════════════════════════════════════════
% 辅助函数
% ═══════════════════════════════════════════════

function bar = getPheromoneBar(value)
    % 生成信息素值的可视化条形图
    maxBar = 20;
    barLength = min(maxBar, round(value / 10 * maxBar));
    if barLength < 0
        barLength = 0;
    end
    bar = ['[', repmat('█', 1, barLength), repmat('░', 1, maxBar-barLength), ']'];
end

function level = getInfluenceLevel(value)
    % 获取影响强度级别
    if value > 50
        level = '🔴 极强';
    elseif value > 20
        level = '🟠 强';
    elseif value > 5
        level = '🟡 中等';
    else
        level = '🟢 弱';
    end
end

function bar = getGradientBar(magnitude)
    % 生成梯度强度可视化
    if magnitude > 0.5
        bar = '[▰▰▰▰▰▰▰▰▰▰] 极强';
    elseif magnitude > 0.3
        bar = '[▰▰▰▰▰▰▰░░░] 强';
    elseif magnitude > 0.1
        bar = '[▰▰▰▰░░░░░░] 中';
    else
        bar = '[▰▰░░░░░░░░] 弱';
    end
end

function dirStr = getDirectionFromVector(vec)
    % 从向量获取主要方向描述
    [~, maxIdx] = max(abs(vec));
    switch maxIdx
        case 1
            dirStr = sprintf('X轴 (%s)', ternary(vec(1)>0, '→东', '←西'));
        case 2
            dirStr = sprintf('Y轴 (%s)', ternary(vec(2)>0, '↑北', '↓南'));
        case 3
            dirStr = sprintf('Z轴 (%s)', ternary(vec(3)>0, '⬆上', '⬇下'));
    end
end

function result = ternary(condition, trueVal, falseVal)
    % 三元运算符
    if condition
        result = trueVal;
    else
        result = falseVal;
    end
end

% ═══════════════════════════════════════════════
% 快捷查询函数
% ═══════════════════════════════════════════════

function quickQueryDrone()
    % 快速查询无人机当前位置的信息素
    global droneState;
    if ~isempty(droneState) && length(droneState) >= 3
        comprehensiveQueryPheromone(droneState(1:3), 3);
    else
        fprintf('无人机位置未初始化\n');
    end
end

function quickQueryGoal()
    % 快速查询目标点的信息素
    global goal;
    if ~isempty(goal)
        comprehensiveQueryPheromone(goal, 3);
    else
        fprintf('目标点未设置\n');
    end
end

function quickQueryCustom(x, y, z, radius)
    % 快速自定义查询
    % 用法: quickQueryCustom(50, 50, 20, 3)
    if nargin < 4
        radius = 3;
    end
    comprehensiveQueryPheromone([x, y, z], radius);
end

% ═══════════════════════════════════════════════
% 实时监控函数
% ═══════════════════════════════════════════════
function startRealtimeMonitoring(interval)
    % 启动实时监控提示
    % 用法: startRealtimeMonitoring(5)  % 每5秒查询一次
    
    if nargin < 1
        interval = 5;
    end
    
    fprintf('\n╔════════════════════════════════════════════════════════════╗\n');
    fprintf('║           实时监控配置指南                                 ║\n');
    fprintf('╚════════════════════════════════════════════════════════════╝\n');
    fprintf('监控间隔: %.1f 秒\n\n', interval);
    fprintf('请在主仿真循环中添加以下代码：\n\n');
    fprintf('persistent lastMonitorTime;\n');
    fprintf('if isempty(lastMonitorTime); lastMonitorTime = 0; end\n');
    fprintf('if simulationTime - lastMonitorTime >= %.1f\n', interval);
    fprintf('    try\n');
    fprintf('        comprehensiveQueryPheromone(droneState(1:3), 3);\n');
    fprintf('        lastMonitorTime = simulationTime;\n');
    fprintf('    catch ME\n');
    fprintf('        disp([''监控查询出错: '', ME.message]);\n');
    fprintf('    end\n');
    fprintf('end\n\n');
    fprintf('或者使用简化版本：\n\n');
    fprintf('if mod(round(simulationTime), %.0f) == 0 && mod(round(simulationTime/dt), 5) == 0\n', interval);
    fprintf('    quickQueryDrone();\n');
    fprintf('end\n\n');
    fprintf('╚════════════════════════════════════════════════════════════╝\n\n');
end

% ═══════════════════════════════════════════════
% 批量查询函数 - 查询路径上多个点
% ═══════════════════════════════════════════════
function batchQueryPath(numPoints)
    % 批量查询规划路径上的信息素
    % 用法: batchQueryPath(5)  % 查询路径上5个点
    
    global flightPath;
    
    if nargin < 1
        numPoints = 5;
    end
    
    if isempty(flightPath) || size(flightPath, 1) < numPoints
        fprintf('路径点不足\n');
        return;
    end
    
    fprintf('\n╔════════════════════════════════════════════════════════════╗\n');
    fprintf('║           批量路径查询                                     ║\n');
    fprintf('╚════════════════════════════════════════════════════════════╝\n');
    
    % 均匀选择路径上的点
    indices = round(linspace(1, size(flightPath, 1), numPoints));
    
    for i = 1:length(indices)
        idx = indices(i);
        queryPos = flightPath(idx, :);
        
        fprintf('\n【查询点 %d/%d - 路径索引 %d】\n', i, numPoints, idx);
        comprehensiveQueryPheromone(queryPos, 3);
        
        % 添加分隔
        if i < length(indices)
            fprintf('\n');
            fprintf('════════════════════════════════════════════════════════════\n');
            pause(0.5);  % 短暂暂停以便查看
        end
    end
    
    fprintf('\n╔════════════════════════════════════════════════════════════╗\n');
    fprintf('║           批量查询完成                                     ║\n');
    fprintf('╚════════════════════════════════════════════════════════════╝\n\n');
end

% ═══════════════════════════════════════════════
% 比较查询函数 - 比较两个位置的信息素
% ═══════════════════════════════════════════════
function comparePositions(pos1, pos2)
    % 比较两个位置的信息素差异
    % 用法: comparePositions([30,30,20], [70,70,40])
    
    fprintf('\n╔════════════════════════════════════════════════════════════╗\n');
    fprintf('║           位置比较分析                                     ║\n');
    fprintf('╚════════════════════════════════════════════════════════════╝\n\n');
    
    fprintf('【位置 A: [%.1f, %.1f, %.1f]】\n', pos1(1), pos1(2), pos1(3));
    comprehensiveQueryPheromone(pos1, 3);
    
    fprintf('\n════════════════════════════════════════════════════════════\n\n');
    
    fprintf('【位置 B: [%.1f, %.1f, %.1f]】\n', pos2(1), pos2(2), pos2(3));
    comprehensiveQueryPheromone(pos2, 3);
    
    % 计算差异
    global pheromoneManager;
    if ~isempty(pheromoneManager)
        [val1, ~] = getPheromoneValue(pheromoneManager, pos1);
        [val2, ~] = getPheromoneValue(pheromoneManager, pos2);
        
        fprintf('\n╔════════════════════════════════════════════════════════════╗\n');
        fprintf('║           差异总结                                         ║\n');
        fprintf('╚════════════════════════════════════════════════════════════╝\n');
        fprintf('位置A信息素: %.4f\n', val1);
        fprintf('位置B信息素: %.4f\n', val2);
        fprintf('差值: %.4f\n', val2 - val1);
        fprintf('比率: %.2f%%\n', (val2/val1 - 1) * 100);
        
        if val2 > val1
            fprintf('结论: 位置B的信息素更高 (可能更适合飞行)\n');
        elseif val2 < val1
            fprintf('结论: 位置A的信息素更高 (可能更适合飞行)\n');
        else
            fprintf('结论: 两个位置信息素相同\n');
        end
    end
    
    fprintf('╚════════════════════════════════════════════════════════════╝\n\n');
end
function forceInitializeObstaclePheromones()
    global pheromoneManager globalObstacles;
    
    fprintf('\n🔄 强制重新初始化障碍物信息素...\n');
    
    % ✅ 修正：使用正确的方式获取网格维度
    dims = size(pheromoneManager.pheromoneMatrix);
    
    % 初始化三层矩阵
    tempAvoidMatrix = zeros(dims);      % 避障代价层(正值)
    tempPathMatrix = ones(dims) * pheromoneManager.initialPheromone;  % 路径吸引层(正值)
    
    totalObstacles = 0;
    
    % 1. 处理动态障碍物
    if isfield(globalObstacles, 'dynamic') && ~isempty(globalObstacles.dynamic)
        for i = 1:size(globalObstacles.dynamic, 1)
            obsPos = globalObstacles.dynamic(i, 1:3);
            obsRadius = globalObstacles.dynamic(i, 4);
            effectiveRadius = obsRadius * 3.0;
            
            centerIdx = worldToGrid(pheromoneManager, obsPos);
            radiusGrid = ceil(effectiveRadius / pheromoneManager.gridSize);
            depositCount = 0;
            
            for dx = -radiusGrid:radiusGrid
                for dy = -radiusGrid:radiusGrid
                    for dz = -radiusGrid:radiusGrid
                        idx = centerIdx + [dx, dy, dz];
                        if isValidGrid(pheromoneManager, idx)
                            dist = sqrt(dx^2 + dy^2 + dz^2) * pheromoneManager.gridSize;
                            if dist <= effectiveRadius
                                factor = exp(-dist^2/(2*(effectiveRadius/3.0)^2));
                                
                                % 避障层存储代价(正值表示危险程度)
                                avoidCost = 50.0 * factor;
                                tempAvoidMatrix(idx(1), idx(2), idx(3)) = ...
                                    max(tempAvoidMatrix(idx(1), idx(2), idx(3)), avoidCost);
                                
                                % 路径层大幅降低(表示不适合飞行)
                                reductionFactor = 1.0 - 0.95 * factor;  % 最多降低95%
                                tempPathMatrix(idx(1), idx(2), idx(3)) = ...
                                    tempPathMatrix(idx(1), idx(2), idx(3)) * reductionFactor;
                                
                                depositCount = depositCount + 1;
                            end
                        end
                    end
                end
            end
            
            fprintf('  ✅ 动态障碍物 #%d: 位置[%.1f, %.1f, %.1f], 沉积点数:%d\n', ...
                    i, obsPos(1), obsPos(2), obsPos(3), depositCount);
            totalObstacles = totalObstacles + 1;
        end
    end
    
    % 2. 处理建筑物
    if isfield(globalObstacles, 'buildings') && ~isempty(globalObstacles.buildings)
        for i = 1:size(globalObstacles.buildings, 1)
            x = globalObstacles.buildings(i, 1);
            y = globalObstacles.buildings(i, 2);
            width = globalObstacles.buildings(i, 3);
            depth = globalObstacles.buildings(i, 4);
            height = globalObstacles.buildings(i, 5);
            
            centerPos = [x, y, height/2];
            effectiveRadius = max(width, depth) * 0.8;
            
            centerIdx = worldToGrid(pheromoneManager, centerPos);
            radiusGrid = ceil(effectiveRadius / pheromoneManager.gridSize);
            
            for dx = -radiusGrid:radiusGrid
                for dy = -radiusGrid:radiusGrid
                    for dz = -radiusGrid:radiusGrid
                        idx = centerIdx + [dx, dy, dz];
                        if isValidGrid(pheromoneManager, idx)
                            dist = sqrt(dx^2 + dy^2 + dz^2) * pheromoneManager.gridSize;
                            if dist <= effectiveRadius
                                factor = exp(-dist^2/(2*(effectiveRadius/3.0)^2));
                                avoidCost = 40.0 * factor;
                                
                                tempAvoidMatrix(idx(1), idx(2), idx(3)) = ...
                                    max(tempAvoidMatrix(idx(1), idx(2), idx(3)), avoidCost);
                                
                                reductionFactor = 1.0 - 0.90 * factor;
                                tempPathMatrix(idx(1), idx(2), idx(3)) = ...
                                    tempPathMatrix(idx(1), idx(2), idx(3)) * reductionFactor;
                            end
                        end
                    end
                end
            end
            
            fprintf('  ✅ 建筑物 #%d: 中心[%.1f, %.1f]\n', i, x, y);
            totalObstacles = totalObstacles + 1;
        end
    end
    
    % 3. 处理树木
    if isfield(globalObstacles, 'trees') && ~isempty(globalObstacles.trees)
        for i = 1:size(globalObstacles.trees, 1)
            x = globalObstacles.trees(i, 1);
            y = globalObstacles.trees(i, 2);
            radius = globalObstacles.trees(i, 3);
            treeHeight = globalObstacles.trees(i, 4);
            
            treePos = [x, y, treeHeight/2];
            effectiveRadius = radius * 2.0;
            
            centerIdx = worldToGrid(pheromoneManager, treePos);
            radiusGrid = ceil(effectiveRadius / pheromoneManager.gridSize);
            
            for dx = -radiusGrid:radiusGrid
                for dy = -radiusGrid:radiusGrid
                    for dz = -radiusGrid:radiusGrid
                        idx = centerIdx + [dx, dy, dz];
                        if isValidGrid(pheromoneManager, idx)
                            dist = sqrt(dx^2 + dy^2 + dz^2) * pheromoneManager.gridSize;
                            if dist <= effectiveRadius
                                factor = exp(-dist^2/(2*(effectiveRadius/3.0)^2));
                                avoidCost = 30.0 * factor;
                                
                                tempAvoidMatrix(idx(1), idx(2), idx(3)) = ...
                                    max(tempAvoidMatrix(idx(1), idx(2), idx(3)), avoidCost);
                                
                                reductionFactor = 1.0 - 0.75 * factor;
                                tempPathMatrix(idx(1), idx(2), idx(3)) = ...
                                    tempPathMatrix(idx(1), idx(2), idx(3)) * reductionFactor;
                            end
                        end
                    end
                end
            end
            
            fprintf('  ✅ 树木 #%d: 位置[%.1f, %.1f]\n', i, x, y);
            totalObstacles = totalObstacles + 1;
        end
    end
    
    % 🔥 关键修改: 正确的主矩阵计算逻辑
    pheromoneManager.avoidancePheromones = tempAvoidMatrix;
    pheromoneManager.pathPheromones = tempPathMatrix;
    
    % 主矩阵 = 吸引力 - 排斥力
    % 负值表示障碍物区域,正值表示适合飞行区域
    pheromoneManager.pheromoneMatrix = tempPathMatrix - tempAvoidMatrix;
    
    % 确保值在合理范围(允许负值)
    pheromoneManager.pheromoneMatrix = max(-50, pheromoneManager.pheromoneMatrix);
    pheromoneManager.pheromoneMatrix = min(pheromoneManager.maxPheromone, ...
                                           pheromoneManager.pheromoneMatrix);
    
    % 统计信息
    fprintf('\n📊 信息素初始化统计:\n');
    fprintf('   总障碍物数量: %d\n', totalObstacles);
    fprintf('   避障层最大值: %.2f\n', max(tempAvoidMatrix(:)));
    fprintf('   路径层最小值: %.2f\n', min(tempPathMatrix(:)));
    fprintf('   主矩阵最大值: %.2f\n', max(pheromoneManager.pheromoneMatrix(:)));
    fprintf('   主矩阵最小值: %.2f (负值表示障碍物)\n', min(pheromoneManager.pheromoneMatrix(:)));
    fprintf('   主矩阵平均值: %.2f\n', mean(pheromoneManager.pheromoneMatrix(:)));
    
    fprintf('\n✅ 障碍物信息素初始化完成!\n\n');
end