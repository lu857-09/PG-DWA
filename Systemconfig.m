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
pheromoneParams.pathLayerWeight = 1.2;      % 路径层权重 - 增强路径层影响
pheromoneParams.avoidanceLayerWeight = 2.8; % 避障层权重 - 增强避障层影响
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