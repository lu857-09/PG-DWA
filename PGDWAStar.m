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