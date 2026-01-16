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