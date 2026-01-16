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