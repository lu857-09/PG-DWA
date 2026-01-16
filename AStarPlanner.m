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