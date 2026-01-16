%% 主仿真循环
disp('开始飞行仿真...');

while energy > 0 && simulationTime < 300 && ~reachedFinalGoal
    % 检查暂停状态
    if isPaused
        pause(0.1);
        continue;
    end
    
   % 使用全局变量代替 persistent
    
if isempty(pheromoneUpdateCounter)
    pheromoneUpdateCounter = 0;
end
pheromoneUpdateCounter = pheromoneUpdateCounter + 1;

% 每5个循环周期执行一次（约1秒）
if mod(pheromoneUpdateCounter, 5) == 0
    % 1. 蒸发
    evapRate = pheromoneManager.evaporationRate * 0.2;
    
    
    pheromoneManager.pathPheromones = max(pheromoneManager.initialPheromone, ...
        pheromoneManager.pathPheromones * (1 - evapRate * 0.8));
    
    pheromoneManager.energyPheromones = max(pheromoneManager.initialPheromone, ...
        pheromoneManager.energyPheromones * (1 - evapRate * 0.7));
    
    % 2.实时更新动态障碍物的避障信息素
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
    
    % 3. 在当前位置沉积信息素
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
    
    % 4. 扩散
    if pheromoneManager.diffusionRate > 0 && mod(pheromoneUpdateCounter, 25) == 0
        kernel = ones(3,3,3) / 27;
        diffused = convn(pheromoneManager.pathPheromones, kernel, 'same');
        pheromoneManager.pathPheromones = pheromoneManager.pathPheromones * (1 - pheromoneManager.diffusionRate * 0.5) + ...
            diffused * (pheromoneManager.diffusionRate * 0.5);
    end
    
    % 5.更新主矩阵 = 路径层 - 避障层
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