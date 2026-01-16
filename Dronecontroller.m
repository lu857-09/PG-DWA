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