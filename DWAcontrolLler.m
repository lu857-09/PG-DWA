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
