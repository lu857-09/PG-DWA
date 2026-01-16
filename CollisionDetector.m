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