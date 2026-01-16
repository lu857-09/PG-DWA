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
