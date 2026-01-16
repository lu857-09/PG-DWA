function smoothedPath = smoothPathPoints(path, obstacles)
    % 增强路径平滑处理
    smoothedPath = enhancedSmoothPath(path, obstacles, 0.6, 5);
end

function smoothedPath = enhancedSmoothPath(path, obstacles, smoothWeight, iterations)
    % 如果路径点太少，直接返回
    if size(path, 1) <= 2
        smoothedPath = path;
        return;
    end
    
    % 创建工作副本
    smoothedPath = path;
    
    % 定义平滑参数
    alpha = 0.1;  % 原路径权重 - 较小值使平滑更明显
    beta = 1.0 - alpha;  % 直线拉伸权重
    
    % 执行迭代平滑
    for iter = 1:iterations
        % 从第二个点到倒数第二个点进行平滑
        for i = 2:size(smoothedPath, 1)-1
            % 保存原始点
            oldPoint = smoothedPath(i, :);
            
            % 计算相邻点的中间位置 (直线拉伸)
            prev = smoothedPath(i-1, :);
            next = smoothedPath(i+1, :);
            
            % 直线拉伸 - 将点向相邻点的连线移动
            linePoint = (prev + next) / 2;
            
            % 加权平均原点和直线点
            newPoint = oldPoint * alpha + linePoint * beta;
            
            % 检查新点是否会导致碰撞
            if ~checkCollision(newPoint, obstacles)
                smoothedPath(i, :) = newPoint;
            end
        end
    end
end