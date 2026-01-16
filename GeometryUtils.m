function segments = calculatePathSegmentInfo(path)
    % 预计算路径段信息，用于计算曲率
    segments = struct('startIdx', {}, 'endIdx', {}, 'length', {}, 'curvature', {}, 'direction', {});

    % 路径段至少需要两个点
    if size(path, 1) < 2
        return;
    end

    % 计算每段路径的信息
    for i = 1:size(path, 1)-1
        segment = struct();
        segment.startIdx = i;
        segment.endIdx = i+1;
        
        % 计算路径段长度和方向
        segment.direction = path(i+1,:) - path(i,:);
        segment.length = norm(segment.direction);
        
        if segment.length > 0
            segment.direction = segment.direction / segment.length; % 归一化
        end
        
        % 计算曲率 - 如果有足够的点
        if i > 1 && i < size(path, 1)-1
            % 使用相邻三个点估计曲率
            p1 = path(i-1,:);
            p2 = path(i,:);
            p3 = path(i+1,:);
            
            % 计算两个方向向量
            v1 = p2 - p1;
            v2 = p3 - p2;
            
            % 向量夹角的变化率可以反映曲率
            v1_norm = norm(v1);
            v2_norm = norm(v2);
            
            if v1_norm > 0 && v2_norm > 0
                v1 = v1 / v1_norm;
                v2 = v2 / v2_norm;
                dot_product = dot(v1, v2);
                % 防止数值误差导致的问题
                dot_product = min(1, max(-1, dot_product));
                angle = acos(dot_product);
                
                % 曲率与转弯角度和路径长度有关
                segment.curvature = angle / segment.length;
            else
                segment.curvature = 0;
            end
        else
            segment.curvature = 0;
        end
        
        segments(end+1) = segment;
    end
end