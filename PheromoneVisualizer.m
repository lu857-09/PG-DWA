function visualizePheromoneSlices(manager, figHandle, queryPos)
    % 信息素可视化函数 - 支持实时交互式查询
    % queryPos: 可选参数,指定要查询的世界坐标 [x, y, z]
    
    if nargin < 2 || ~ishandle(figHandle); figHandle = gcf; end
    if nargin < 3; queryPos = []; end

    figure(figHandle);
    clf(figHandle);

    combinedPheromones = manager.pheromoneMatrix;
    
    % 获取数据维度
    dims = size(combinedPheromones);

    % 如果提供了查询点,使用该点;否则使用地图中心
    if ~isempty(queryPos)
        % 转换世界坐标到网格索引
        gridIdx = ceil(queryPos / manager.gridSize);
        % 确保索引在有效范围内
        gridIdx = max([1,1,1], min(gridIdx, dims));
        midX = gridIdx(1);
        midY = gridIdx(2);
        midZ = gridIdx(3);
        titleSuffix = sprintf(' (查询点: [%.1f, %.1f, %.1f])', queryPos(1), queryPos(2), queryPos(3));
    else
        midX = ceil(dims(1)/2);
        midY = ceil(dims(2)/2);
        midZ = ceil(dims(3)/2);
        titleSuffix = ' (地图中心)';
    end

    % 计算整个信息素矩阵的最小值和最大值
    minVal = min(combinedPheromones(:));
    maxVal = max(combinedPheromones(:));
    
    % 如果所有值都相同，设置一个小的范围
    if minVal == maxVal
        minVal = minVal - 0.1;
        maxVal = maxVal + 0.1;
    end
    
    % 创建统一的颜色映射
    colormap(jet);
    
    % 创建子图 - 3个主切片,1个3D视图
    subplot(2, 2, 1); % XY平面切片
    xySlice = squeeze(combinedPheromones(:, :, midZ));
    imagesc([0 manager.mapSize(1)], [0 manager.mapSize(2)], xySlice);
    
    caxis([minVal, maxVal]);
    
    title(sprintf('XY平面信息素分布 (Z=%d)%s', midZ*manager.gridSize, titleSuffix));
    xlabel('X轴'); ylabel('Y轴');
    axis([0 manager.mapSize(1) 0 manager.mapSize(2)]);
    axis manual;
    set(gca, 'YDir', 'normal');

    % 标记查询点
    if ~isempty(queryPos)
        hold on;
        plot(queryPos(1), queryPos(2), 'w*', 'MarkerSize', 15, 'LineWidth', 2);
        plot(queryPos(1), queryPos(2), 'ko', 'MarkerSize', 8, 'LineWidth', 2);
        hold off;
    end

    % 添加 XY平面上的路径投影
    global originalPath finalPath
    hold on;
    if ~isempty(originalPath) 
        plot(originalPath(:,1), originalPath(:,2), 'k-', 'LineWidth', 1.5);
    end
    if ~isempty(finalPath)
        plot(finalPath(:,1), finalPath(:,2), 'w-', 'LineWidth', 1.5);
    end
    hold off;

    subplot(2, 2, 2); % XZ平面切片
    xzSlice = squeeze(combinedPheromones(:, midY, :));
    imagesc([0 manager.mapSize(1)], [0 manager.mapSize(3)], xzSlice');
    
    caxis([minVal, maxVal]);
    
    title(sprintf('XZ平面信息素分布 (Y=%d)%s', midY*manager.gridSize, titleSuffix));
    xlabel('X轴'); ylabel('Z轴');
    axis([0 manager.mapSize(1) 0 manager.mapSize(3)]);
    axis manual;
    set(gca, 'YDir', 'normal');

    % 标记查询点
    if ~isempty(queryPos)
        hold on;
        plot(queryPos(1), queryPos(3), 'w*', 'MarkerSize', 15, 'LineWidth', 2);
        plot(queryPos(1), queryPos(3), 'ko', 'MarkerSize', 8, 'LineWidth', 2);
        hold off;
    end

    % 添加 XZ平面上的路径投影
    hold on;
    if ~isempty(originalPath)
        plot(originalPath(:,1), originalPath(:,3), 'k-', 'LineWidth', 1.5);
    end
    if ~isempty(finalPath)
        plot(finalPath(:,1), finalPath(:,3), 'w-', 'LineWidth', 1.5);
    end
    hold off;

    subplot(2, 2, 3); % YZ平面切片
    yzSlice = squeeze(combinedPheromones(midX, :, :));
    imagesc([0 manager.mapSize(2)], [0 manager.mapSize(3)], yzSlice');
    
    caxis([minVal, maxVal]);
    
    title(sprintf('YZ平面信息素分布 (X=%d)%s', midX*manager.gridSize, titleSuffix));
    xlabel('Y轴'); ylabel('Z轴');
    axis([0 manager.mapSize(2) 0 manager.mapSize(3)]);
    axis manual;
    set(gca, 'YDir', 'normal');

    % 标记查询点
    if ~isempty(queryPos)
        hold on;
        plot(queryPos(2), queryPos(3), 'w*', 'MarkerSize', 15, 'LineWidth', 2);
        plot(queryPos(2), queryPos(3), 'ko', 'MarkerSize', 8, 'LineWidth', 2);
        hold off;
    end

    % 添加 YZ平面上的路径投影
    hold on;
    if ~isempty(originalPath)
        plot(originalPath(:,2), originalPath(:,3), 'k-', 'LineWidth', 1.5);
    end
    if ~isempty(finalPath)
        plot(finalPath(:,2), finalPath(:,3), 'w-', 'LineWidth', 1.5);
    end
    hold off;

    % 调整子图位置，为颜色条腾出空间
    subplot(2, 2, 4);
    pos = get(gca, 'Position');
    
    % 创建颜色条，放在右侧
    c = colorbar('Location', 'eastoutside');
    c.Label.String = '信息素浓度';
    c.Label.FontSize = 12;
    c.Label.FontWeight = 'bold';
    
    % 设置颜色条的范围
    caxis([minVal, maxVal]);
    
    % 恢复子图位置
    set(gca, 'Position', pos);

    % 可选的3D视图
    % 创建世界坐标网格
    [X, Y, Z] = meshgrid(1:dims(2), 1:dims(1), 1:dims(3));
    X = X * manager.gridSize;
    Y = Y * manager.gridSize;
    Z = Z * manager.gridSize;

    % 只显示高于阈值的值 - 使用等值面
    meanVal = mean(combinedPheromones(:));
    stdVal = std(combinedPheromones(:));

    % 定义高阈值
    threshold = meanVal + 0.6 * stdVal;

    % 绘制等值面
    p = patch(isosurface(X, Y, Z, combinedPheromones, threshold));
    isonormals(X, Y, Z, combinedPheromones, p);
    set(p, 'FaceColor', 'red', 'EdgeColor', 'none', 'FaceAlpha', 0.5);

    % 添加路径
    hold on;
    if ~isempty(originalPath)
        plot3(originalPath(:,1), originalPath(:,2), originalPath(:,3), 'k-', 'LineWidth', 1.5);
    end
    if ~isempty(finalPath)
        plot3(finalPath(:,1), finalPath(:,2), finalPath(:,3), 'w-', 'LineWidth', 1.5);
    end

    % 标记查询点
    if ~isempty(queryPos)
        plot3(queryPos(1), queryPos(2), queryPos(3), 'w*', 'MarkerSize', 20, 'LineWidth', 3);
        plot3(queryPos(1), queryPos(2), queryPos(3), 'ko', 'MarkerSize', 10, 'LineWidth', 2);
        
        % 添加查询点标签
        text(queryPos(1), queryPos(2), queryPos(3)+5, '查询点', ...
            'Color', 'w', 'FontWeight', 'bold', 'FontSize', 12, ...
            'HorizontalAlignment', 'center', 'BackgroundColor', [0 0 0 0.5]);
    end

    title(sprintf('3D信息素分布 (红色 = 高浓度区域)%s', titleSuffix));
    xlabel('X轴'); ylabel('Y轴'); zlabel('Z轴');
    axis([0 manager.mapSize(1) 0 manager.mapSize(2) 0 manager.mapSize(3)]);
    grid on;
    view(3);
    camlight;
    lighting gouraud;

    % 添加信息素统计信息
    annotation('textbox', [0.1, 0.01, 0.8, 0.04], 'String', ...
        sprintf('信息素统计: 最小=%.2f, 最大=%.2f, 平均=%.2f, 标准差=%.2f', ...
                minVal, maxVal, meanVal, stdVal), ...
        'FontSize', 9, 'FitBoxToText', 'on', 'BackgroundColor', [1 1 1 0.7], ...
        'HorizontalAlignment', 'center');
    
    % 设置窗口的ButtonDownFcn
    set(figHandle, 'WindowButtonDownFcn', @(src, event) clickQueryCallback(src, event, manager));
    
    % 添加说明文本
    annotation('textbox', [0.02, 0.95, 0.3, 0.04], 'String', ...
        '💡 点击任意切片图查询该位置信息素', ...
        'FontSize', 10, 'FontWeight', 'bold', 'EdgeColor', 'none', ...
        'BackgroundColor', [1 1 0.8 0.8], 'HorizontalAlignment', 'left');
end