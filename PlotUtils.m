function plotBuilding(position, width, depth, height, color, alpha)
    % 绘制建筑物
    if nargin < 6
        alpha = 0.7;
    end

    % 顶点坐标
    vertices = [
        0, 0, 0;
        width, 0, 0;
        width, depth, 0;
        0, depth, 0;
        0, 0, height;
        width, 0, height;
        width, depth, height;
        0, depth, height
    ];

    % 移动到指定位置
    vertices = vertices + repmat(position, 8, 1);

    % 定义面
    faces = [
        1, 2, 6, 5;    % 前面
        2, 3, 7, 6;    % 右面
        3, 4, 8, 7;    % 后面
        4, 1, 5, 8;    % 左面
        5, 6, 7, 8;    % 顶面
        1, 2, 3, 4     % 底面
    ];

    % 绘制建筑物
    patch('Vertices', vertices, 'Faces', faces, ...
          'FaceColor', color, 'EdgeColor', 'k', 'FaceAlpha', alpha);
end

function handle = plotCube(position, width, depth, height, color, alpha, tag)
    % 绘制立方体函数
    if nargin < 6
        alpha = 0.7;
    end
    if nargin < 7
        tag = '';
    end

    % 确保颜色值在有效范围内
    if ~isempty(color) && isnumeric(color) && length(color) == 3
        % 确保RGB值在0-1范围内
        color = max(0, min(1, color));
    else
        % 默认颜色
        color = [0.7, 0.7, 0.7];
    end

    % 顶点坐标
    vertices = [
        0, 0, 0;
        width, 0, 0;
        width, depth, 0;
        0, depth, 0;
        0, 0, height;
        width, 0, height;
        width, depth, height;
        0, depth, height
    ];

    % 移动到指定位置
    vertices = vertices + repmat(position, 8, 1);

    % 定义面
    faces = [
        1, 2, 6, 5;    % 前面
        2, 3, 7, 6;    % 右面
        3, 4, 8, 7;    % 后面
        4, 1, 5, 8;    % 左面
        5, 6, 7, 8;    % 顶面
        1, 2, 3, 4     % 底面
    ];

    % 绘制立方体
    handle = patch('Vertices', vertices, 'Faces', faces, ...
          'FaceColor', color, 'EdgeColor', 'k', 'FaceAlpha', alpha, 'Tag', tag);
end