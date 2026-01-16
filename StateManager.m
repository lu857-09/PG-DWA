%% 初始化无人机参数
% 无人机外形参数 
droneSize = 1.5;  
[sx, sy, sz] = sphere(10);

% 初始化无人机状态
if loadPreviousState && loaded && ~restartFlight
    droneState = loadedDroneState;
    finalPath = loadedPath;
    energy = loadedEnergy;
    simulationTime = loadedTime;
    disp('已还原无人机状态');
else
    % 初始位置、姿态和速度
    droneState = [start, 0, 0, 0, 0, 0, 0]; % [x, y, z, yaw, pitch, roll, vx, vy, vz]
    
    % 计算初始路径方向并设置无人机朝向和初始速度
    if size(flightPath, 1) >= 2
        initialDir = flightPath(2,:) - flightPath(1,:);
        if norm(initialDir) > 0
            initialDir = initialDir / norm(initialDir);
            initialYaw = atan2(initialDir(2), initialDir(1));
            initialPitch = asin(max(-1, min(1, initialDir(3))));
            
            % 更新无人机的初始朝向
            droneState(4) = initialYaw;  % 设置初始航向
            droneState(5) = initialPitch; % 设置初始俯仰角
            
            % 设置初始速度向量与朝向一致
            initialSpeed = maxSpeed * 0.3; % 设置初始速度为最大速度的30%
            droneState(7) = initialSpeed * cos(initialYaw) * cos(initialPitch); % vx
            droneState(8) = initialSpeed * sin(initialYaw) * cos(initialPitch); % vy
            droneState(9) = initialSpeed * sin(initialPitch); % vz
        end
    end
    
    finalPath = start;
    energy = initialEnergy;
    simulationTime = 0;
    disp('初始化无人机状态');
end

% 绘制初始位置的无人机
droneHandle = surf(droneSize/2*sx + droneState(1), droneSize/2*sy + droneState(2), droneSize/2*sz + droneState(3), ...
              'FaceColor', 'b', 'EdgeColor', 'none', 'FaceAlpha', 0.8);
headingLength = 5;
headingX = [droneState(1), droneState(1) + headingLength * cos(droneState(4)) * cos(droneState(5))];
headingY = [droneState(2), droneState(2) + headingLength * sin(droneState(4)) * cos(droneState(5))];
headingZ = [droneState(3), droneState(3) + headingLength * sin(droneState(5))];
headingLine = plot3(headingX, headingY, headingZ, 'r-', 'LineWidth', 2);

% 绘制路径轨迹 - 使用明显的绿色
pathHandle = [];

% 绘制投影图
xyHandle = [];
xzHandle = [];
yzHandle = [];

% 绘制状态图表
altHandle = [];
velHandle = [];
engHandle = [];
vzHandle = [];
omegaHandle = [];

% 设置当前目标点
currentGoalIdx = 2;
if currentGoalIdx <= size(flightPath, 1)
    currentTargetHandle = plot3(flightPath(currentGoalIdx,1), flightPath(currentGoalIdx,2), flightPath(currentGoalIdx,3), ...
                           'mo', 'MarkerSize', 6, 'MarkerFaceColor', 'm');
else
    currentTargetHandle = [];
end

% 初始化飞行统计
pathLength = 0;
averageSpeed = 0;
totalFlightTime = 0;
avoidanceEventCount = 0;
totalAvoidanceTime = 0;
avgAvoidanceTime = 0;
pathDeviationCount = 0;
visualCounter = 0;
lastPathAdvanceTime = 0;
lastPathDeviationTime = 0;
lastAvoidanceInfo = struct('active', false, 'direction', [0,0,0], 'distance', 0, 'text', '');
isDeviated = false;
maxAltitude = droneState(3);

% 历史数据
altitudeHistory = droneState(3);
velocityHistory = 0;
timeHistory = 0;
yawHistory = droneState(4);
omegaHistory = 0;
vzHistory = 0;
energyHistory = (energy/initialEnergy)*100;

% 设置初始标志
reachedFinalGoal = false;