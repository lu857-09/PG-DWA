%% 优先队列实现
function queue = createPriorityQueue()
    % 创建优先队列
    queue = struct();
    queue.keys = {};
    queue.priorities = [];
    queue.map = containers.Map('KeyType', 'char', 'ValueType', 'double');
    queue.size = 0;
end

function queue = queueInsert(queue, key, priority)
    % 向队列中插入元素
    if queueContains(queue, key)
        queue = queueUpdatePriority(queue, key, priority);
        return;
    end

    queue.size = queue.size + 1;
    queue.keys{queue.size} = key;
    queue.priorities(queue.size) = priority;
    queue.map(key) = queue.size;

    % 执行上浮操作
    queue = queueBubbleUp(queue, queue.size);
end

function [queue, key] = queuePop(queue)
    % 弹出最高优先级元素
    key = '';
    if queueIsEmpty(queue)
        return;
    end

    key = queue.keys{1};
    remove(queue.map, key);

    if queue.size > 1
        queue.keys{1} = queue.keys{queue.size};
        queue.priorities(1) = queue.priorities(queue.size);
        queue.map(queue.keys{1}) = 1;
    end

    queue.keys(queue.size) = [];
    queue.priorities(queue.size) = [];
    queue.size = queue.size - 1;

    if queue.size > 0
        queue = queueBubbleDown(queue, 1);
    end
end