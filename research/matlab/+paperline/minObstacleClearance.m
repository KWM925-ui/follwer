function clearance = minObstacleClearance(point, obstacles)
%minObstacleClearance Compute point clearance to circular obstacles.

    clearance = inf;

    for i = 1:numel(obstacles)
        clearance = min(clearance, norm(point - obstacles(i).center) - obstacles(i).radius);
    end
end
