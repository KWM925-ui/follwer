function clearance = minSegmentObstacleClearance(segmentA, segmentB, obstacles)
%minSegmentObstacleClearance Compute segment clearance to circular obstacles.

    clearance = inf;

    for i = 1:numel(obstacles)
        distance = pointToSegmentDistance(obstacles(i).center, segmentA, segmentB);
        clearance = min(clearance, distance - obstacles(i).radius);
    end
end

function distance = pointToSegmentDistance(point, segmentA, segmentB)
    ab = segmentB - segmentA;
    if norm(ab) < 1e-12
        distance = norm(point - segmentA);
        return
    end

    t = dot(point - segmentA, ab) / dot(ab, ab);
    t = max(0.0, min(1.0, t));
    projection = segmentA + t * ab;
    distance = norm(point - projection);
end
