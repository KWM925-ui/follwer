function risk = lineOfSightRisk(viewpoint, targetPoint, obstacles, cfg)
%lineOfSightRisk Return 2-D line-of-sight occlusion risk in [0, 1].

    risk = 0.0;

    for i = 1:numel(obstacles)
        center = obstacles(i).center;
        radius = obstacles(i).radius + cfg.bodyRadius;
        distance = pointToSegmentDistance(center, viewpoint, targetPoint);

        if distance <= radius
            risk = 1.0;
            return
        end

        nearDistance = radius + cfg.occlusionNearDistance;
        if distance < nearDistance
            risk = max(risk, 1.0 - (distance - radius) / cfg.occlusionNearDistance);
        end
    end

    risk = max(0.0, min(1.0, risk));
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
