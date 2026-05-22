function [visibility, parts] = computeVisibility(targetPosition, uavPosition, cameraHeading, obstacles, cfg)
%computeVisibility Evaluate 2-D target visibility from a UAV viewpoint.

    targetVector = targetPosition - uavPosition;
    targetDistance = norm(targetVector);

    parts.targetDistance = targetDistance;
    parts.bearingError = 0.0;
    parts.fovScore = 1.0;
    parts.distanceScore = 1.0;
    parts.occlusionRisk = 0.0;

    if targetDistance < 1e-9
        visibility = 1.0;
        return
    end

    if targetDistance > cfg.maxVisibleDistance
        parts.distanceScore = 0.0;
        visibility = 0.0;
        return
    end

    cameraHeading = normalizeVector(cameraHeading, targetVector);
    targetDirection = targetVector / targetDistance;
    parts.bearingError = acos(max(-1, min(1, dot(cameraHeading, targetDirection))));
    parts.fovScore = max(0, 1 - max(0, parts.bearingError - 0.55 * cfg.fovHalfAngle) / ...
        (0.45 * cfg.fovHalfAngle));
    parts.distanceScore = max(0, 1 - max(0, targetDistance - cfg.fullVisibilityDistance) / ...
        max(eps, cfg.maxVisibleDistance - cfg.fullVisibilityDistance));
    parts.occlusionRisk = paperline.lineOfSightRisk(uavPosition, targetPosition, obstacles, cfg);

    visibility = max(0, min(1, 0.65 * parts.fovScore + 0.35 * parts.distanceScore ...
        - cfg.visibilityOcclusionPenalty * parts.occlusionRisk));
end

function out = normalizeVector(vector, fallback)
    if norm(vector) < 1e-9
        vector = fallback;
    end

    if norm(vector) < 1e-9
        vector = [1 0];
    end

    out = vector / norm(vector);
end
