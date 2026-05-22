function candidates = generateCandidates(prediction, uavPosition, fsmState, cfg)
%generateCandidates Generate sparse follow-viewpoint candidates.

    if isfield(prediction, "viewpointReference")
        p = prediction.viewpointReference;
    else
        p = prediction.position;
    end
    velocity = prediction.velocity;

    if norm(velocity) < 1e-6
        direction = p - uavPosition;
        if norm(direction) < 1e-6
            direction = [1 0];
        end
    else
        direction = velocity;
    end

    eForward = direction / norm(direction);
    eLeft = [-eForward(2) eForward(1)];

    candidateNames = ["behind" "left" "right" "far_safe"];
    candidatePoints = [
        p - cfg.desiredDistance * eForward
        p - cfg.desiredDistance * eForward + cfg.lateralOffset * eLeft
        p - cfg.desiredDistance * eForward - cfg.lateralOffset * eLeft
        p - cfg.farSafeDistance * eForward
    ];

    if string(fsmState) == "SEARCH_SAFE_VIEWPOINT"
        angles = cfg.searchFanAngles;
        candidateNames = strings(1, numel(angles));
        candidatePoints = zeros(numel(angles), 2);
        for i = 1:numel(angles)
            searchDirection = rotateVector(eForward, angles(i));
            candidateNames(i) = "search_reacquire_" + string(i);
            candidatePoints(i, :) = p - cfg.searchRadius * searchDirection;
        end
    end

    candidates = repmat(struct("name", "", "point", [0 0]), 1, numel(candidateNames));
    for i = 1:numel(candidateNames)
        candidates(i).name = candidateNames(i);
        candidates(i).point = candidatePoints(i, :);
        candidates(i).heading = normalizeVector(p - candidatePoints(i, :), eForward);
    end
end

function out = rotateVector(vector, angle)
    rotation = [cos(angle) -sin(angle); sin(angle) cos(angle)];
    out = (rotation * vector(:)).';
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
