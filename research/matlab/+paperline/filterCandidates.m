function [safeCandidates, info] = filterCandidates(candidates, uavPosition, prediction, obstacles, cfg)
%filterCandidates Remove candidates that fail hard safety checks.

    safeMask = true(1, numel(candidates));
    reasons = strings(1, numel(candidates));
    margin = paperline.dynamicSafetyMargin(prediction, uavPosition, cfg);

    for i = 1:numel(candidates)
        point = candidates(i).point;
        clearance = paperline.minObstacleClearance(point, obstacles);
        segmentClearance = paperline.minSegmentObstacleClearance(uavPosition, point, obstacles);
        reachableDistance = cfg.uavMaxSpeed * cfg.predictionHorizon + cfg.minReachableMargin;

        if clearance <= margin
            safeMask(i) = false;
            reasons(i) = "point_clearance";
        elseif segmentClearance <= margin
            safeMask(i) = false;
            reasons(i) = "segment_clearance";
        elseif norm(point - uavPosition) > max(reachableDistance, cfg.searchRadius + cfg.farSafeDistance + cfg.sideOffset)
            safeMask(i) = false;
            reasons(i) = "reachability";
        else
            reasons(i) = "safe";
        end
    end

    safeCandidates = candidates(safeMask);
    info.safeMask = safeMask;
    info.reasons = reasons;
    info.margin = margin;
end
