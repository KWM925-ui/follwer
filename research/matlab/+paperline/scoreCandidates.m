function selected = scoreCandidates(candidates, uavPosition, prediction, obstacles, lastName, cfg)
%scoreCandidates Score safe candidates and return the best one.

    scores = zeros(1, numel(candidates));
    occlusionRisks = zeros(1, numel(candidates));

    for i = 1:numel(candidates)
        point = candidates(i).point;
        distanceError = abs(norm(point - prediction.position) - cfg.desiredDistance);
        viewVector = prediction.position - point;
        targetDirection = normalized(prediction.velocity, prediction.position - uavPosition);
        angleError = acos(max(-1, min(1, dot(normalized(viewVector, [1 0]), targetDirection))));
        clearance = paperline.minObstacleClearance(point, obstacles);
        occlusion = paperline.lineOfSightRisk(point, prediction.position, obstacles, cfg);
        motionCost = norm(point - uavPosition);
        if ~isfield(cfg, "useSwitchPenalty") || cfg.useSwitchPenalty
            switchPenalty = double(string(candidates(i).name) ~= string(lastName) && strlength(string(lastName)) > 0);
        else
            switchPenalty = 0.0;
        end
        uncertainty = trace(prediction.covariance(1:2, 1:2));

        sDistance = exp(-distanceError / cfg.scoreScales.distance);
        sAngle = exp(-angleError / cfg.scoreScales.angle);
        sClearance = min(1.0, clearance / cfg.scoreScales.clearance);
        if ~isfield(cfg, "useOcclusionScore") || cfg.useOcclusionScore
            sOcclusion = 1.0 - occlusion;
        else
            sOcclusion = 1.0;
        end
        sMotion = exp(-motionCost / cfg.scoreScales.motion);
        if ~isfield(cfg, "useVisibilityScore") || cfg.useVisibilityScore
            [candidateVisibility, ~] = paperline.computeVisibility(prediction.position, ...
                point, candidates(i).heading, obstacles, cfg);
            sVisibility = 0.5 * prediction.confidence + 0.5 * candidateVisibility;
        else
            sVisibility = 1.0;
        end
        sSwitch = 1.0 - switchPenalty;
        sUncertainty = exp(-uncertainty / cfg.scoreScales.uncertainty);

        scores(i) = cfg.weights.distance * sDistance ...
            + cfg.weights.angle * sAngle ...
            + cfg.weights.clearance * sClearance ...
            + cfg.weights.occlusion * sOcclusion ...
            + cfg.weights.motion * sMotion ...
            + cfg.weights.visibility * sVisibility ...
            + cfg.weights.switching * sSwitch ...
            + cfg.weights.uncertainty * sUncertainty;
        occlusionRisks(i) = occlusion;
    end

    [bestScore, bestIndex] = max(scores);
    selected = candidates(bestIndex);
    selected.score = bestScore;
    selected.occlusionRisk = occlusionRisks(bestIndex);
    selected.allScores = scores;
end

function out = normalized(vector, fallback)
    if norm(vector) < 1e-9
        vector = fallback;
    end

    if norm(vector) < 1e-9
        vector = [1 0];
    end

    out = vector / norm(vector);
end
