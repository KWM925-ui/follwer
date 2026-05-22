function result = simulateRun(scenario, cfg)
%simulateRun Run the decision-layer prototype on one scenario.

    time = scenario.time;
    n = numel(time);

    uav = zeros(n, 2);
    uav(1, :) = scenario.initialUav;

    targetEstimate = zeros(n, 2);
    predictedTarget = zeros(n, 2);
    covarianceTrace = zeros(n, 1);
    visibility = zeros(n, 1);
    occlusionRisk = zeros(n, 1);
    selectedPoint = nan(n, 2);
    selectedName = strings(n, 1);
    stateName = strings(n, 1);
    scoreBest = nan(n, 1);
    numSafeCandidates = zeros(n, 1);
    plannerHealthy = true(n, 1);
    plannerCommandHealthy = true(n, 1);
    cameraHeading = zeros(n, 2);
    cameraHeading(1, :) = normalizeVector(scenario.target(1, :) - uav(1, :), [1 0]);
    reportedPlannerHealthy = true;
    blacklistedCandidate = strings(n, 1);
    blacklistActive = false(n, 1);
    failedCandidateName = "";
    failedCandidatePoint = [NaN NaN];
    blacklistCountdown = 0;
    consecutivePlannerFailures = 0;

    kf = initializeFilter(scenario.target(1, :), cfg);
    fsm = paperline.updateFsm("FOLLOW", struct("reset", true), cfg);
    lastCandidateName = "";

    for k = 1:n
        plannerHealthy(k) = reportedPlannerHealthy;
        measurement = makeMeasurement(scenario, k, uav(k, :), cameraHeading(k, :), cfg);
        visibility(k) = measurement.confidence;

        [prediction, kf] = paperline.predictTarget(kf, measurement, cfg);
        targetEstimate(k, :) = prediction.estimate;
        predictedTarget(k, :) = prediction.position;
        covarianceTrace(k) = trace(prediction.covariance(1:2, 1:2));

        context.visible = measurement.confidence >= cfg.visibilityThreshold;
        context.covarianceTrace = covarianceTrace(k);
        context.plannerHealthy = plannerHealthy(k) ...
            && consecutivePlannerFailures < cfg.plannerFailureHoldFrames ...
            && blacklistCountdown <= 0;
        context.hasSafeCandidate = true;
        fsm = paperline.updateFsm(fsm.state, context, cfg);

        decisionPrediction = prediction;
        decisionPrediction.viewpointReference = prediction.estimate ...
            + cfg.viewpointPredictionBlend * (prediction.position - prediction.estimate);
        candidates = paperline.generateCandidates(decisionPrediction, uav(k, :), fsm.state, cfg);
        [safeCandidates, safetyInfo] = paperline.filterCandidates(candidates, uav(k, :), ...
            prediction, scenario.obstacles, cfg);
        [safeCandidates, blacklistInfo] = applyCandidateBlacklist(safeCandidates, ...
            failedCandidateName, failedCandidatePoint, blacklistCountdown, cfg);
        blacklistActive(k) = blacklistInfo.active;
        blacklistedCandidate(k) = blacklistInfo.name;
        numSafeCandidates(k) = numel(safeCandidates);

        if fsm.state == "FAILSAFE"
            selected = makeHoldSelection("failsafe", uav(k, :), NaN, 1);
        elseif isempty(safeCandidates)
            context.hasSafeCandidate = false;
            fsm = paperline.updateFsm(fsm.state, context, cfg);
            selected = makeHoldSelection("none", uav(k, :), NaN, 1);
        elseif fsm.state == "HOLD_SAFE"
            selected = makeHoldSelection("hold_safe", uav(k, :), NaN, 1);
        else
            selected = selectCandidate(safeCandidates, uav(k, :), prediction, ...
                scenario.obstacles, lastCandidateName, cfg);
        end

        selectedPoint(k, :) = selected.point;
        selectedName(k) = selected.name;
        scoreBest(k) = selected.score;
        occlusionRisk(k) = selected.occlusionRisk;
        stateName(k) = fsm.state;
        lastCandidateName = selected.name;
        plannerCommandHealthy(k) = computePlannerCommandHealth(scenario, k, ...
            uav(k, :), selected.point, selected.name, cfg);
        [failedCandidateName, failedCandidatePoint, blacklistCountdown] = updateCandidateBlacklist( ...
            selected, plannerCommandHealthy(k), failedCandidateName, failedCandidatePoint, ...
            blacklistCountdown, cfg);
        if plannerCommandHealthy(k)
            consecutivePlannerFailures = 0;
        else
            consecutivePlannerFailures = consecutivePlannerFailures + 1;
        end

        if k < n
            if plannerCommandHealthy(k)
                uav(k + 1, :) = updateUav(uav(k, :), selected.point, cfg);
            else
                uav(k + 1, :) = uav(k, :);
            end
            cameraHeading(k + 1, :) = updateCameraHeading(cameraHeading(k, :), selected, ...
                prediction.position - uav(k, :), cfg);
            reportedPlannerHealthy = plannerCommandHealthy(k);
        end

        result.debug(k).candidates = candidates;
        result.debug(k).safeCandidates = safeCandidates;
        result.debug(k).safetyInfo = safetyInfo;
        result.debug(k).blacklistInfo = blacklistInfo;
    end

    result.time = time;
    result.scenario = scenario;
    result.uav = uav;
    result.targetEstimate = targetEstimate;
    result.predictedTarget = predictedTarget;
    result.covarianceTrace = covarianceTrace;
    result.visibility = visibility;
    result.occlusionRisk = occlusionRisk;
    result.selectedPoint = selectedPoint;
    result.selectedName = selectedName;
    result.stateName = stateName;
    result.scoreBest = scoreBest;
    result.numSafeCandidates = numSafeCandidates;
    result.plannerHealthy = plannerHealthy;
    result.plannerCommandHealthy = plannerCommandHealthy;
    result.cameraHeading = cameraHeading;
    result.blacklistActive = blacklistActive;
    result.blacklistedCandidate = blacklistedCandidate;
end

function kf = initializeFilter(initialTarget, cfg)
    kf.x = [initialTarget(1); initialTarget(2); 0; 0];
    kf.P = eye(4) * cfg.initialCovariance;
    kf.xCa = [initialTarget(1); initialTarget(2); 0; 0; 0; 0];
    kf.PCa = eye(6) * cfg.initialCovariance;
    kf.acceleration = [0; 0];
end

function measurement = makeMeasurement(scenario, k, uavPosition, cameraHeading, cfg)
    targetPosition = scenario.target(k, :);
    geometricVisibility = paperline.computeVisibility(targetPosition, uavPosition, ...
        cameraHeading, scenario.obstacles, cfg);

    if scenario.dropout(k)
        measurement.available = false;
        measurement.position = [NaN NaN];
        measurement.confidence = geometricVisibility * 0.35;
        return
    end

    noise = cfg.measurementSigma * randn(1, 2);
    measurement.available = geometricVisibility >= 0.15;
    measurement.position = targetPosition + noise;
    if measurement.available
        measurement.confidence = min(0.95, 0.35 + 0.6 * geometricVisibility);
    else
        measurement.position = [NaN NaN];
        measurement.confidence = geometricVisibility * 0.35;
    end
end

function nextUav = updateUav(currentUav, goalPoint, cfg)
    delta = goalPoint - currentUav;
    step = cfg.uavResponseGain * delta;
    maxStep = cfg.uavMaxSpeed * cfg.dt;
    stepNorm = norm(step);

    if stepNorm > maxStep
        step = step / stepNorm * maxStep;
    end

    nextUav = currentUav + step;
end

function nextHeading = updateCameraHeading(currentHeading, selected, fallbackVector, cfg)
    currentHeading = normalizeVector(currentHeading, [1 0]);
    if isfield(selected, "heading")
        desiredHeading = normalizeVector(selected.heading, currentHeading);
    else
        desiredHeading = normalizeVector(fallbackVector, currentHeading);
    end
    maxTurn = cfg.cameraSlewRate * cfg.dt;
    currentAngle = atan2(currentHeading(2), currentHeading(1));
    desiredAngle = atan2(desiredHeading(2), desiredHeading(1));
    delta = wrapToPiLocal(desiredAngle - currentAngle);
    delta = max(-maxTurn, min(maxTurn, delta));
    nextAngle = currentAngle + delta;
    nextHeading = [cos(nextAngle) sin(nextAngle)];
end

function commandHealthy = computePlannerCommandHealth(scenario, k, uavPosition, goalPoint, selectedName, cfg)
    if any(string(selectedName) == ["hold_safe" "failsafe" "none"])
        commandHealthy = true;
        return
    end

    if ~scenario.plannerFailure(k)
        commandHealthy = true;
        return
    end

    if ~isfield(scenario, "plannerFailureObstacles") || isempty(scenario.plannerFailureObstacles)
        commandHealthy = false;
        return
    end

    segmentClearance = paperline.minSegmentObstacleClearance(uavPosition, ...
        goalPoint, scenario.plannerFailureObstacles);
    commandHealthy = segmentClearance > cfg.plannerFailureClearance;
end

function [safeCandidates, info] = applyCandidateBlacklist(safeCandidates, failedName, failedPoint, countdown, cfg)
    info.active = false;
    info.name = "";

    enabled = (~isfield(cfg, "enableCandidateBlacklist") || cfg.enableCandidateBlacklist) ...
        && (~isfield(cfg, "usePlannerFeedback") || cfg.usePlannerFeedback);
    if ~enabled || countdown <= 0 || isempty(safeCandidates)
        return
    end

    names = string({safeCandidates.name});
    sameName = strlength(string(failedName)) > 0 & names == string(failedName);
    sameGeometry = false(1, numel(safeCandidates));
    if all(isfinite(failedPoint))
        candidatePoints = reshape([safeCandidates.point], 2, []).';
        sameGeometry = vecnorm(candidatePoints - failedPoint, 2, 2).' <= cfg.failedCandidateDistance;
    end
    keep = ~(sameName | sameGeometry);
    if any(keep)
        safeCandidates = safeCandidates(keep);
        info.active = true;
        info.name = string(failedName);
    end
end

function [failedName, failedPoint, countdown] = updateCandidateBlacklist(selected, commandHealthy, failedName, failedPoint, countdown, cfg)
    enabled = (~isfield(cfg, "enableCandidateBlacklist") || cfg.enableCandidateBlacklist) ...
        && (~isfield(cfg, "usePlannerFeedback") || cfg.usePlannerFeedback);
    if ~enabled
        failedName = "";
        failedPoint = [NaN NaN];
        countdown = 0;
        return
    end

    holdNames = ["hold_safe" "failsafe" "none"];
    if ~commandHealthy && ~any(string(selected.name) == holdNames)
        failedName = string(selected.name);
        failedPoint = selected.point;
        countdown = cfg.failedCandidateCooldownFrames;
    elseif countdown > 0
        countdown = countdown - 1;
        if countdown == 0
            failedName = "";
            failedPoint = [NaN NaN];
        end
    end
end

function angle = wrapToPiLocal(angle)
    angle = mod(angle + pi, 2 * pi) - pi;
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

function selected = selectCandidate(safeCandidates, uavPosition, prediction, obstacles, lastName, cfg)
    policy = lower(string(cfg.decisionPolicy));

    switch policy
        case "proposed"
            selected = paperline.scoreCandidates(safeCandidates, uavPosition, prediction, ...
                obstacles, lastName, cfg);

        case "fixed_behind"
            selected = selectNamedCandidate(safeCandidates, "behind", uavPosition, prediction, obstacles, cfg);

        case "nearest_feasible"
            distances = arrayfun(@(candidate) norm(candidate.point - uavPosition), safeCandidates);
            [~, bestIndex] = min(distances);
            selected = safeCandidates(bestIndex);
            selected.score = 1 / (1 + distances(bestIndex));
            selected.occlusionRisk = paperline.lineOfSightRisk(selected.point, ...
                prediction.position, obstacles, cfg);
            selected.allScores = selected.score;

        otherwise
            error("paperline:simulateRun:UnknownDecisionPolicy", ...
                "Unknown decisionPolicy '%s'.", policy);
    end
end

function selected = selectNamedCandidate(safeCandidates, name, uavPosition, prediction, obstacles, cfg)
    names = string({safeCandidates.name});
    index = find(names == string(name), 1);
    if isempty(index)
        selected = makeHoldSelection("unavailable_" + string(name), uavPosition, NaN, 1);
        return
    end

    selected = safeCandidates(index);
    selected.score = 1.0;
    selected.occlusionRisk = paperline.lineOfSightRisk(selected.point, prediction.position, obstacles, cfg);
    selected.allScores = selected.score;
end

function selected = makeHoldSelection(name, point, score, occlusionRisk)
    selected = struct("name", string(name), "point", point, ...
        "score", score, "occlusionRisk", occlusionRisk);
end
