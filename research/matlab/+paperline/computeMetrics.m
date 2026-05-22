function metrics = computeMetrics(result, cfg)
%computeMetrics Compute paper-line prototype metrics.

    target = result.scenario.target;
    uav = result.uav;
    distanceError = abs(vecnorm(uav - target, 2, 2) - cfg.desiredDistance);
    predictionError = vecnorm(result.predictedTarget - target, 2, 2);
    visibleMask = result.visibility >= cfg.visibilityThreshold;

    clearance = zeros(numel(result.time), 1);
    for k = 1:numel(result.time)
        clearance(k) = paperline.minObstacleClearance(uav(k, :), result.scenario.obstacles);
    end

    viewAngleError = computeViewAngleError(result);
    [lossCount, lossDuration, reacquisitionTime] = computeLossMetrics(result.time, visibleMask);
    nearMissCount = sum(clearance < cfg.nearMissClearance);
    plannerFailureRecoveryTime = computePlannerFailureRecoveryTime(result);
    plannerFailureCount = computePlannerFailureCount(result);
    plannerFailureBurstMax = computePlannerFailureBurstMax(result);

    metrics.meanDistanceError = mean(distanceError, "omitnan");
    metrics.maxDistanceError = max(distanceError);
    metrics.meanViewAngleError = mean(viewAngleError, "omitnan");
    metrics.maxViewAngleError = max(viewAngleError, [], "omitnan");
    metrics.meanPredictionError = mean(predictionError, "omitnan");
    metrics.maxPredictionError = max(predictionError);
    metrics.minClearance = min(clearance);
    metrics.nearMissCount = nearMissCount;
    metrics.visibilityRatio = mean(visibleMask);
    metrics.lossCount = lossCount;
    metrics.lossDuration = lossDuration;
    metrics.meanReacquisitionTime = reacquisitionTime;
    metrics.meanOcclusionRisk = mean(result.occlusionRisk, "omitnan");
    metrics.stateSwitchCount = sum(result.stateName(2:end) ~= result.stateName(1:end - 1));
    metrics.candidateSwitchCount = sum(result.selectedName(2:end) ~= result.selectedName(1:end - 1));
    metrics.safeCandidateRatio = mean(result.numSafeCandidates > 0);
    metrics.plannerFailureRecoveryTime = plannerFailureRecoveryTime;
    metrics.plannerFailureCount = plannerFailureCount;
    metrics.plannerFailureBurstMax = plannerFailureBurstMax;
    if isfield(result, "blacklistActive")
        metrics.blacklistActivationCount = sum(result.blacklistActive);
    else
        metrics.blacklistActivationCount = 0;
    end
    metrics.trackingSuccess = double(metrics.visibilityRatio >= cfg.successMinVisibilityRatio ...
        && metrics.lossDuration <= cfg.successMaxLossDuration);
    metrics.recoverySuccess = double(metrics.meanReacquisitionTime <= cfg.successMaxReacquisitionTime ...
        && ~any(result.stateName == "FAILSAFE"));
    metrics.safetySuccess = double(metrics.nearMissCount == 0 ...
        && metrics.plannerFailureCount <= cfg.successMaxPlannerFailureCount);
    metrics.taskSuccess = double(metrics.trackingSuccess == 1 ...
        && metrics.recoverySuccess == 1 ...
        && metrics.safetySuccess == 1);
    metrics.decisionLatencyMs = NaN;
end

function viewAngleError = computeViewAngleError(result)
    targetVelocity = [diff(result.scenario.target, 1, 1); [0 0]];
    viewVector = result.scenario.target - result.uav;
    viewAngleError = nan(numel(result.time), 1);

    for k = 1:numel(result.time)
        if norm(targetVelocity(k, :)) < 1e-9 || norm(viewVector(k, :)) < 1e-9
            continue
        end
        targetForward = targetVelocity(k, :) / norm(targetVelocity(k, :));
        preferredView = -targetForward;
        actualView = viewVector(k, :) / norm(viewVector(k, :));
        viewAngleError(k) = acos(max(-1, min(1, dot(preferredView, actualView))));
    end
end

function [lossCount, lossDuration, meanReacquisitionTime] = computeLossMetrics(time, visibleMask)
    dt = median(diff(time));
    lossMask = ~visibleMask;
    lossStarts = find(diff([false; lossMask]) == 1);
    lossEnds = find(diff([lossMask; false]) == -1);

    lossCount = numel(lossStarts);
    lossDuration = sum(lossMask) * dt;
    reacquisitionTimes = nan(lossCount, 1);

    for i = 1:lossCount
        nextVisibleIndex = find(visibleMask(lossEnds(i) + 1:end), 1);
        if ~isempty(nextVisibleIndex)
            reacquisitionTimes(i) = time(lossEnds(i) + nextVisibleIndex) - time(lossEnds(i));
        end
    end

    meanReacquisitionTime = mean(reacquisitionTimes, "omitnan");
    if isnan(meanReacquisitionTime)
        meanReacquisitionTime = 0;
    end
end

function recoveryTime = computePlannerFailureRecoveryTime(result)
    if ~isfield(result, "plannerHealthy")
        recoveryTime = 0;
        return
    end

    failureMask = ~result.plannerHealthy;
    failureEnds = find(diff([failureMask; false]) == -1);
    recoveryTimes = nan(numel(failureEnds), 1);

    for i = 1:numel(failureEnds)
        startIndex = failureEnds(i);
        followIndex = find(result.stateName(startIndex:end) == "FOLLOW", 1);
        if ~isempty(followIndex)
            recoveryTimes(i) = result.time(startIndex + followIndex - 1) - result.time(startIndex);
        end
    end

    recoveryTime = mean(recoveryTimes, "omitnan");
    if isnan(recoveryTime)
        recoveryTime = 0;
    end
end

function failureCount = computePlannerFailureCount(result)
    if ~isfield(result, "plannerCommandHealthy")
        failureCount = 0;
        return
    end

    failureCount = sum(~result.plannerCommandHealthy);
end

function maxBurst = computePlannerFailureBurstMax(result)
    if ~isfield(result, "plannerCommandHealthy")
        maxBurst = 0;
        return
    end

    failureMask = ~result.plannerCommandHealthy;
    burstStarts = find(diff([false; failureMask]) == 1);
    burstEnds = find(diff([failureMask; false]) == -1);
    if isempty(burstStarts)
        maxBurst = 0;
    else
        maxBurst = max(burstEnds - burstStarts + 1);
    end
end
