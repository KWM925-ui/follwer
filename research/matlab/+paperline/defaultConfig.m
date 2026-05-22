function cfg = defaultConfig()
%defaultConfig Return default parameters for the paper-line prototype.

    cfg.dt = 0.1;
    cfg.duration = 24.0;
    cfg.predictionHorizon = 1.0;
    cfg.viewpointPredictionBlend = 0.20;
    cfg.predictionMode = "cv_kf";
    cfg.decisionPolicy = "proposed";
    cfg.scenarioType = "occlusion_reacquire";

    cfg.desiredDistance = 3.0;
    cfg.lateralOffset = 1.2;
    cfg.sideOffset = 2.3;
    cfg.farSafeDistance = 4.8;
    cfg.searchRadius = 4.2;

    cfg.uavMaxSpeed = 1.4;
    cfg.uavResponseGain = 0.75;
    cfg.bodyRadius = 0.35;
    cfg.obstacleMargin = 0.45;
    cfg.speedMarginGain = 0.30;
    cfg.covarianceMarginGain = 1.20;
    cfg.minReachableMargin = 0.15;
    cfg.useSpeedMargin = true;
    cfg.useCovarianceMargin = true;

    cfg.measurementSigma = 0.08;
    cfg.processNoise = 0.06;
    cfg.initialCovariance = 0.20;
    cfg.measurementCovariance = cfg.measurementSigma^2;

    cfg.visibilityThreshold = 0.45;
    cfg.reacquireFrames = 3;
    cfg.predictHoldFrames = 16;
    cfg.searchFramesToFailsafe = 110;
    cfg.safeHoldFramesToFailsafe = 80;
    cfg.covarianceHoldThreshold = 3.0;
    cfg.covarianceFailsafeThreshold = 14.0;

    cfg.occlusionNearDistance = 0.25;
    cfg.fovHalfAngle = deg2rad(62);
    cfg.cameraSlewRate = deg2rad(180);
    cfg.fullVisibilityDistance = 4.8;
    cfg.maxVisibleDistance = 9.5;
    cfg.visibilityOcclusionPenalty = 0.65;
    cfg.nearMissClearance = 1.25;
    cfg.plannerFailureClearance = 1.10;
    cfg.enableCandidateBlacklist = true;
    cfg.failedCandidateCooldownFrames = 14;
    cfg.failedCandidateDistance = 0.85;
    cfg.plannerFailureHoldFrames = 3;
    cfg.searchFanAngles = deg2rad([-55 -25 0 25 55]);

    cfg.successMinVisibilityRatio = 0.58;
    cfg.successMaxLossDuration = 8.5;
    cfg.successMaxReacquisitionTime = 2.0;
    cfg.successMaxPlannerFailureCount = 4;

    cfg.weights.distance = 0.22;
    cfg.weights.angle = 0.18;
    cfg.weights.clearance = 0.20;
    cfg.weights.occlusion = 0.28;
    cfg.weights.motion = 0.12;
    cfg.weights.visibility = 0.10;
    cfg.weights.switching = 0.02;
    cfg.weights.uncertainty = 0.02;

    cfg.scoreScales.distance = 2.5;
    cfg.scoreScales.angle = pi;
    cfg.scoreScales.clearance = 3.0;
    cfg.scoreScales.motion = 3.0;
    cfg.scoreScales.uncertainty = 5.0;

    cfg.useOcclusionScore = true;
    cfg.useVisibilityScore = true;
    cfg.useSwitchPenalty = true;
    cfg.useFsmRecovery = true;
    cfg.usePlannerFeedback = true;
end
