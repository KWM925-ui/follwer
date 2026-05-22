function margin = dynamicSafetyMargin(prediction, uavPosition, cfg)
%dynamicSafetyMargin Compute speed/covariance-inflated clearance threshold.

    positionCovariance = prediction.covariance(1:2, 1:2);
    covarianceScale = sqrt(max(eig(positionCovariance)));
    speedScale = norm(prediction.position - uavPosition) / max(cfg.predictionHorizon, eps);

    margin = cfg.bodyRadius + cfg.obstacleMargin;

    if ~isfield(cfg, "useSpeedMargin") || cfg.useSpeedMargin
        margin = margin + cfg.speedMarginGain * min(speedScale, cfg.uavMaxSpeed);
    end

    if ~isfield(cfg, "useCovarianceMargin") || cfg.useCovarianceMargin
        margin = margin + cfg.covarianceMarginGain * covarianceScale;
    end
end
