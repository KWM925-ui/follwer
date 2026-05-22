function [prediction, filterState] = predictTarget(filterState, measurement, cfg)
%predictTarget Update the selected short-horizon target predictor.

    mode = lower(string(cfg.predictionMode));

    switch mode
        case "current_measurement"
            [prediction, filterState] = predictCurrentMeasurement(filterState, measurement, cfg);

        case "cv_extrapolation"
            [prediction, filterState] = predictCvExtrapolation(filterState, measurement, cfg);

        case "ca_extrapolation"
            [prediction, filterState] = predictCaExtrapolation(filterState, measurement, cfg);

        case "ca_kf"
            [prediction, filterState] = predictCaKalman(filterState, measurement, cfg);

        case "cv_kf"
            [prediction, filterState] = predictCvKalman(filterState, measurement, cfg);

        otherwise
            error("paperline:predictTarget:UnknownMode", ...
                "Unknown predictionMode '%s'.", mode);
    end
end

function [prediction, filterState] = predictCurrentMeasurement(filterState, measurement, cfg)
    if measurement.available
        position = measurement.position(:);
        velocity = estimateVelocity(filterState.x(1:2), position, cfg.dt);
    else
        position = filterState.x(1:2);
        velocity = filterState.x(3:4);
    end

    filterState.x = [position; velocity];
    filterState.P = updateHeuristicCovariance(filterState.P, measurement.available, cfg);

    prediction.estimate = position.';
    prediction.velocity = velocity.';
    prediction.position = position.';
    prediction.covariance = filterState.P;
    prediction.confidence = measurement.confidence;
end

function [prediction, filterState] = predictCvExtrapolation(filterState, measurement, cfg)
    if measurement.available
        position = measurement.position(:);
        velocity = estimateVelocity(filterState.x(1:2), position, cfg.dt);
    else
        position = filterState.x(1:2) + filterState.x(3:4) * cfg.dt;
        velocity = filterState.x(3:4);
    end

    filterState.x = [position; velocity];
    filterState.P = updateHeuristicCovariance(filterState.P, measurement.available, cfg);
    horizonPosition = position + velocity * cfg.predictionHorizon;

    prediction.estimate = position.';
    prediction.velocity = velocity.';
    prediction.position = horizonPosition.';
    prediction.covariance = filterState.P;
    prediction.confidence = measurement.confidence;
end

function [prediction, filterState] = predictCaExtrapolation(filterState, measurement, cfg)
    if ~isfield(filterState, "acceleration")
        filterState.acceleration = [0; 0];
    end

    if measurement.available
        position = measurement.position(:);
        velocity = estimateVelocity(filterState.x(1:2), position, cfg.dt);
        acceleration = (velocity - filterState.x(3:4)) / cfg.dt;
    else
        acceleration = filterState.acceleration;
        position = filterState.x(1:2) + filterState.x(3:4) * cfg.dt + 0.5 * acceleration * cfg.dt^2;
        velocity = filterState.x(3:4) + acceleration * cfg.dt;
    end

    filterState.x = [position; velocity];
    filterState.acceleration = acceleration;
    filterState.P = updateHeuristicCovariance(filterState.P, measurement.available, cfg);
    h = cfg.predictionHorizon;
    horizonPosition = position + velocity * h + 0.5 * acceleration * h^2;

    prediction.estimate = position.';
    prediction.velocity = velocity.';
    prediction.position = horizonPosition.';
    prediction.covariance = filterState.P;
    prediction.confidence = measurement.confidence;
end

function [prediction, filterState] = predictCvKalman(filterState, measurement, cfg)
    dt = cfg.dt;
    F = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1];
    H = [1 0 0 0; 0 1 0 0];
    q = cfg.processNoise;
    Q = q^2 * [dt^4/4 0 dt^3/2 0; 0 dt^4/4 0 dt^3/2; dt^3/2 0 dt^2 0; 0 dt^3/2 0 dt^2];
    R = eye(2) * cfg.measurementCovariance;

    xPred = F * filterState.x;
    PPred = F * filterState.P * F.' + Q;

    if measurement.available
        z = measurement.position(:);
        innovation = z - H * xPred;
        S = H * PPred * H.' + R;
        K = PPred * H.' / S;
        filterState.x = xPred + K * innovation;
        filterState.P = (eye(4) - K * H) * PPred;
    else
        filterState.x = xPred;
        filterState.P = PPred;
    end

    h = cfg.predictionHorizon;
    Fh = [1 0 h 0; 0 1 0 h; 0 0 1 0; 0 0 0 1];
    xHorizon = Fh * filterState.x;
    PHorizon = Fh * filterState.P * Fh.';

    prediction.estimate = filterState.x(1:2).';
    prediction.velocity = filterState.x(3:4).';
    prediction.position = xHorizon(1:2).';
    prediction.covariance = PHorizon;
    prediction.confidence = measurement.confidence;
end

function [prediction, filterState] = predictCaKalman(filterState, measurement, cfg)
    dt = cfg.dt;
    F = [ ...
        1 0 dt 0 0.5*dt^2 0
        0 1 0 dt 0 0.5*dt^2
        0 0 1 0 dt 0
        0 0 0 1 0 dt
        0 0 0 0 1 0
        0 0 0 0 0 1];
    H = [1 0 0 0 0 0; 0 1 0 0 0 0];
    q = cfg.processNoise;
    Q = q^2 * diag([dt^4 dt^4 dt^2 dt^2 1 1]);
    R = eye(2) * cfg.measurementCovariance;

    xPred = F * filterState.xCa;
    PPred = F * filterState.PCa * F.' + Q;

    if measurement.available
        z = measurement.position(:);
        innovation = z - H * xPred;
        S = H * PPred * H.' + R;
        K = PPred * H.' / S;
        filterState.xCa = xPred + K * innovation;
        filterState.PCa = (eye(6) - K * H) * PPred;
    else
        filterState.xCa = xPred;
        filterState.PCa = PPred;
    end

    h = cfg.predictionHorizon;
    Fh = [ ...
        1 0 h 0 0.5*h^2 0
        0 1 0 h 0 0.5*h^2
        0 0 1 0 h 0
        0 0 0 1 0 h
        0 0 0 0 1 0
        0 0 0 0 0 1];
    xHorizon = Fh * filterState.xCa;
    PHorizon = Fh * filterState.PCa * Fh.';

    filterState.x = filterState.xCa(1:4);
    filterState.P = filterState.PCa(1:4, 1:4);

    prediction.estimate = filterState.xCa(1:2).';
    prediction.velocity = filterState.xCa(3:4).';
    prediction.position = xHorizon(1:2).';
    prediction.covariance = PHorizon;
    prediction.confidence = measurement.confidence;
end

function velocity = estimateVelocity(previousPosition, currentPosition, dt)
    if any(~isfinite(previousPosition)) || norm(currentPosition - previousPosition) < 1e-9
        velocity = [0; 0];
    else
        velocity = (currentPosition - previousPosition) / dt;
    end
end

function covariance = updateHeuristicCovariance(covariance, measurementAvailable, cfg)
    if measurementAvailable
        covariance = max(cfg.measurementCovariance, 0.85) * covariance;
        covariance(1:2, 1:2) = covariance(1:2, 1:2) + eye(2) * cfg.measurementCovariance;
    else
        covariance = covariance + eye(4) * cfg.processNoise;
    end
end
