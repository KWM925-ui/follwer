function scenario = makeScenario(cfg, options)
%makeScenario Create deterministic 2-D paper-line scenarios.

    arguments
        cfg (1,1) struct
        options.Seed (1,1) double = 7
    end

    rng(options.Seed, "twister");

    time = (0:cfg.dt:cfg.duration).';
    n = numel(time);
    scenarioType = string(cfg.scenarioType);
    target = makeTargetTrajectory(time, scenarioType);
    obstacles = makeObstacles(scenarioType);

    dropout = false(n, 1);
    switch scenarioType
        case "short_loss"
            dropout(time >= 8.0 & time <= 8.8) = true;

        case "occlusion_reacquire"
            dropout(time >= 7.2 & time <= 8.6) = true;
            dropout(time >= 15.0 & time <= 16.4) = true;

        case "recovery_corner_loss"
            dropout(time >= 7.4 & time <= 9.4) = true;
            dropout(time >= 14.0 & time <= 15.0) = true;

        otherwise
            dropout(:) = false;
    end

    plannerFailure = false(n, 1);
    if any(scenarioType == ["planner_failure" "planner_blocked_goal"])
        plannerFailure(time >= 10.5 & time <= 12.0) = true;
    elseif scenarioType == "planner_feedback_stress"
        plannerFailure(time >= 7.5 & time <= 13.5) = true;
    end

    scenario.time = time;
    scenario.type = scenarioType;
    scenario.target = target;
    scenario.initialUav = target(1, :) + [-cfg.desiredDistance -0.4];
    scenario.obstacles = obstacles;
    scenario.dropout = dropout;
    scenario.plannerFailure = plannerFailure;
    scenario.plannerFailureObstacles = makePlannerFailureObstacles(scenarioType);
end

function target = makeTargetTrajectory(time, scenarioType)
    target = zeros(numel(time), 2);

    switch scenarioType
        case "straight"
            target(:, 1) = 0.50 * time;
            target(:, 2) = 0.15 * sin(0.35 * time);

        case "sudden_turn"
            for k = 1:numel(time)
                t = time(k);
                if t < 8.0
                    target(k, :) = [0.55 * t, 0.1 * sin(0.4 * t)];
                elseif t < 16.0
                    target(k, :) = [4.4 + 0.15 * (t - 8.0), 0.75 * (t - 8.0)];
                else
                    target(k, :) = [5.6 + 0.45 * (t - 16.0), 6.0 - 0.2 * (t - 16.0)];
                end
            end

        case "obstacle_occlusion"
            target(:, 1) = 0.45 * time;
            target(:, 2) = 0.75 * sin(0.45 * time);

        case "short_loss"
            target(:, 1) = 0.48 * time;
            target(:, 2) = 0.45 * sin(0.55 * time);

        case "planner_failure"
            target(:, 1) = 0.43 * time;
            target(:, 2) = 0.9 * sin(0.38 * time) + 0.2 * sin(1.0 * time);

        case "occlusion_reacquire"
            for k = 1:numel(time)
                t = time(k);
                target(k, :) = [ ...
                    0.45 * t, ...
                    1.15 * sin(0.42 * t) + 0.35 * sin(1.05 * t)];
            end

        case "fixed_behind_occlusion"
            target(:, 1) = 0.45 * time;
            target(:, 2) = 0.20 * sin(0.35 * time);

        case "nearest_visibility_trap"
            target(:, 1) = 0.44 * time;
            target(:, 2) = 0.70 * sin(0.40 * time) + 0.20 * sin(0.95 * time);

        case "recovery_corner_loss"
            for k = 1:numel(time)
                t = time(k);
                if t < 7.5
                    target(k, :) = [0.50 * t, 0.0];
                elseif t < 14.0
                    target(k, :) = [3.75 + 0.12 * (t - 7.5), 0.80 * (t - 7.5)];
                else
                    target(k, :) = [4.53 + 0.42 * (t - 14.0), 5.20 - 0.05 * (t - 14.0)];
                end
            end

        case "planner_feedback_stress"
            target(:, 1) = 0.42 * time;
            target(:, 2) = -0.20 + 0.35 * sin(0.32 * time);

        case "planner_blocked_goal"
            target(:, 1) = 0.42 * time;
            target(:, 2) = -0.15 + 0.25 * sin(0.50 * time);

        otherwise
            error("paperline:makeScenario:UnknownScenarioType", ...
                "Unknown scenarioType '%s'.", scenarioType);
    end
end

function obstacles = makePlannerFailureObstacles(scenarioType)
    switch scenarioType
        case "planner_failure"
            obstacles = struct( ...
                "center", {[1.60 -1.05], [2.10 -1.00]}, ...
                "radius", {0.55, 0.55});

        case {"planner_feedback_stress", "planner_blocked_goal"}
            obstacles = struct( ...
                "center", {[1.35 -0.75], [1.95 -0.70], [2.55 -0.65]}, ...
                "radius", {0.55, 0.55, 0.55});

        otherwise
            obstacles = struct("center", {}, "radius", {});
    end
end

function obstacles = makeObstacles(scenarioType)
    switch scenarioType
        case "straight"
            obstacles = struct("center", {[8.0 4.0]}, "radius", {0.5});

        case "sudden_turn"
            obstacles = struct( ...
                "center", {[4.8 2.6], [6.5 4.4]}, ...
                "radius", {0.55, 0.65});

        case "obstacle_occlusion"
            obstacles = struct( ...
                "center", {[3.1 -1.0], [4.6 0.4], [6.4 -0.9], [8.0 0.8]}, ...
                "radius", {0.70, 0.80, 0.65, 0.70});

        case "short_loss"
            obstacles = struct( ...
                "center", {[5.0 1.2], [8.0 -1.1]}, ...
                "radius", {0.55, 0.55});

        case "planner_failure"
            obstacles = struct( ...
                "center", {[4.5 0.3], [6.8 -0.9], [9.0 0.9]}, ...
                "radius", {0.65, 0.65, 0.70});

        case "occlusion_reacquire"
            obstacles = struct( ...
                "center", {[3.8 0.1], [5.9 -1.2], [7.4 -1.0], [9.0 1.2]}, ...
                "radius", {0.75, 0.65, 0.55, 0.70});

        case "fixed_behind_occlusion"
            obstacles = struct( ...
                "center", {[2.2 -0.05], [4.2 0.05], [6.2 -0.05], [8.2 0.05]}, ...
                "radius", {0.48, 0.52, 0.50, 0.52});

        case "nearest_visibility_trap"
            obstacles = struct( ...
                "center", {[3.0 -0.75], [4.8 0.25], [6.4 -0.80], [8.0 0.45]}, ...
                "radius", {0.70, 0.75, 0.70, 0.75});

        case "recovery_corner_loss"
            obstacles = struct( ...
                "center", {[3.8 1.4], [4.5 2.7], [5.0 4.1], [6.0 5.0]}, ...
                "radius", {0.85, 0.75, 0.70, 0.65});

        case "planner_feedback_stress"
            obstacles = struct( ...
                "center", {[3.3 0.75], [5.0 -1.0], [6.7 0.85]}, ...
                "radius", {0.65, 0.65, 0.70});

        case "planner_blocked_goal"
            obstacles = struct( ...
                "center", {[4.6 0.55], [6.4 -0.75], [8.0 0.70]}, ...
                "radius", {0.65, 0.65, 0.70});
    end
end
