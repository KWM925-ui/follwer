function summary = runPaperLineBatch(options)
%runPaperLineBatch Run stage-one paper-line scenarios, baselines, and ablations.

    arguments
        options.Seeds (1,:) double = 1:5
        options.OutputDir (1,1) string = fullfile("..", "outputs", "paper_line_batch")
        options.FigureDir (1,1) string = fullfile("..", "figures_generated", "paper_line_batch")
        options.SaveOutputs (1,1) logical = true
    end

    scenarioTypes = ["straight" "sudden_turn" "obstacle_occlusion" ...
        "short_loss" "occlusion_reacquire" "planner_failure"];
    conditions = ["proposed" "fixed_behind" "nearest_feasible" ...
        "no_prediction" "fixed_safety_margin" "no_recovery_fsm" ...
        "ca_kf" "no_occlusion_score" "no_visibility_score" "no_planner_feedback"];

    runs = runStageOneBatch(scenarioTypes, conditions, options.Seeds);
    summary.runs = runs;
    summary.byCondition = summarizeRuns(runs, "condition");
    summary.byScenarioCondition = summarizeRuns(runs, ["scenario" "condition"]);

    if options.SaveOutputs
        if ~isfolder(options.OutputDir)
            mkdir(options.OutputDir);
        end
        if ~isfolder(options.FigureDir)
            mkdir(options.FigureDir);
        end

        writetable(runs, fullfile(options.OutputDir, "stage1_runs.csv"));
        writetable(summary.byCondition, fullfile(options.OutputDir, "stage1_by_condition.csv"));
        writetable(summary.byScenarioCondition, fullfile(options.OutputDir, "stage1_by_scenario_condition.csv"));

        plotConditionSummary(summary.byCondition, fullfile(options.FigureDir, "stage1_condition_summary.png"));
    end
end

function runs = runStageOneBatch(scenarioTypes, conditions, seeds)
    rows = cell(numel(scenarioTypes) * numel(conditions) * numel(seeds), 1);
    row = 1;
    for scenarioType = scenarioTypes
        for condition = conditions
            for seed = seeds
                cfg = applyCondition(paperline.defaultConfig(), scenarioType, condition);
                rows{row} = runOne(seed, scenarioType, condition, cfg);
                row = row + 1;
            end
        end
    end
    runs = struct2table([rows{:}].');
end

function cfg = applyCondition(cfg, scenarioType, condition)
    cfg.scenarioType = scenarioType;
    cfg.decisionPolicy = "proposed";
    cfg.predictionMode = "cv_kf";

    switch string(condition)
        case "proposed"
            return

        case "fixed_behind"
            cfg.decisionPolicy = "fixed_behind";

        case "nearest_feasible"
            cfg.decisionPolicy = "nearest_feasible";

        case "no_prediction"
            cfg.predictionMode = "current_measurement";

        case "fixed_safety_margin"
            cfg.useSpeedMargin = false;
            cfg.useCovarianceMargin = false;

        case "no_recovery_fsm"
            cfg.useFsmRecovery = false;

        case "ca_kf"
            cfg.predictionMode = "ca_kf";

        case "no_occlusion_score"
            cfg.useOcclusionScore = false;

        case "no_visibility_score"
            cfg.useVisibilityScore = false;

        case "no_planner_feedback"
            cfg.usePlannerFeedback = false;

        otherwise
            error("paperline:runPaperLineBatch:UnknownCondition", ...
                "Unknown condition '%s'.", condition);
    end
end

function row = runOne(seed, scenarioType, condition, cfg)
    rng(seed, "twister");
    scenario = paperline.makeScenario(cfg, Seed=seed);
    result = paperline.simulateRun(scenario, cfg);
    metrics = paperline.computeMetrics(result, cfg);

    row.scenario = string(scenarioType);
    row.condition = string(condition);
    row.seed = seed;
    row.predictionMode = string(cfg.predictionMode);
    row.decisionPolicy = string(cfg.decisionPolicy);
    row.meanDistanceError = metrics.meanDistanceError;
    row.maxDistanceError = metrics.maxDistanceError;
    row.meanViewAngleError = metrics.meanViewAngleError;
    row.maxViewAngleError = metrics.maxViewAngleError;
    row.meanPredictionError = metrics.meanPredictionError;
    row.maxPredictionError = metrics.maxPredictionError;
    row.minClearance = metrics.minClearance;
    row.nearMissCount = metrics.nearMissCount;
    row.visibilityRatio = metrics.visibilityRatio;
    row.lossCount = metrics.lossCount;
    row.lossDuration = metrics.lossDuration;
    row.meanReacquisitionTime = metrics.meanReacquisitionTime;
    row.meanOcclusionRisk = metrics.meanOcclusionRisk;
    row.stateSwitchCount = metrics.stateSwitchCount;
    row.candidateSwitchCount = metrics.candidateSwitchCount;
    row.safeCandidateRatio = metrics.safeCandidateRatio;
    row.plannerFailureRecoveryTime = metrics.plannerFailureRecoveryTime;
    row.plannerFailureCount = metrics.plannerFailureCount;
    row.plannerFailureBurstMax = metrics.plannerFailureBurstMax;
    row.blacklistActivationCount = metrics.blacklistActivationCount;
    row.trackingSuccess = metrics.trackingSuccess;
    row.recoverySuccess = metrics.recoverySuccess;
    row.safetySuccess = metrics.safetySuccess;
    row.taskSuccess = metrics.taskSuccess;
    row.decisionLatencyMs = metrics.decisionLatencyMs;
end

function summary = summarizeRuns(runs, groupVariables)
    groupVariables = string(groupVariables);
    [groupIds, groupTable] = findgroups(runs(:, cellstr(groupVariables)));
    metricNames = ["meanDistanceError" "meanViewAngleError" "meanPredictionError" ...
        "minClearance" "nearMissCount" "visibilityRatio" "lossDuration" ...
        "meanReacquisitionTime" "meanOcclusionRisk" "candidateSwitchCount" ...
        "plannerFailureRecoveryTime" "plannerFailureCount" "plannerFailureBurstMax" ...
        "blacklistActivationCount" "trackingSuccess" "recoverySuccess" ...
        "safetySuccess" "taskSuccess"];

    summary = groupTable;
    summary.n = splitapply(@numel, runs.seed, groupIds);

    for metricName = metricNames
        values = runs.(metricName);
        summary.(metricName + "_mean") = splitapply(@(x) mean(x, "omitnan"), values, groupIds);
        summary.(metricName + "_std") = splitapply(@(x) std(x, "omitnan"), values, groupIds);
    end
end

function plotConditionSummary(summaryTable, outputPath)
    fig = figure("Visible", "off", "Name", "Stage-one condition summary", "NumberTitle", "off");
    tiledlayout(fig, 2, 3, "TileSpacing", "compact");
    conditions = categorical(summaryTable.condition);

    nexttile;
    bar(conditions, summaryTable.visibilityRatio_mean);
    ylabel("visible ratio");
    grid on;

    nexttile;
    bar(conditions, summaryTable.lossDuration_mean);
    ylabel("loss duration");
    grid on;

    nexttile;
    bar(conditions, summaryTable.meanReacquisitionTime_mean);
    ylabel("reacquisition time");
    grid on;

    nexttile;
    bar(conditions, summaryTable.nearMissCount_mean);
    ylabel("near-miss count");
    grid on;

    nexttile;
    bar(conditions, summaryTable.minClearance_mean);
    ylabel("min clearance");
    grid on;

    nexttile;
    bar(conditions, summaryTable.plannerFailureRecoveryTime_mean);
    ylabel("planner recovery time");
    grid on;

    sgtitle("Stage-one paper-line conditions", "Interpreter", "none");
    exportgraphics(fig, outputPath, "Resolution", 160);
    close(fig);
end
