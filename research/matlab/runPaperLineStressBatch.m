function summary = runPaperLineStressBatch(options)
%runPaperLineStressBatch Run stress scenarios for the paper-line prototype.

    arguments
        options.Seeds (1,:) double = 1:5
        options.OutputDir (1,1) string = fullfile("..", "outputs", "paper_line_stress")
        options.FigureDir (1,1) string = fullfile("..", "figures_generated", "paper_line_stress")
        options.SaveOutputs (1,1) logical = true
    end

    scenarioTypes = ["fixed_behind_occlusion" "nearest_visibility_trap" ...
        "recovery_corner_loss" "planner_feedback_stress" "planner_blocked_goal"];
    conditions = ["proposed" "fixed_behind" "nearest_feasible" ...
        "no_prediction" "fixed_safety_margin" "no_recovery_fsm" ...
        "no_occlusion_score" "no_visibility_score" "no_planner_feedback"];

    runs = runBatch(scenarioTypes, conditions, options.Seeds);
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

        writetable(runs, fullfile(options.OutputDir, "stress_runs.csv"));
        writetable(summary.byCondition, fullfile(options.OutputDir, "stress_by_condition.csv"));
        writetable(summary.byScenarioCondition, fullfile(options.OutputDir, "stress_by_scenario_condition.csv"));

        plotStressSummary(summary.byCondition, fullfile(options.FigureDir, "stress_condition_summary.png"));
    end
end

function runs = runBatch(scenarioTypes, conditions, seeds)
    rows = cell(numel(scenarioTypes) * numel(conditions) * numel(seeds), 1);
    row = 1;
    for scenarioType = scenarioTypes
        for condition = conditions
            for seed = seeds
                cfg = applyStressCondition(paperline.defaultConfig(), scenarioType, condition);
                rows{row} = runOne(seed, scenarioType, condition, cfg);
                row = row + 1;
            end
        end
    end
    runs = struct2table([rows{:}].');
end

function cfg = applyStressCondition(cfg, scenarioType, condition)
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

        case "no_occlusion_score"
            cfg.useOcclusionScore = false;

        case "no_visibility_score"
            cfg.useVisibilityScore = false;

        case "no_planner_feedback"
            cfg.usePlannerFeedback = false;

        otherwise
            error("paperline:runPaperLineStressBatch:UnknownCondition", ...
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
    row.meanViewAngleError = metrics.meanViewAngleError;
    row.meanPredictionError = metrics.meanPredictionError;
    row.minClearance = metrics.minClearance;
    row.nearMissCount = metrics.nearMissCount;
    row.visibilityRatio = metrics.visibilityRatio;
    row.lossDuration = metrics.lossDuration;
    row.meanReacquisitionTime = metrics.meanReacquisitionTime;
    row.meanOcclusionRisk = metrics.meanOcclusionRisk;
    row.candidateSwitchCount = metrics.candidateSwitchCount;
    row.plannerFailureCount = metrics.plannerFailureCount;
    row.plannerFailureBurstMax = metrics.plannerFailureBurstMax;
    row.blacklistActivationCount = metrics.blacklistActivationCount;
    row.trackingSuccess = metrics.trackingSuccess;
    row.recoverySuccess = metrics.recoverySuccess;
    row.safetySuccess = metrics.safetySuccess;
    row.taskSuccess = metrics.taskSuccess;
end

function summary = summarizeRuns(runs, groupVariables)
    groupVariables = string(groupVariables);
    [groupIds, groupTable] = findgroups(runs(:, cellstr(groupVariables)));
    metricNames = ["meanDistanceError" "meanViewAngleError" "meanPredictionError" ...
        "minClearance" "nearMissCount" "visibilityRatio" "lossDuration" ...
        "meanReacquisitionTime" "meanOcclusionRisk" "candidateSwitchCount" ...
        "plannerFailureCount" "plannerFailureBurstMax" "blacklistActivationCount" ...
        "trackingSuccess" "recoverySuccess" "safetySuccess" "taskSuccess"];

    summary = groupTable;
    summary.n = splitapply(@numel, runs.seed, groupIds);

    for metricName = metricNames
        values = runs.(metricName);
        summary.(metricName + "_mean") = splitapply(@(x) mean(x, "omitnan"), values, groupIds);
        summary.(metricName + "_std") = splitapply(@(x) std(x, "omitnan"), values, groupIds);
    end
end

function plotStressSummary(summaryTable, outputPath)
    fig = figure("Visible", "off", "Name", "Stress condition summary", "NumberTitle", "off");
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
    bar(conditions, summaryTable.nearMissCount_mean);
    ylabel("near-miss count");
    grid on;

    nexttile;
    bar(conditions, summaryTable.plannerFailureCount_mean);
    ylabel("planner failures");
    grid on;

    nexttile;
    bar(conditions, summaryTable.plannerFailureBurstMax_mean);
    ylabel("max failure burst");
    grid on;

    nexttile;
    bar(conditions, summaryTable.taskSuccess_mean);
    ylabel("task success");
    grid on;

    sgtitle("Paper-line stress conditions", "Interpreter", "none");
    exportgraphics(fig, outputPath, "Resolution", 160);
    close(fig);
end
