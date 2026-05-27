function report = summarizePaperLineResults(options)
%summarizePaperLineResults Print key paper-line result comparisons.
%
% Run this after runPaperLineBatch(..., SaveOutputs=true) and
% runPaperLineStressBatch(..., SaveOutputs=true).

    arguments
        options.BatchDir (1,1) string = fullfile("..", "outputs", "paper_line_batch")
        options.StressDir (1,1) string = fullfile("..", "outputs", "paper_line_stress")
    end

    batchPath = fullfile(options.BatchDir, "stage1_by_condition.csv");
    stressPath = fullfile(options.StressDir, "stress_by_condition.csv");

    report = struct();
    if isfile(batchPath)
        batch = readtable(batchPath);
        report.batch = summarizeTable(batch, "stage1");
    else
        warning("paperline:summary:MissingBatch", "Missing %s", batchPath);
        report.batch = table();
    end

    if isfile(stressPath)
        stress = readtable(stressPath);
        report.stress = summarizeTable(stress, "stress");
    else
        warning("paperline:summary:MissingStress", "Missing %s", stressPath);
        report.stress = table();
    end
end

function out = summarizeTable(tbl, label)
    wanted = ["proposed" "fixed_safety_margin" "no_planner_feedback" ...
        "no_prediction" "no_recovery_fsm" "no_occlusion_score" ...
        "no_visibility_score" "fixed_behind" "nearest_feasible"];
    condition = string(tbl.condition);
    keep = ismember(condition, wanted);
    out = tbl(keep, :);

    fprintf("\n%s key rows:\n", label);
    printMetric(out, "nearMissCount_mean");
    printMetric(out, "minClearance_mean");
    printMetric(out, "plannerFailureCount_mean");
    printMetric(out, "plannerFailureBurstMax_mean");
    printMetric(out, "taskSuccess_mean");

    explainDelta(out, "proposed", "fixed_safety_margin", "nearMissCount_mean", ...
        "dynamic safety margin near-miss effect");
    explainDelta(out, "proposed", "no_planner_feedback", "plannerFailureBurstMax_mean", ...
        "planner feedback burst effect");
    explainDelta(out, "proposed", "no_prediction", "taskSuccess_mean", ...
        "prediction contribution check");
end

function printMetric(tbl, metricName)
    if ~ismember(metricName, string(tbl.Properties.VariableNames))
        return
    end
    fprintf("  %s\n", metricName);
    for i = 1:height(tbl)
        fprintf("    %-22s %.4f\n", string(tbl.condition(i)), tbl.(metricName)(i));
    end
end

function explainDelta(tbl, proposedName, baselineName, metricName, label)
    names = string(tbl.condition);
    proposedIdx = find(names == proposedName, 1);
    baselineIdx = find(names == baselineName, 1);
    if isempty(proposedIdx) || isempty(baselineIdx)
        return
    end
    if ~ismember(metricName, string(tbl.Properties.VariableNames))
        return
    end
    proposedValue = tbl.(metricName)(proposedIdx);
    baselineValue = tbl.(metricName)(baselineIdx);
    fprintf("  %s: proposed=%.4f baseline(%s)=%.4f delta=%.4f\n", ...
        label, proposedValue, baselineName, baselineValue, baselineValue - proposedValue);
end
