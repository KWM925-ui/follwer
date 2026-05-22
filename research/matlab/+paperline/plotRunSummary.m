function plotRunSummary(result, cfg, options)
%plotRunSummary Generate the first paper-line prototype figures.

    arguments
        result (1,1) struct
        cfg (1,1) struct
        options.OutputDir (1,1) string = fullfile("..", "figures_generated", "paper_line_demo")
        options.ShowFigures (1,1) logical = false
        options.SaveFigures (1,1) logical = true
    end

    if options.SaveFigures && ~isfolder(options.OutputDir)
        mkdir(options.OutputDir);
    end

    visibility = ternaryFigureVisibility(options.ShowFigures);

    fig = figure("Visible", visibility, ...
        "Name", "Paper-line demo: trajectory", ...
        "NumberTitle", "off");
    tiledlayout(fig, 2, 2, "TileSpacing", "compact");
    sgtitle(sprintf("Paper-line demo (%s)", cfg.predictionMode), "Interpreter", "none");

    nexttile;
    hold on;
    plot(result.scenario.target(:, 1), result.scenario.target(:, 2), "k-", "LineWidth", 1.5);
    plot(result.uav(:, 1), result.uav(:, 2), "b-", "LineWidth", 1.3);
    plot(result.selectedPoint(:, 1), result.selectedPoint(:, 2), "g.", "MarkerSize", 8);
    for i = 1:numel(result.scenario.obstacles)
        drawCircle(result.scenario.obstacles(i).center, result.scenario.obstacles(i).radius);
    end
    axis equal;
    grid on;
    title("Trajectory and selected follow points");
    legend(["target" "uav" "selected point" "obstacle"], "Location", "best");

    nexttile;
    plot(result.time, vecnorm(result.predictedTarget - result.scenario.target, 2, 2), "LineWidth", 1.2);
    hold on;
    plot(result.time, result.covarianceTrace, "LineWidth", 1.2);
    grid on;
    title("Prediction error and covariance");
    legend("prediction error", "covariance trace", "Location", "best");

    nexttile;
    plot(result.time, result.visibility, "LineWidth", 1.2);
    hold on;
    plot(result.time, result.occlusionRisk, "LineWidth", 1.2);
    ylim([-0.05 1.05]);
    grid on;
    title("Visibility and occlusion risk");
    legend("visibility", "occlusion risk", "Location", "best");

    nexttile;
    plotCategoricalTimeline(result.time, result.stateName);
    title("FSM timeline");

    if options.SaveFigures
        exportgraphics(fig, fullfile(options.OutputDir, "paper_line_summary.png"), "Resolution", 160);
    end

    fig2 = figure("Visible", visibility, ...
        "Name", "Paper-line demo: scores and metrics", ...
        "NumberTitle", "off");
    tiledlayout(fig2, 1, 2, "TileSpacing", "compact");
    nexttile;
    plot(result.time, result.scoreBest, "LineWidth", 1.2);
    grid on;
    title("Best candidate score");

    nexttile;
    bar(categorical(["meanDistErr" "maxDistErr" "minClearance" "visibility"]), ...
        [result.metrics.meanDistanceError result.metrics.maxDistanceError ...
        result.metrics.minClearance result.metrics.visibilityRatio]);
    title("First metric summary");
    grid on;

    if options.SaveFigures
        exportgraphics(fig2, fullfile(options.OutputDir, "paper_line_scores_metrics.png"), "Resolution", 160);
    end

    if options.ShowFigures
        drawnow;
        figure(fig);
        try
            movegui(fig, "center");
            movegui(fig2, "east");
        catch
        end
    end

    if ~options.ShowFigures
        close(fig);
        close(fig2);
    end
end

function visibility = ternaryFigureVisibility(showFigures)
    if showFigures
        visibility = "on";
    else
        visibility = "off";
    end
end

function drawCircle(center, radius)
    theta = linspace(0, 2*pi, 80);
    plot(center(1) + radius * cos(theta), center(2) + radius * sin(theta), "r-", "LineWidth", 1.0);
end

function plotCategoricalTimeline(time, states)
    uniqueStates = unique(states, "stable");
    y = zeros(size(time));
    for i = 1:numel(uniqueStates)
        y(states == uniqueStates(i)) = i;
    end

    stairs(time, y, "LineWidth", 1.2);
    yticks(1:numel(uniqueStates));
    yticklabels(uniqueStates);
    grid on;
end
