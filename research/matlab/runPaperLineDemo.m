function result = runPaperLineDemo(options)
%runPaperLineDemo Run the first paper-line MATLAB prototype demo.

    arguments
        options.OutputDir (1,1) string = fullfile("..", "figures_generated", "paper_line_demo")
        options.ShowFigures (1,1) logical = true
        options.SaveFigures (1,1) logical = true
        options.Seed (1,1) double = 7
    end

    rng(options.Seed, "twister");

    cfg = paperline.defaultConfig();
    scenario = paperline.makeScenario(cfg, Seed=options.Seed);
    result = paperline.simulateRun(scenario, cfg);
    result.metrics = paperline.computeMetrics(result, cfg);

    if options.SaveFigures || options.ShowFigures
        paperline.plotRunSummary(result, cfg, ...
            OutputDir=options.OutputDir, ...
            ShowFigures=options.ShowFigures, ...
            SaveFigures=options.SaveFigures);
    end
end
