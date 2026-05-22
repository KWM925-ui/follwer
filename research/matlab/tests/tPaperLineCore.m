classdef tPaperLineCore < matlab.unittest.TestCase
    %tPaperLineCore Unit tests for the paper-line MATLAB scaffold.

    methods (TestClassSetup)
        function addPackagePath(testCase)
            root = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root, ...
                IncludingSubfolders=true));
        end
    end

    methods (TestMethodSetup)
        function resetRandomSeed(testCase)
            originalRng = rng;
            testCase.addTeardown(@() rng(originalRng));
            rng(7, "twister");
        end
    end

    methods (Test)
        function testGenerateFollowCandidates(testCase)
            cfg = paperline.defaultConfig();
            prediction.position = [3 1];
            prediction.viewpointReference = prediction.position;
            prediction.velocity = [1 0];
            prediction.covariance = eye(4) * 0.1;

            candidates = paperline.generateCandidates(prediction, [0 0], "FOLLOW", cfg);
            names = string({candidates.name});

            testCase.verifyEqual(numel(candidates), 4);
            testCase.verifyTrue(any(names == "behind"));
            testCase.verifyTrue(any(names == "left"));
            testCase.verifyTrue(any(names == "right"));
            testCase.verifyTrue(any(names == "far_safe"));
            testCase.verifyFalse(any(names == "search_reacquire"));
            for i = 1:numel(candidates)
                targetDirection = prediction.position - candidates(i).point;
                targetDirection = targetDirection / norm(targetDirection);
                testCase.verifyGreaterThan(dot(candidates(i).heading, targetDirection), 0.99);
            end
        end

        function testSearchCandidateOnlyInSearchState(testCase)
            cfg = paperline.defaultConfig();
            prediction.position = [3 1];
            prediction.viewpointReference = prediction.position;
            prediction.velocity = [1 0];
            prediction.covariance = eye(4) * 0.1;

            candidates = paperline.generateCandidates(prediction, [0 0], "SEARCH_SAFE_VIEWPOINT", cfg);
            names = string({candidates.name});

            testCase.verifyGreaterThan(numel(candidates), 1);
            testCase.verifyTrue(all(startsWith(names, "search_reacquire_")));
        end

        function testSafetyFilterRejectsBlockedPoint(testCase)
            cfg = paperline.defaultConfig();
            prediction.position = [0 0];
            prediction.velocity = [1 0];
            prediction.covariance = eye(4) * 0.01;
            obstacle = struct("center", [1 0], "radius", 0.5);
            candidates = struct("name", "blocked", "point", [1 0]);

            [safeCandidates, info] = paperline.filterCandidates(candidates, [0 0], prediction, obstacle, cfg);

            testCase.verifyEmpty(safeCandidates);
            testCase.verifyEqual(info.reasons, "point_clearance");
        end

        function testLineOfSightRiskDetectsIntersection(testCase)
            cfg = paperline.defaultConfig();
            obstacle = struct("center", [1 0], "radius", 0.4);

            blockedRisk = paperline.lineOfSightRisk([0 0], [2 0], obstacle, cfg);
            clearRisk = paperline.lineOfSightRisk([0 2], [2 2], obstacle, cfg);

            testCase.verifyEqual(blockedRisk, 1.0);
            testCase.verifyLessThan(clearRisk, 0.1);
        end

        function testVisibilityModelSanity(testCase)
            cfg = paperline.defaultConfig();
            noObstacles = struct("center", {}, "radius", {});

            frontVisible = paperline.computeVisibility([3 0], [0 0], [1 0], noObstacles, cfg);
            sideVisible = paperline.computeVisibility([0 3], [0 0], [1 0], noObstacles, cfg);
            farVisible = paperline.computeVisibility([20 0], [0 0], [1 0], noObstacles, cfg);

            testCase.verifyGreaterThan(frontVisible, 0.9);
            testCase.verifyLessThan(sideVisible, frontVisible);
            testCase.verifyEqual(farVisible, 0.0);
        end

        function testVisibilityDropsForOcclusion(testCase)
            cfg = paperline.defaultConfig();
            obstacle = struct("center", [1.5 0], "radius", 0.5);
            noObstacles = struct("center", {}, "radius", {});

            clearVisible = paperline.computeVisibility([3 0], [0 0], [1 0], noObstacles, cfg);
            blockedVisible = paperline.computeVisibility([3 0], [0 0], [1 0], obstacle, cfg);

            testCase.verifyLessThan(blockedVisible, clearVisible);
            testCase.verifyLessThan(blockedVisible, cfg.visibilityThreshold);
        end

        function testFsmMovesFromFollowToPredictHold(testCase)
            cfg = paperline.defaultConfig();
            paperline.updateFsm("FOLLOW", struct("reset", true), cfg);

            context.visible = false;
            context.covarianceTrace = 0.2;
            context.plannerHealthy = true;
            context.hasSafeCandidate = true;
            fsm = paperline.updateFsm("FOLLOW", context, cfg);

            testCase.verifyEqual(fsm.state, "PREDICT_HOLD");
        end

        function testSimulateRunProducesMetrics(testCase)
            cfg = paperline.defaultConfig();
            cfg.duration = 3.0;
            scenario = paperline.makeScenario(cfg, Seed=7);

            result = paperline.simulateRun(scenario, cfg);
            metrics = paperline.computeMetrics(result, cfg);

            testCase.verifySize(result.uav, [numel(scenario.time) 2]);
            testCase.verifyTrue(isfield(metrics, "meanDistanceError"));
            testCase.verifyTrue(isfield(metrics, "nearMissCount"));
            testCase.verifyTrue(isfield(metrics, "lossDuration"));
            testCase.verifyGreaterThanOrEqual(metrics.visibilityRatio, 0);
        end

        function testPredictionBaselinesRun(testCase)
            modes = ["current_measurement" "cv_extrapolation" ...
                "ca_extrapolation" "cv_kf" "ca_kf"];
            for mode = modes
                cfg = paperline.defaultConfig();
                cfg.duration = 1.0;
                cfg.predictionMode = mode;
                scenario = paperline.makeScenario(cfg, Seed=7);

                result = paperline.simulateRun(scenario, cfg);

                testCase.verifySize(result.predictedTarget, [numel(scenario.time) 2]);
                testCase.verifyFalse(any(isnan(result.predictedTarget), "all"));
            end
        end

        function testStageOneScenariosRun(testCase)
            scenarioTypes = ["straight" "sudden_turn" "obstacle_occlusion" ...
                "short_loss" "occlusion_reacquire" "planner_failure"];
            for scenarioType = scenarioTypes
                cfg = paperline.defaultConfig();
                cfg.duration = 2.0;
                cfg.scenarioType = scenarioType;
                scenario = paperline.makeScenario(cfg, Seed=7);

                result = paperline.simulateRun(scenario, cfg);

                testCase.verifySize(result.uav, [numel(scenario.time) 2]);
                testCase.verifyEqual(scenario.type, scenarioType);
            end
        end

        function testDecisionPolicyBaselinesRun(testCase)
            policies = ["proposed" "fixed_behind" "nearest_feasible"];
            for policy = policies
                cfg = paperline.defaultConfig();
                cfg.duration = 2.0;
                cfg.decisionPolicy = policy;
                scenario = paperline.makeScenario(cfg, Seed=7);

                result = paperline.simulateRun(scenario, cfg);

                testCase.verifySize(result.selectedPoint, [numel(scenario.time) 2]);
            end
        end
    end
end
