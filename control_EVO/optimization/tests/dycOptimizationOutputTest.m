classdef dycOptimizationOutputTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addOptimizationPath(testCase)
            testFolder = fileparts(mfilename('fullpath'));
            optimizationFolder = fileparts(testFolder);
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(optimizationFolder));
        end
    end

    methods (Test)
        function testCheckpointAppendsRows(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            cfg = dyc_pid_optimization_config(struct('resultsDir', fixture.Folder, 'runLogPath', fullfile(fixture.Folder, 'run_log.csv')));
            first = localResult(1, 6000, 200, 0, "valid", 21.6);
            second = localResult(2, 7000, 300, 10, "invalid", NaN);

            append_dyc_optimization_checkpoint(cfg, first);
            append_dyc_optimization_checkpoint(cfg, second);

            tableOut = readtable(cfg.runLogPath, 'TextType', 'string');
            testCase.verifyEqual(height(tableOut), 2);
            testCase.verifyEqual(tableOut.iteration, [1; 2]);
            testCase.verifyEqual(tableOut.status(1), "valid");
            testCase.verifyEqual(tableOut.status(2), "invalid");
        end

        function testReportWritesSummaryAndApplyScript(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            cfg = dyc_pid_optimization_config(struct('resultsDir', fixture.Folder, 'summaryPath', fullfile(fixture.Folder, 'summary.md'), 'bestPidScriptPath', fullfile(fixture.Folder, 'best_pid_set_param.m'), 'resultMatPath', fullfile(fixture.Folder, 'optimization_result.mat')));
            results = [localResult(1, 6000, 200, 0, "valid", 21.6), localResult(2, 8000, 100, 30, "valid", 20.8)];

            write_dyc_pid_optimization_report(cfg, results, struct('MinObjective', 20.8));

            summaryText = fileread(cfg.summaryPath);
            scriptText = fileread(cfg.bestPidScriptPath);
            testCase.verifyTrue(contains(summaryText, 'Best PID'));
            testCase.verifyTrue(contains(summaryText, '20.8000'));
            testCase.verifyTrue(contains(scriptText, 'bestKp = 8000'));
            testCase.verifyTrue(isfile(cfg.resultMatPath));
        end

        function testReportDoesNotWriteExecutableApplyScriptForNoValidCandidates(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            cfg = dyc_pid_optimization_config(struct('resultsDir', fixture.Folder, 'summaryPath', fullfile(fixture.Folder, 'summary.md'), 'bestPidScriptPath', fullfile(fixture.Folder, 'best_pid_set_param.m'), 'resultMatPath', fullfile(fixture.Folder, 'optimization_result.mat')));
            results = [localResult(1, 6000, 200, 0, "invalid", NaN), localResult(2, 8000, 100, 30, "invalid", NaN)];

            write_dyc_pid_optimization_report(cfg, results, struct('MinObjective', 9999));

            summaryText = fileread(cfg.summaryPath);
            scriptText = fileread(cfg.bestPidScriptPath);
            testCase.verifyTrue(contains(summaryText, 'No valid PID candidate found'));
            testCase.verifyTrue(contains(summaryText, 'Valid candidates: 0'));
            testCase.verifyFalse(contains(scriptText, 'set_param'));
            testCase.verifyTrue(isfile(cfg.resultMatPath));
        end

        function testReportHandlesEmptyResults(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            cfg = dyc_pid_optimization_config(struct('resultsDir', fixture.Folder, 'summaryPath', fullfile(fixture.Folder, 'summary.md'), 'bestPidScriptPath', fullfile(fixture.Folder, 'best_pid_set_param.m'), 'resultMatPath', fullfile(fixture.Folder, 'optimization_result.mat')));

            write_dyc_pid_optimization_report(cfg, [], struct('MinObjective', NaN));

            summaryText = fileread(cfg.summaryPath);
            scriptText = fileread(cfg.bestPidScriptPath);
            testCase.verifyTrue(contains(summaryText, 'Evaluated candidates: 0'));
            testCase.verifyTrue(contains(summaryText, 'Valid candidates: 0'));
            testCase.verifyTrue(contains(summaryText, 'Invalid candidates: 0'));
            testCase.verifyFalse(contains(scriptText, 'set_param'));
            testCase.verifyTrue(isfile(cfg.resultMatPath));
        end

        function testTextFieldsAcceptMixedCharAndStringValues(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            cfg = dyc_pid_optimization_config(struct('resultsDir', fixture.Folder, 'runLogPath', fullfile(fixture.Folder, 'run_log.csv'), 'summaryPath', fullfile(fixture.Folder, 'summary.md'), 'bestPidScriptPath', fullfile(fixture.Folder, 'best_pid_set_param.m'), 'resultMatPath', fullfile(fixture.Folder, 'optimization_result.mat')));
            first = localResult(1, 6000, 200, 0, "valid", 21.6);
            second = localResult(2, 8000, 100, 30, "valid", 20.8);
            first.status = 'valid';
            first.stopReason = 'Station limit reached';
            first.failureReason = '';
            first.lastRunLogPath = 'E:\example\LastRun_log.txt';

            append_dyc_optimization_checkpoint(cfg, first);
            write_dyc_pid_optimization_report(cfg, [first, second], struct('MinObjective', 20.8));

            tableOut = readtable(cfg.runLogPath, 'TextType', 'string');
            scriptText = fileread(cfg.bestPidScriptPath);
            testCase.verifyEqual(tableOut.status(1), "valid");
            testCase.verifyTrue(contains(scriptText, 'bestKp = 8000'));
        end

        function testReportBestSelectionIgnoresInvalidRows(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            cfg = dyc_pid_optimization_config(struct('resultsDir', fixture.Folder, 'summaryPath', fullfile(fixture.Folder, 'summary.md'), 'bestPidScriptPath', fullfile(fixture.Folder, 'best_pid_set_param.m'), 'resultMatPath', fullfile(fixture.Folder, 'optimization_result.mat')));
            invalidFast = localResult(1, 7000, 300, 10, "invalid", NaN);
            invalidFast.objective = 1;
            validBest = localResult(2, 8000, 100, 30, "valid", 20.8);

            write_dyc_pid_optimization_report(cfg, [invalidFast, validBest], struct('MinObjective', 20.8));

            summaryText = fileread(cfg.summaryPath);
            scriptText = fileread(cfg.bestPidScriptPath);
            testCase.verifyTrue(contains(summaryText, 'Best PID: Kp = 8000'));
            testCase.verifyTrue(contains(scriptText, 'bestKp = 8000'));
            testCase.verifyFalse(contains(scriptText, 'bestKp = 7000'));
        end

        function testOutputArtifactsStayUnderResultsDir(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            cfg = dyc_pid_optimization_config(struct('resultsDir', fixture.Folder));
            result = localResult(1, 6000, 200, 0, "valid", 21.6);

            append_dyc_optimization_checkpoint(cfg, result);
            write_dyc_pid_optimization_report(cfg, result, struct('MinObjective', 21.6));

            testCase.verifyTrue(isfile(fullfile(fixture.Folder, 'run_log.csv')));
            testCase.verifyTrue(isfile(fullfile(fixture.Folder, 'summary.md')));
            testCase.verifyTrue(isfile(fullfile(fixture.Folder, 'best_pid_set_param.m')));
            testCase.verifyTrue(isfile(fullfile(fixture.Folder, 'optimization_result.mat')));
        end

        function testCheckpointRejectsParentTraversalRunLogPath(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            cfg = dyc_pid_optimization_config(struct( ...
                'resultsDir', fixture.Folder, ...
                'runLogPath', fullfile(fixture.Folder, '..', 'escape', 'run_log.csv')));
            result = localResult(1, 6000, 200, 0, "valid", 21.6);

            testCase.verifyError( ...
                @() append_dyc_optimization_checkpoint(cfg, result), ...
                'dyc:optimization:ArtifactPathOutsideResultsDir');
            testCase.verifyFalse(isfile(fullfile(fixture.Folder, '..', 'escape', 'run_log.csv')));
        end

        function testReportRejectsParentTraversalArtifactPaths(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            cfg = dyc_pid_optimization_config(struct( ...
                'resultsDir', fixture.Folder, ...
                'summaryPath', fullfile(fixture.Folder, '..', 'escape', 'summary.md'), ...
                'bestPidScriptPath', fullfile(fixture.Folder, 'best_pid_set_param.m'), ...
                'resultMatPath', fullfile(fixture.Folder, 'optimization_result.mat')));
            result = localResult(1, 6000, 200, 0, "valid", 21.6);

            testCase.verifyError( ...
                @() write_dyc_pid_optimization_report(cfg, result, struct('MinObjective', 21.6)), ...
                'dyc:optimization:ArtifactPathOutsideResultsDir');
            testCase.verifyFalse(isfile(fullfile(fixture.Folder, '..', 'escape', 'summary.md')));
        end
    end
end

function result = localResult(iteration, kp, ki, kd, status, lapTime)
result = struct();
result.iteration = iteration;
result.Kp = kp;
result.Ki = ki;
result.Kd = kd;
result.lapTime_s = lapTime;
result.finishStation_m = 245.0;
result.objective = lapTime;
if isnan(lapTime)
    result.objective = 9999;
end
result.penalty = result.objective - lapTime;
if isnan(lapTime)
    result.penalty = 9999;
end
result.stopReason = "Station limit reached";
result.status = status;
result.failureReason = "";
if status == "invalid"
    result.failureReason = "CarSim stop reason was not station limit";
end
result.lastRunLogPath = "E:\example\LastRun_log.txt";
result.elapsedWallTime_s = 1.2;
end
