classdef extractDycLaptimeMetricTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addOptimizationPath(testCase)
            testFolder = fileparts(mfilename('fullpath'));
            optimizationFolder = fileparts(testFolder);
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(optimizationFolder));
        end
    end

    methods (Test)
        function testExtractsStationStopLaptime(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            logFile = fullfile(fixture.Folder, 'LastRun_log.txt');
            endFile = fullfile(fixture.Folder, 'LastRun_end.par');
            writelines([
                "Run started: VS output file = temp.vsb"
                "Run stopped at t = 21.6025. Station limit reached: driver station = 245.002"
                "Computational time ratio: RTIME = 0.142341"
            ], logFile);
            writelines([
                "OPT_STOP            1 ! stop when TSTOP or SSTOP reached"
                "TSTOP              30 ; s ! Stop time"
                "SSTOP             245 ; m ! Stop station"
                "SV_STATION 245.0016584 ; m ! Station of vehicle"
            ], endFile);
            cfg = dyc_pid_optimization_config();

            metric = extract_dyc_laptime_metric(logFile, endFile, cfg);

            testCase.verifyEqual(metric.status, "valid");
            testCase.verifyEqual(metric.lapTime_s, 21.6025, 'AbsTol', 1e-9);
            testCase.verifyEqual(metric.finishStation_m, 245.002, 'AbsTol', 1e-9);
            testCase.verifyEqual(metric.stopReason, "Station limit reached");
            testCase.verifyEqual(metric.failureReason, "");
        end

        function testRejectsTimeLimitStop(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            logFile = fullfile(fixture.Folder, 'LastRun_log.txt');
            endFile = fullfile(fixture.Folder, 'LastRun_end.par');
            writelines([
                "Run started: VS output file = temp.vsb"
                "Run stopped at t = 30.0000. Time limit reached."
            ], logFile);
            writelines("SV_STATION 210.0 ; m ! Station of vehicle", endFile);
            cfg = dyc_pid_optimization_config();

            metric = extract_dyc_laptime_metric(logFile, endFile, cfg);

            testCase.verifyEqual(metric.status, "invalid");
            testCase.verifyTrue(contains(metric.failureReason, "station limit"));
            testCase.verifyTrue(isnan(metric.lapTime_s));
        end

        function testExtractsAutocrossExternalControlStopTime(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            logFile = fullfile(fixture.Folder, 'LastRun_log.txt');
            endFile = fullfile(fixture.Folder, 'LastRun_end.par');
            writelines([
                "Run started: VS output file = temp.vsb"
                "Run stopped at t = 20.374. External control (manual or external model)"
            ], logFile);
            writelines("TSTART         20.374 ; s ! Starting time for the simulation clock", endFile);
            cfg = dyc_pid_optimization_config(struct('metric', struct( ...
                'primaryMetric', 'carsim_autocross_external_stop_time')));

            metric = extract_dyc_laptime_metric(logFile, endFile, cfg);

            testCase.verifyEqual(metric.status, "valid");
            testCase.verifyEqual(metric.lapTime_s, 20.374, 'AbsTol', 1e-9);
            testCase.verifyEqual(metric.stopReason, "External control (manual or external model)");
            testCase.verifyEqual(metric.objective, 20.374, 'AbsTol', 1e-9);
        end

        function testExtractsAutocrossEventStopTime(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            logFile = fullfile(fixture.Folder, 'LastRun_log.txt');
            endFile = fullfile(fixture.Folder, 'LastRun_end.par');
            writelines([
                "Run started: VS output file = temp.vsb"
                "Event #1 occurred at t = 84.044: SV_N_START_CROSS >= 2"
                "Run stopped at t = 84.044. VS Command STOP_RUN_NOW End event triggered"
                "Used Dataset: Events; End Events"
            ], logFile);
            writelines("SV_STATION 1234.5 ; m ! Station of vehicle", endFile);
            cfg = dyc_pid_optimization_config(struct('metric', struct( ...
                'primaryMetric', 'carsim_autocross_stop_time')));

            metric = extract_dyc_laptime_metric(logFile, endFile, cfg);

            testCase.verifyEqual(metric.status, "valid");
            testCase.verifyEqual(metric.lapTime_s, 84.044, 'AbsTol', 1e-9);
            testCase.verifyEqual(metric.stopReason, "VS Command STOP_RUN_NOW End event triggered");
            testCase.verifyEqual(metric.objective, 84.044, 'AbsTol', 1e-9);
            testCase.verifyEqual(metric.svStation_m, 1234.5, 'AbsTol', 1e-9);
        end

        function testRejectsMissingLog(testCase)
            cfg = dyc_pid_optimization_config();
            metric = extract_dyc_laptime_metric('Z:\missing\LastRun_log.txt', '', cfg);

            testCase.verifyEqual(metric.status, "invalid");
            testCase.verifyEqual(metric.failureReason, "LastRun_log.txt not found");
        end
    end
end
