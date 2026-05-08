classdef parseDycSimfileTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addOptimizationPath(testCase)
            testFolder = fileparts(mfilename('fullpath'));
            optimizationFolder = fileparts(testFolder);
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(optimizationFolder));
        end
    end

    methods (Test)
        function testParsesAndExpandsCarSimMacros(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            simfilePath = fullfile(fixture.Folder, 'simfile.sim');
            content = [
                "SIMFILE"
                "SET_MACRO $(ROOT_FILE_NAME)$ Run_abc"
                "SET_MACRO $(OUTPUT_PATH)$ E:\carsim\db\Results"
                "SET_MACRO $(WORK_DIR)$ E:\carsim\db\"
                "SET_MACRO $(OUTPUT_FILE_PREFIX)$ $(WORK_DIR)$Results\$(ROOT_FILE_NAME)$\LastRun"
                "FILEBASE $(OUTPUT_FILE_PREFIX)$"
                "LOGFILE $(OUTPUT_FILE_PREFIX)$_log.txt"
                "FINAL $(OUTPUT_FILE_PREFIX)$_end.par"
                "PROGDIR D:\Program Files (x86)\CarSim2024.1_Prog\"
                "PRODUCT_VER 2024.1"
                "EXT_MODEL_STEP 0.00050000"
                "END"
            ];
            writelines(content, simfilePath);

            info = parse_dyc_simfile(simfilePath);

            testCase.verifyEqual(info.rootFileName, 'Run_abc');
            testCase.verifyEqual(info.outputPath, 'E:\carsim\db\Results');
            testCase.verifyEqual(info.workDir, 'E:\carsim\db\');
            testCase.verifyEqual(info.outputFilePrefix, 'E:\carsim\db\Results\Run_abc\LastRun');
            testCase.verifyEqual(info.logFile, 'E:\carsim\db\Results\Run_abc\LastRun_log.txt');
            testCase.verifyEqual(info.endFile, 'E:\carsim\db\Results\Run_abc\LastRun_end.par');
            testCase.verifyEqual(info.progDir, 'D:\Program Files (x86)\CarSim2024.1_Prog\');
            testCase.verifyEqual(info.matlabSolverFolder, ...
                'D:\Program Files (x86)\CarSim2024.1_Prog\Programs\solvers\Matlab');
            testCase.verifyEqual(info.productVersion, '2024.1');
            testCase.verifyEqual(info.extModelStep_s, 0.0005);
        end

        function testMissingRequiredMacroReturnsInvalidInfo(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            simfilePath = fullfile(fixture.Folder, 'simfile.sim');
            writelines(["SIMFILE"; "END"], simfilePath);

            info = parse_dyc_simfile(simfilePath);

            testCase.verifyFalse(info.isValid);
            testCase.verifyNotEmpty(info.failureReason);
        end

        function testCyclicMacrosReturnInvalidInfo(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            simfilePath = fullfile(fixture.Folder, 'simfile.sim');
            content = [
                "SIMFILE"
                "SET_MACRO $(ROOT_FILE_NAME)$ Run_cycle"
                "SET_MACRO $(OUTPUT_PATH)$ E:\carsim\db\Results"
                "SET_MACRO $(WORK_DIR)$ E:\carsim\db\"
                "SET_MACRO $(CYCLE_A)$ $(CYCLE_B)$"
                "SET_MACRO $(CYCLE_B)$ $(CYCLE_A)$"
                "SET_MACRO $(OUTPUT_FILE_PREFIX)$ $(CYCLE_A)$\LastRun"
                "END"
            ];
            writelines(content, simfilePath);

            info = parse_dyc_simfile(simfilePath);

            testCase.verifyFalse(info.isValid);
            testCase.verifyNotEmpty(info.failureReason);
        end

        function testUnknownMacroReturnsInvalidInfo(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            simfilePath = fullfile(fixture.Folder, 'simfile.sim');
            content = [
                "SIMFILE"
                "SET_MACRO $(ROOT_FILE_NAME)$ Run_bad"
                "SET_MACRO $(OUTPUT_PATH)$ E:\carsim\db\Results"
                "SET_MACRO $(WORK_DIR)$ E:\carsim\db\"
                "SET_MACRO $(OUTPUT_FILE_PREFIX)$ $(WORK_DIR)$Results\$(BAD_MACRO)$\LastRun"
                "END"
            ];
            writelines(content, simfilePath);

            info = parse_dyc_simfile(simfilePath);

            testCase.verifyFalse(info.isValid);
            testCase.verifyNotEmpty(info.failureReason);
        end

        function testCarSimTimestampMacroDefaultsToLastRun(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            simfilePath = fullfile(fixture.Folder, 'simfile.sim');
            content = [
                "SIMFILE"
                "SET_MACRO $(ROOT_FILE_NAME)$ Run_abc"
                "SET_MACRO $(OUTPUT_PATH)$ E:\carsim\db\Results"
                "SET_MACRO $(WORK_DIR)$ E:\carsim\db\"
                "SET_MACRO $(OUTPUT_FILE_PREFIX)$ $(OUTPUT_PATH)$\$(ROOT_FILE_NAME)$\$(TIMESTAMP)$"
                "LOGFILE $(OUTPUT_FILE_PREFIX)$_log.txt"
                "FINAL $(OUTPUT_FILE_PREFIX)$_end.par"
                "END"
            ];
            writelines(content, simfilePath);

            info = parse_dyc_simfile(simfilePath);

            testCase.verifyTrue(info.isValid, info.failureReason);
            testCase.verifyEqual(info.outputFilePrefix, 'E:\carsim\db\Results\Run_abc\LastRun');
            testCase.verifyEqual(info.logFile, 'E:\carsim\db\Results\Run_abc\LastRun_log.txt');
            testCase.verifyEqual(info.endFile, 'E:\carsim\db\Results\Run_abc\LastRun_end.par');
        end

        function testUnknownRootFileNameMacroReturnsInvalidInfo(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            simfilePath = fullfile(fixture.Folder, 'simfile.sim');
            content = [
                "SIMFILE"
                "SET_MACRO $(ROOT_FILE_NAME)$ Run_$(UNKNOWN)$"
                "SET_MACRO $(OUTPUT_PATH)$ E:\carsim\db\Results"
                "SET_MACRO $(WORK_DIR)$ E:\carsim\db\"
                "SET_MACRO $(OUTPUT_FILE_PREFIX)$ E:\carsim\db\Results\Run_fixed\LastRun"
                "END"
            ];
            writelines(content, simfilePath);

            info = parse_dyc_simfile(simfilePath);

            testCase.verifyFalse(info.isValid);
            testCase.verifyNotEmpty(info.failureReason);
        end

        function testSelfReferentialMacroReturnsInvalidInfo(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            simfilePath = fullfile(fixture.Folder, 'simfile.sim');
            content = [
                "SIMFILE"
                "SET_MACRO $(ROOT_FILE_NAME)$ Run_self"
                "SET_MACRO $(OUTPUT_PATH)$ E:\carsim\db\Results"
                "SET_MACRO $(WORK_DIR)$ E:\carsim\db\"
                "SET_MACRO $(SELF_REF)$ $(SELF_REF)$suffix"
                "SET_MACRO $(OUTPUT_FILE_PREFIX)$ $(SELF_REF)$\LastRun"
                "END"
            ];
            writelines(content, simfilePath);

            info = parse_dyc_simfile(simfilePath);

            testCase.verifyFalse(info.isValid);
            testCase.verifyNotEmpty(info.failureReason);
        end
    end
end
