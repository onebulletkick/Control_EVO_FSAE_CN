function solverFolder = add_dyc_carsim_solver_path(cfg)
%ADD_DYC_CARSIM_SOLVER_PATH Add CarSim MATLAB solver folder from simfile.sim.

solverFolder = "";
if ~isfield(cfg, 'simfilePath') || ~isfile(cfg.simfilePath)
    return;
end

simfileInfo = parse_dyc_simfile(cfg.simfilePath);
if ~isfield(simfileInfo, 'matlabSolverFolder') || strlength(string(simfileInfo.matlabSolverFolder)) == 0
    return;
end

solverFolder = string(simfileInfo.matlabSolverFolder);
if isfolder(solverFolder) && ~contains(path, char(solverFolder))
    addpath(char(solverFolder));
end
end
