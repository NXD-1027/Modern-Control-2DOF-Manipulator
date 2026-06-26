%MAIN_2DOF_LQR Entry point for the 2-DOF planar manipulator LQR project.
%
% This script performs the full modern-control workflow:
% mechanism-based modeling, operating-point linearization, state-space
% analysis, LQR design, Lyapunov verification, simulation, plotting, and
% result export.

clear; clc; close all;

projectRoot = fileparts(mfilename('fullpath'));
srcDir = fullfile(projectRoot, 'src');
figDir = fullfile(projectRoot, 'figures');
resultsDir = fullfile(projectRoot, 'results');
reportAssetsDir = fullfile(projectRoot, 'report_assets');

addpath(srcDir);
ensure_dir(figDir);
ensure_dir(resultsDir);
ensure_dir(reportAssetsDir);

[A, B, C, D, params, model] = dynamics_2dof_linearized();
[K, S, poles, Q, R] = lqr_controller_2dof(A, B);
result = analysis_2dof(A, B, C, K);
sim = simulate_2dof(A, B, K, params);

plot_results_2dof(params, model, sim, result, K, projectRoot);

save(fullfile(resultsDir, 'sim_results.mat'), ...
    'A', 'B', 'C', 'D', 'params', 'model', 'K', 'S', 'poles', 'Q', 'R', 'result', 'sim');

summaryPath = fullfile(resultsDir, 'summary.txt');
fid = fopen(summaryPath, 'w');
if fid == -1
    error('Cannot open summary file for writing: %s', summaryPath);
end

print_summary(1, A, B, C, D, params, model, K, Q, R, result);
print_summary(fid, A, B, C, D, params, model, K, Q, R, result);
fclose(fid);

fprintf('\nFigures saved to: %s\n', figDir);
fprintf('Results saved to: %s\n', resultsDir);
fprintf('Summary written to: %s\n', summaryPath);

function ensure_dir(dirPath)
%ENSURE_DIR Create a directory if it does not exist.
if ~exist(dirPath, 'dir')
    mkdir(dirPath);
end
end

function print_summary(fid, A, B, C, D, params, model, K, Q, R, result)
%PRINT_SUMMARY Print key project values to command window or text file.
fprintf(fid, '============================================================\n');
fprintf(fid, '2-DOF Planar Manipulator Modern Control Summary\n');
fprintf(fid, '============================================================\n\n');

fprintf(fid, 'Physical parameters:\n');
fprintf(fid, 'm1 = %.6f kg\n', params.m1);
fprintf(fid, 'm2 = %.6f kg\n', params.m2);
fprintf(fid, 'l1 = %.6f m\n', params.l1);
fprintf(fid, 'l2 = %.6f m\n', params.l2);
fprintf(fid, 'lc1 = %.6f m\n', params.lc1);
fprintf(fid, 'lc2 = %.6f m\n', params.lc2);
fprintf(fid, 'I1 = %.6f kg*m^2\n', params.I1);
fprintf(fid, 'I2 = %.6f kg*m^2\n', params.I2);
fprintf(fid, 'g = %.6f m/s^2\n\n', params.g);

print_matrix(fid, 'q0 (rad)', params.q0);
print_matrix(fid, 'q0 (deg)', rad2deg(params.q0));
print_matrix(fid, 'x0 = [Delta q1; Delta q2; Delta dq1; Delta dq2]', params.x0);
print_matrix(fid, 'Dv', params.Dv);

print_matrix(fid, 'M0', model.M0);
print_matrix(fid, 'Kg', model.Kg);
print_matrix(fid, 'G0', model.G0);
print_matrix(fid, 'tau0', model.tau0);

print_matrix(fid, 'A', A);
print_matrix(fid, 'B', B);
print_matrix(fid, 'C', C);
print_matrix(fid, 'D', D);

print_matrix(fid, 'Q', Q);
print_matrix(fid, 'R', R);
print_matrix(fid, 'K', K);

print_matrix(fid, 'open-loop eigenvalues eig(A)', result.eig_A);
fprintf(fid, 'rank(Co) = %d\n', result.rank_Co);
fprintf(fid, 'rank(Ob) = %d\n', result.rank_Ob);
print_matrix(fid, 'closed-loop eigenvalues eig(A-B*K)', result.eig_Ac);
print_matrix(fid, 'eig(P)', result.eig_P);

fprintf(fid, 'is_controllable = %d\n', result.is_controllable);
fprintf(fid, 'is_observable = %d\n', result.is_observable);
fprintf(fid, 'is_closed_loop_stable = %d\n', result.is_closed_loop_stable);
fprintf(fid, 'is_P_positive_definite = %d\n', result.is_P_positive_definite);
fprintf(fid, '\n');

fprintf(fid, 'Dimension checks:\n');
fprintf(fid, 'size(A) = %dx%d\n', size(A,1), size(A,2));
fprintf(fid, 'size(B) = %dx%d\n', size(B,1), size(B,2));
fprintf(fid, 'size(C) = %dx%d\n', size(C,1), size(C,2));
fprintf(fid, 'size(D) = %dx%d\n', size(D,1), size(D,2));
fprintf(fid, '============================================================\n\n');
end

function print_matrix(fid, name, X)
%PRINT_MATRIX Print a matrix with a label.
fprintf(fid, '%s =\n', name);
for i = 1:size(X, 1)
    for j = 1:size(X, 2)
        val = X(i, j);
        if abs(imag(val)) > 1e-12
            fprintf(fid, '  % .8e%+ .8ei', real(val), imag(val));
        else
            fprintf(fid, '  % .8e', real(val));
        end
    end
    fprintf(fid, '\n');
end
fprintf(fid, '\n');
end
