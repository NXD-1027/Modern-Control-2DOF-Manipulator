function plot_results_2dof(params, model, sim, result, K, projectRoot)
%PLOT_RESULTS_2DOF Generate and save all figures required for the report.
%
% Inputs:
%   params      - model and simulation parameters
%   model       - linearization model structure
%   sim         - simulation result structure
%   result      - analysis result structure
%   K           - LQR gain matrix
%   projectRoot - path to the project root folder

figDir = fullfile(projectRoot, 'figures');
if ~exist(figDir, 'dir')
    mkdir(figDir);
end

q0 = params.q0;
x0 = params.x0;

%% Figure 1: operating-point arm structure
fig = figure('Name', '2-DOF operating point', 'Color', 'w');
ax = axes(fig);
draw_arm_2dof(q0, params, 'Parent', ax, 'DisplayName', 'Operating point', ...
    'Color', [0 0.4470 0.7410], 'ShowLabels', true);
axis(ax, 'equal');
grid(ax, 'on');
xlabel(ax, 'x (m)');
ylabel(ax, 'y (m)');
title(ax, '2-DOF planar manipulator at operating point');
legend(ax, 'Location', 'best');
set_arm_axis_limits(ax, params);
save_figure_pair(fig, figDir, 'fig1_arm_structure');

%% Figure 2: open-loop response
fig = figure('Name', 'Open-loop response', 'Color', 'w');
subplot(2,1,1);
plot(sim.t_open, sim.x_open(:,1), 'LineWidth', 1.5); hold on;
plot(sim.t_open, sim.x_open(:,2), 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Angle perturbation (rad)');
title('Open-loop angle response');
legend('\Delta q_1', '\Delta q_2', 'Location', 'best');

subplot(2,1,2);
plot(sim.t_open, sim.x_open(:,3), 'LineWidth', 1.5); hold on;
plot(sim.t_open, sim.x_open(:,4), 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Angular velocity perturbation (rad/s)');
title('Open-loop angular velocity response');
legend('\Delta dq_1', '\Delta dq_2', 'Location', 'best');
save_figure_pair(fig, figDir, 'fig2_open_loop_response');

%% Figure 3: closed-loop response
fig = figure('Name', 'Closed-loop response', 'Color', 'w');
subplot(2,1,1);
plot(sim.t_cl, sim.x_cl(:,1), 'LineWidth', 1.5); hold on;
plot(sim.t_cl, sim.x_cl(:,2), 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Angle perturbation (rad)');
title('Closed-loop angle response under LQR');
legend('\Delta q_1', '\Delta q_2', 'Location', 'best');

subplot(2,1,2);
plot(sim.t_cl, sim.x_cl(:,3), 'LineWidth', 1.5); hold on;
plot(sim.t_cl, sim.x_cl(:,4), 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Angular velocity perturbation (rad/s)');
title('Closed-loop angular velocity response under LQR');
legend('\Delta dq_1', '\Delta dq_2', 'Location', 'best');
save_figure_pair(fig, figDir, 'fig3_closed_loop_response');

%% Figure 4: LQR control input
fig = figure('Name', 'LQR torque input', 'Color', 'w');
plot(sim.t_cl, sim.u(:,1), 'LineWidth', 1.5); hold on;
plot(sim.t_cl, sim.u(:,2), 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Torque perturbation (N*m)');
title('LQR control input \Delta\tau = -K\Delta x');
legend('\Delta\tau_1', '\Delta\tau_2', 'Location', 'best');
save_figure_pair(fig, figDir, 'fig4_lqr_torque');

%% Figure 5: pole map
fig = figure('Name', 'Pole map', 'Color', 'w');
plot(real(result.eig_A), imag(result.eig_A), 'x', 'LineWidth', 2, 'MarkerSize', 9); hold on;
plot(real(result.eig_Ac), imag(result.eig_Ac), 'o', 'LineWidth', 2, 'MarkerSize', 7);
xline(0, '--');
grid on;
xlabel('Real axis');
ylabel('Imaginary axis');
title('Open-loop and closed-loop pole map');
legend('Open-loop poles', 'Closed-loop poles', 'Imaginary axis', 'Location', 'best');
save_figure_pair(fig, figDir, 'fig5_pole_map');

%% Figure 6: initial perturbation pose vs target operating pose
fig = figure('Name', 'Initial vs target pose', 'Color', 'w');
ax = axes(fig);
q_target = q0;
q_initial = q0 + x0(1:2);
draw_arm_2dof(q_target, params, 'Parent', ax, 'DisplayName', 'Target pose q_0', ...
    'Color', [0 0.4470 0.7410], 'LineStyle', '-', 'ShowLabels', false);
draw_arm_2dof(q_initial, params, 'Parent', ax, 'DisplayName', 'Initial pose q_0 + \Delta q(0)', ...
    'Color', [0.8500 0.3250 0.0980], 'LineStyle', '--', 'ShowLabels', false);
axis(ax, 'equal');
grid(ax, 'on');
xlabel(ax, 'x (m)');
ylabel(ax, 'y (m)');
title(ax, 'Initial perturbed pose vs target operating pose');
legend(ax, 'Location', 'best');
set_arm_axis_limits(ax, params);
save_figure_pair(fig, figDir, 'fig6_initial_vs_target_pose');

if isempty(K) || isempty(model)
    warning('K or model is empty. Figures were still generated from available data.');
end

end

function save_figure_pair(fig, figDir, baseName)
%SAVE_FIGURE_PAIR Save one figure as PNG and FIG.
pngPath = fullfile(figDir, [baseName, '.png']);
figPath = fullfile(figDir, [baseName, '.fig']);

try
    exportgraphics(fig, pngPath, 'Resolution', 300);
catch
    saveas(fig, pngPath);
end
savefig(fig, figPath);
end

function set_arm_axis_limits(ax, params)
%SET_ARM_AXIS_LIMITS Set symmetric axis limits for the planar arm.
reach = params.l1 + params.l2;
margin = 0.08;
xlim(ax, [-reach - margin, reach + margin]);
ylim(ax, [-reach - margin, reach + margin]);
end
