function h = draw_arm_2dof(q, params, varargin)
%DRAW_ARM_2DOF Draw a planar 2-DOF arm for a given absolute joint state.
%
% Inputs:
%   q      - actual joint angles [q1; q2], not perturbations
%   params - parameter structure containing l1 and l2
%
% Name-value options:
%   'Parent'      - axes handle
%   'Color'       - line color
%   'LineStyle'   - line style
%   'LineWidth'   - line width
%   'DisplayName' - legend display name
%   'ShowLabels'  - true/false, label joints and end-effector
%
% Output:
%   h - structure of plot handles and point coordinates

parser = inputParser;
parser.addParameter('Parent', [], @(x) isempty(x) || isgraphics(x, 'axes'));
parser.addParameter('Color', [0 0.4470 0.7410]);
parser.addParameter('LineStyle', '-');
parser.addParameter('LineWidth', 2.5);
parser.addParameter('DisplayName', 'Arm');
parser.addParameter('ShowLabels', false, @(x) islogical(x) || isnumeric(x));
parser.parse(varargin{:});
opts = parser.Results;

if isempty(opts.Parent)
    ax = gca;
else
    ax = opts.Parent;
end

q1 = q(1);
q2 = q(2);
l1 = params.l1;
l2 = params.l2;

p0 = [0; 0];
p1 = [l1*cos(q1); l1*sin(q1)];
p2 = [l1*cos(q1) + l2*cos(q1 + q2);
      l1*sin(q1) + l2*sin(q1 + q2)];

hold(ax, 'on');

h.armLine = plot(ax, [p0(1), p1(1), p2(1)], [p0(2), p1(2), p2(2)], ...
    'Color', opts.Color, 'LineStyle', opts.LineStyle, 'LineWidth', opts.LineWidth, ...
    'Marker', 'o', 'MarkerSize', 6, 'MarkerFaceColor', opts.Color, ...
    'DisplayName', opts.DisplayName);

h.base = plot(ax, p0(1), p0(2), 'ks', 'MarkerSize', 7, 'MarkerFaceColor', 'k', ...
    'HandleVisibility', 'off');
h.joint2 = plot(ax, p1(1), p1(2), 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'w', ...
    'HandleVisibility', 'off');
h.ee = plot(ax, p2(1), p2(2), 'ko', 'MarkerSize', 6, 'MarkerFaceColor', opts.Color, ...
    'HandleVisibility', 'off');

if opts.ShowLabels
    text(ax, p0(1), p0(2), '  Joint 1', 'FontSize', 10, 'VerticalAlignment', 'bottom');
    text(ax, p1(1), p1(2), '  Joint 2', 'FontSize', 10, 'VerticalAlignment', 'bottom');
    text(ax, p2(1), p2(2), '  End-effector', 'FontSize', 10, 'VerticalAlignment', 'bottom');
end

h.p0 = p0;
h.p1 = p1;
h.p2 = p2;

end
