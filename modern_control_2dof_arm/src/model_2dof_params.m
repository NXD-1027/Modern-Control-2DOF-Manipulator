function params = model_2dof_params()
%MODEL_2DOF_PARAMS Return physical, operating-point, and simulation parameters.
%
% Output:
%   params - structure containing link parameters, damping, operating point,
%            initial perturbation, and simulation time vector.

params.m1 = 3.0;
params.m2 = 2.5;

params.l1 = 0.30;
params.l2 = 0.25;

params.lc1 = params.l1 / 2;
params.lc2 = params.l2 / 2;

params.I1 = (1/12) * params.m1 * params.l1^2;
params.I2 = (1/12) * params.m2 * params.l2^2;

params.g = 9.81;

params.Dv = diag([0.06, 0.05]);

% 工作点：第一连杆抬起 30 deg，第二连杆绝对方向竖直向下
% 角度定义：q1 是第一连杆相对水平正方向的角度，q2 是第二连杆相对第一连杆的角度
% 因此第二连杆绝对角为 q1 + q2 = -90 deg
params.q0 = deg2rad([30; -120]);

params.dq0 = [0; 0];

% 初始小扰动，状态顺序为 [Delta q1; Delta q2; Delta dq1; Delta dq2]
params.x0 = [0.10; -0.05; 0; 0];

params.t = 0:0.01:5;

end
