function params = model_2dof_params()
%MODEL_2DOF_PARAMS Physical and simulation parameters for the 2-DOF arm.
%
% The chosen operating point is q0 = [30 deg; -120 deg]. With the
% common planar-arm convention, q1 is the absolute angle of link 1 and q2
% is the relative angle of link 2. Therefore q1 + q2 = -90 deg, which
% means link 2 points vertically downward at the operating point.

params.m1 = 3.0;     % kg, mass of link 1 including equivalent mounted parts
params.m2 = 2.5;     % kg, mass of link 2 including equivalent mounted parts

params.l1 = 0.30;    % m, length of link 1
params.l2 = 0.25;    % m, length of link 2

params.lc1 = params.l1 / 2;    % m, center of mass of link 1
params.lc2 = params.l2 / 2;    % m, center of mass of link 2

% Uniform slender-rod inertia about each link center of mass.
params.I1 = (1/12) * params.m1 * params.l1^2;
params.I2 = (1/12) * params.m2 * params.l2^2;

params.g = 9.81;     % m/s^2

% Joint viscous damping matrix.
params.Dv = diag([0.06, 0.05]);

% Operating point.
params.q0 = deg2rad([30; -120]);
params.dq0 = [0; 0];

% Initial perturbation around the operating point:
% delta_x0 = [delta_q1; delta_q2; delta_dq1; delta_dq2].
params.x0 = [0.10; -0.05; 0; 0];

% Simulation time vector.
params.t = 0:0.01:5;

% LQR weighting matrices.
params.Q = diag([100, 100, 10, 10]);
params.R = diag([1, 1]);

end
