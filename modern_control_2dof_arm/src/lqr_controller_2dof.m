function [K, S, poles, Q, R] = lqr_controller_2dof(A, B)
%LQR_CONTROLLER_2DOF Design an LQR controller for the linearized 2-DOF arm.
%
% Inputs:
%   A, B - linearized state and input matrices
%
% Outputs:
%   K     - LQR state-feedback gain
%   S     - solution of the continuous algebraic Riccati equation
%   poles - closed-loop poles returned by lqr
%   Q, R  - LQR weighting matrices

Q = diag([100, 100, 10, 10]);
R = diag([1, 1]);

[K, S, poles] = lqr(A, B, Q, R);

end
