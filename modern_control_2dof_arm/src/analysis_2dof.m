function result = analysis_2dof(A, B, C, K)
%ANALYSIS_2DOF Analyze stability, controllability, observability, and Lyapunov stability.
%
% Inputs:
%   A, B, C - linearized state-space matrices
%   K       - LQR feedback gain
%
% Output:
%   result  - structure containing eigenvalues, ranks, Lyapunov matrix, and flags

result = struct();

result.eig_A = eig(A);

Co = ctrb(A, B);
result.Co = Co;
result.rank_Co = rank(Co);

Ob = obsv(A, C);
result.Ob = Ob;
result.rank_Ob = rank(Ob);

Ac = A - B*K;
result.Ac = Ac;
result.eig_Ac = eig(Ac);

% Continuous Lyapunov equation:
%   Ac'*P + P*Ac = -I
P = lyap(Ac', eye(4));
P = (P + P') / 2;  % numerical symmetrization

result.P = P;
result.eig_P = eig(P);

result.is_controllable = (result.rank_Co == 4);
result.is_observable = (result.rank_Ob == 4);
result.is_closed_loop_stable = all(real(result.eig_Ac) < 0);
result.is_P_positive_definite = all(real(result.eig_P) > 0);

end
