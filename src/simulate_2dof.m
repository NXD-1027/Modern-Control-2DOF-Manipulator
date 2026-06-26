function sim = simulate_2dof(A, B, K, params)
%SIMULATE_2DOF Simulate open-loop and closed-loop initial-condition responses.
%
% Inputs:
%   A, B   - linearized state-space matrices
%   K      - LQR state-feedback gain
%   params - parameter structure containing x0 and t
%
% Output:
%   sim    - structure containing time vectors, state responses, and control input

x0 = params.x0;
t = params.t;

% For initial-condition response, the external input is not used. A dummy
% zero-input channel is kept here for broad compatibility with MATLAB R2023
% and Control System Toolbox state-space object construction.
B_dummy = zeros(4, 1);
D_dummy = zeros(4, 1);

% Open-loop response: dx/dt = A*x
sys_open = ss(A, B_dummy, eye(4), D_dummy);
[y_open, t_open, x_open_state] = initial(sys_open, x0, t);
if isempty(x_open_state)
    x_open = y_open;
else
    x_open = x_open_state;
end

% Closed-loop response: dx/dt = (A - B*K)*x
Ac = A - B*K;
sys_cl = ss(Ac, B_dummy, eye(4), D_dummy);
[y_cl, t_cl, x_cl_state] = initial(sys_cl, x0, t);
if isempty(x_cl_state)
    x_cl = y_cl;
else
    x_cl = x_cl_state;
end

% LQR perturbation torque Delta u = -K*Delta x.
u = -(K * x_cl')';

sim.t_open = t_open;
sim.x_open = x_open;
sim.t_cl = t_cl;
sim.x_cl = x_cl;
sim.u = u;

end
