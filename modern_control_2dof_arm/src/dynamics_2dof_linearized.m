function [A, B, C, D, params, model] = dynamics_2dof_linearized()
%DYNAMICS_2DOF_LINEARIZED Build the linearized state-space model around q0.
%
% Outputs:
%   A, B, C, D - continuous-time state-space matrices for Delta x and Delta u
%   params     - physical and simulation parameter structure
%   model      - structure containing M0, G0, tau0, Kg, and related data
%
% Nonlinear model:
%   M(q)*ddq + C(q,dq)*dq + G(q) + Dv*dq = tau
%
% Around dq0 = 0 and tau0 = G(q0), the first-order perturbation model is:
%   M0*Delta_ddq + Dv*Delta_dq + Kg*Delta_q = Delta_tau

params = model_2dof_params();
q0 = params.q0;

m1 = params.m1;
m2 = params.m2;
l1 = params.l1;
lc1 = params.lc1;
lc2 = params.lc2;
I1 = params.I1;
I2 = params.I2;
g = params.g;
Dv = params.Dv;

q1 = q0(1);
q2 = q0(2);

% Inertia matrix M(q0)
M11 = I1 + I2 + m1*lc1^2 + m2*(l1^2 + lc2^2 + 2*l1*lc2*cos(q2));
M12 = I2 + m2*(lc2^2 + l1*lc2*cos(q2));
M22 = I2 + m2*lc2^2;
M0 = [M11, M12;
      M12, M22];

% Gravity vector G(q0)
G1 = (m1*lc1 + m2*l1)*g*cos(q1) + m2*lc2*g*cos(q1 + q2);
G2 = m2*lc2*g*cos(q1 + q2);
G0 = [G1; G2];

% Static torque required to hold the operating point.
tau0 = G0;

% Gravity stiffness matrix Kg = dG/dq at q0
Kg11 = -(m1*lc1 + m2*l1)*g*sin(q1) - m2*lc2*g*sin(q1 + q2);
Kg12 = -m2*lc2*g*sin(q1 + q2);
Kg21 = -m2*lc2*g*sin(q1 + q2);
Kg22 = -m2*lc2*g*sin(q1 + q2);
Kg = [Kg11, Kg12;
      Kg21, Kg22];

A = [zeros(2), eye(2);
     -(M0\Kg), -(M0\Dv)];

B = [zeros(2);
     M0\eye(2)];

C = [1 0 0 0;
     0 1 0 0];

D = zeros(2, 2);

model.M0 = M0;
model.G0 = G0;
model.tau0 = tau0;
model.Kg = Kg;
model.q0 = q0;
model.dq0 = params.dq0;
model.Dv = Dv;
model.description = 'Linearized 2-DOF planar manipulator model around q0.';

end
