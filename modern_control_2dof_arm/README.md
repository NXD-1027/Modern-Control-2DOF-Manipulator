# 二自由度平面机械臂的状态空间建模、稳定性分析与 LQR 控制仿真

This MATLAB project implements a complete modern control theory workflow for a small 2-DOF planar robotic manipulator:

1. mechanism-based modeling
2. operating-point linearization
3. state-space model construction
4. open-loop stability analysis
5. controllability analysis
6. observability analysis
7. LQR controller design
8. Lyapunov stability verification
9. MATLAB simulation
10. export of report-ready figures and numerical data

The project does not use GUI, Simulink, CVX, or complex trajectory tracking.

## Control object

The control object is a two-link planar manipulator with two revolute joints. The joint variables are

```text
q = [q1; q2]
```

where `q1` is the angle of link 1 relative to the positive horizontal axis, and `q2` is the relative angle of link 2 with respect to link 1.

The selected operating point is

```text
q0 = [30 deg; -120 deg]
```

so that

```text
q1 + q2 = -90 deg
```

which means link 2 points vertically downward at the operating point.

The perturbation-state vector is

```text
Delta x = [Delta q1; Delta q2; Delta dq1; Delta dq2]
```

and the perturbation input is

```text
Delta u = [Delta tau1; Delta tau2]
```

The actual joint torque is interpreted as

```text
tau = tau0 + Delta u = G(q0) - K*[q - q0; dq]
```

## File structure

```text
modern_control_2dof_arm/
├── src/
│   ├── model_2dof_params.m
│   ├── dynamics_2dof_linearized.m
│   ├── lqr_controller_2dof.m
│   ├── analysis_2dof.m
│   ├── simulate_2dof.m
│   ├── plot_results_2dof.m
│   └── draw_arm_2dof.m
├── figures/
├── results/
├── report_assets/
├── main_2dof_lqr.m
└── README.md
```

## How to run

Open MATLAB, set `modern_control_2dof_arm/` as the current folder, and run:

```matlab
main_2dof_lqr
```

The main script automatically adds the `src/` folder to the MATLAB path, creates missing output folders, runs the modeling and control workflow, saves figures, and exports numerical results.

## MATLAB dependency

This project requires MATLAB Control System Toolbox because it uses standard modern-control functions:

- `lqr`
- `ss`
- `initial`
- `ctrb`
- `obsv`
- `lyap`

If MATLAB reports that any of these functions are missing, install or enable Control System Toolbox.

## Generated figures

After running `main_2dof_lqr.m`, the following PNG and FIG files are generated in `figures/`:

| Figure | File name | Description |
| --- | --- | --- |
| 1 | `fig1_arm_structure.png/.fig` | 2-DOF manipulator at the operating point |
| 2 | `fig2_open_loop_response.png/.fig` | Open-loop state response |
| 3 | `fig3_closed_loop_response.png/.fig` | Closed-loop state response under LQR |
| 4 | `fig4_lqr_torque.png/.fig` | LQR control input `Delta tau` |
| 5 | `fig5_pole_map.png/.fig` | Open-loop and closed-loop pole map |
| 6 | `fig6_initial_vs_target_pose.png/.fig` | Initial perturbed pose vs target operating pose |

PNG files are intended for the course report. FIG files are provided for later editing in MATLAB.

## Generated result files

The following files are generated in `results/`:

| File | Description |
| --- | --- |
| `sim_results.mat` | MATLAB data file containing model matrices, LQR gain, analysis results, and simulation data |
| `summary.txt` | Text summary of model parameters, matrices, eigenvalues, controllability/observability ranks, and Lyapunov checks |

## Reference note

This project only refers to two open-source projects at the level of organization and visualization ideas:

- `LQR-4-DOF-Manipulator`: referenced for MATLAB project organization, LQR calling style, simulation workflow, and result-analysis organization.
- `Simulate-2DOF-Robot-Arm`: referenced for 2-DOF planar manipulator geometry and visualization ideas.

The dynamics, operating-point linearization, system analysis, LQR simulation, Lyapunov verification, and report-oriented outputs in this repository are independently reimplemented for the modern control theory course assignment.

## Model simplification note

The nonlinear manipulator model is written as

```text
M(q)*ddq + C(q,dq)*dq + G(q) + Dv*dq = tau
```

At the operating point, `dq0 = 0` and `tau0 = G(q0)`. Since `C(q,dq)*dq` is a second-order small quantity around this stationary operating point, its first-order contribution is ignored in the linearized perturbation model:

```text
M0*Delta_ddq + Dv*Delta_dq + Kg*Delta_q = Delta_tau
```

This gives the linearized state-space model used for LQR design and stability analysis.
