# UAV Control — Iterative Learning Control for Quadrotor Flight

Iterative Learning Control (ILC) applied to quadrotor UAV flight-path tracking,
in MATLAB and Simulink. The project derives a state-space model of a quadrotor,
develops several multi-input / multi-output ILC update laws, and evaluates them
in simulation against a higher-fidelity Crazyflie model.

## Summary

- Derived and modelled quadrotor flight dynamics as a linear state-space system.
- Implemented several MIMO ILC update laws, including a point-to-point (P2P) ILC
  variant that tracks only specified waypoints rather than the whole trajectory.
- Compared 3 learning models over 150 iterations; **best-case tracking-error
  reduction of 33%**.
- Validated the controllers on a higher-fidelity Crazyflie quadrotor simulation,
  with automated sweeps to tune the ILC weighting matrices Q and R.

## Background

Iterative Learning Control improves tracking of a repeated task by using the
error from previous executions to refine the command applied on the next one.
Over successive iterations the tracking error converges, which makes ILC well
suited to UAVs flying repeated or pre-planned trajectories. This project builds
the quadrotor model, applies ILC (including a point-to-point formulation), and
measures how quickly and how far the error converges.

## Repository structure

### Phase 2 — analytical model and preliminary ILC
| File | Description |
|------|-------------|
| `Drone4into12.m` | Runs the analytically derived quadrotor model and applies the ILC update laws (includes additional ILC methods not used in the final report). |
| `p2p.m` | Point-to-point (P2P) enabled version of `Drone4into12`, with additional P2P methods. |
| `ILCLinear.m` | Applies the `Drone4into12` ILC updates onto the Simulink model `linearmod`. |
| `linearmod.slx` | Simulink model of the analytically derived state-space quadrotor. |

### Phase 3 — Crazyflie simulation and tuning
| File | Description |
|------|-------------|
| `CrazyflieSimulationPID_20ablock2.slx` | Simulink model of the Crazyflie quadrotor used for testing. Runs without modification. |
| `CrazyflieSimulationParameters.m` | Simulation parameters. **Load this before running the Crazyflie tests.** |
| `CrazyP2P.m` | Main test script — runs either no-ILC or P2P ILC on the Crazyflie model. |
| `Tuning_script.m` | Variant of `CrazyP2P` used by the Q and R tuners (also runs the same tests directly). |
| `Q_tuner.m` | Automated sweep over a range of Q weighting values; writes a `.mat` result per test. |
| `R_tuner.m` | Automated sweep over a range of R weighting values; writes a `.mat` result per test. |

## Requirements

- MATLAB with Simulink
- Control System Toolbox (state-space modelling and control design)

## How to run

**Phase 2 (analytical model):**
```matlab
Drone4into12      % analytical model + ILC
p2p               % point-to-point ILC variant
ILCLinear         % apply ILC updates onto linearmod.slx
```

**Phase 3 (Crazyflie simulation):**
```matlab
CrazyflieSimulationParameters   % load parameters first
CrazyP2P                        % run no-ILC or P2P ILC on the Crazyflie model
```

**Parameter tuning:**
```matlab
Q_tuner           % sweep Q weighting values  -> mydata.mat per run
R_tuner           % sweep R weighting values  -> mydata.mat per run
```

## Notes

- This began as an individual university project. The accompanying report
  contains the full model derivation and results discussion.
- Some scripts include ILC methods that were explored but not used in the final
  report; these are kept for reference.
