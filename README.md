# ME 455: Active Learning in Robotics (Northwestern, Spring 2025)

Two homework notebooks from the course. Both were written in Google Colab and
are saved here with their outputs, so the plots and printed numbers below can
be read without running anything.

| Notebook | Topic |
|---|---|
| `ilqr_unicycle.ipynb` | Trajectory optimisation for a unicycle with iLQR |
| `ergodic_trajectory_optimization.ipynb` | Ergodic coverage trajectory for a differential-drive robot |

## iLQR for a unicycle (`ilqr_unicycle.ipynb`)

**Problem.** A unicycle with state `(x, y, θ)` and controls `(u1, u2)` (forward
speed and turn rate) starts at `(0, 0, π/2)` and has to track a straight-line
reference `xd(t) = (2t/π, 0, π/2)` over `T = 2π` seconds. The cost is a
quadratic tracking error plus a quadratic control penalty, integrated over
time, plus a quadratic terminal error.

**Method.** This is the continuous-time form of iLQR, not the discrete
backward Riccati pass. Each iteration:

1. Simulate the current controls forward (Euler steps, `dt = 0.1`).
2. Linearise the dynamics along that trajectory (`A = ∂f/∂x`, `B = ∂f/∂u`) and
   take the cost gradients (`a = ∂ℓ/∂x`, `b = ∂ℓ/∂u`).
3. Solve a two-point boundary value problem with `scipy.integrate.solve_bvp`
   for the state perturbation `z(t)` and the costate `p(t)`. `z` starts at
   zero and `p` is fixed at the final time by the terminal cost gradient.
4. Take the control correction `v = -R_v⁻¹(Bᵀp + b)`.
5. Choose the step size with an Armijo backtracking line search (halve the
   step up to 20 times) and update the controls.

The loop stops when the cost changes by less than `1e-4` between iterations,
or after 60 iterations.

**Results.** The initial guess is a constant control held for the whole horizon.

| Run | Weights (diagonals) | `dt` | Initial guess | Cost, start → end | Iterations | Converged |
|---|---|---|---|---|---|---|
| Variation 1 (first cell) | `Q_x = (10, 10, 2)`, `R_u = (4, 2)`, `P1 = (20, 20, 5)` | 0.1 | `(1.0, 0.5)` | 2842.5 → 44.6 | 39 | yes |
| Variation 2, heavier heading weight | `Q_x = (5, 5, 20)`, `R_u = (1, 5)`, `P1 = (5, 5, 30)` | 0.05 | `(0.8, -0.3)` | 440.4 → 67.6 | 150 (limit) | no |
| Variation 3, heavier control penalty | `Q_x = (1, 1, 1)`, `R_u = (10, 10)`, `P1 = (1, 1, 1)` | 0.05 | `(1.2, 0.2)` | 389.8 → 38.9 | 35 | yes |

All three use `Q_z = (5, 5, 1)` and `R_v = (2, 1)` for the descent-direction
subproblem. Costs are not comparable across rows because the weights differ.
Each run saves three plots: initial and final path, the final controls, and
cost against iteration.

## Ergodic trajectory optimisation (`ergodic_trajectory_optimization.ipynb`)

**Problem.** A differential-drive robot with state `(x, y, θ)` and controls
`(v, ω)` starts at `(0.3, 0.3, π/2)` in the unit square and moves for 10
seconds. The goal is a trajectory whose time spent in each region is
proportional to a target density, a mixture of three Gaussians with weights
0.5, 0.2 and 0.3.

**Method.**

1. Evaluate the target density on a 200 × 200 grid and compute its Fourier
   cosine coefficients for the 10 × 10 lowest modes.
2. Simulate the robot from the controls (Euler steps, `dt = 0.1`, 100 steps)
   and compute the same cosine coefficients as a time average along the
   trajectory.
3. The ergodic metric is the weighted sum of squared differences between the
   two sets of coefficients, with weight `1 / (1 + i² + j²)` on mode `(i, j)`
   so that low frequencies count more.
4. The whole computation is written in PyTorch, so the gradient of the metric
   with respect to the controls comes from autodiff. Adam (learning rate
   `1e-2`) updates the controls for 300 iterations.

**Results.** Three runs, each from a different small random initial control
sequence.

| Run | Ergodic metric at iteration 1 | At iteration 300 |
|---|---|---|
| 1 | 1.111 | 0.026 |
| 2 | 0.798 | 0.026 |
| 3 | 0.998 | 0.020 |

Each run saves a plot of the trajectory over the target density, the
controls, and the metric against iteration.

## Course-provided

- **iLQR notebook.** Built on the instructor's template, which supplied the
  parameter block (time step, initial state, initial control guess, weight
  matrices), the empty function shells, and the `solve_bvp` scaffold with the
  outer iteration loop. My part: the unicycle dynamics and both Jacobians, the
  cost and its gradients, the terminal and boundary conditions, the contents
  of the boundary value problem matrices, the control update, the Armijo line
  search, the cost history and plots, and the two extra weight variations.
- **Ergodic notebook.** The definition of the ergodic metric and the target
  distribution come from the course tutorial by Max Muchen Sun. My part: the
  differential-drive rollout, the PyTorch implementation of the metric, the
  optimisation loop, and the plots.
- Both notebooks were written with AI assistance. The iLQR boundary value
  problem blocks and cost function were corrected and re-verified in October
  2026.

## Limitations

- **iLQR stops on a stalled line search, not at a verified minimum.** In the
  first cell the last two step sizes are about `1e-5` while the correction `v`
  is still far from zero, so the cost stops changing because the steps become
  tiny. Variation 2 is still decreasing when it reaches the 150-iteration
  limit.
- **Euler integration.** Both notebooks integrate the dynamics with fixed-step
  forward Euler, so the trajectories carry first-order discretisation error.
- **Ergodic results are single-seed runs.** The seed is set once and the three
  runs differ only in their random initial controls. There is no statistic
  over seeds, and the metric is still falling at iteration 300.
- **Positions are clamped to the unit square** when the ergodic metric is
  computed, so a trajectory that leaves the square is not penalised for it.

## Running

```
pip install -r requirements.txt
jupyter notebook
```

The ergodic notebook needs PyTorch and runs on CPU.
