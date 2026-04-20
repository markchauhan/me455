# ME 455 — Active Learning in Robotics
**Northwestern University · McCormick School of Engineering**

Course covering the mathematical foundations and implementations of active learning for robotic systems — combining optimal control, information theory, and probabilistic inference to build robots that learn efficiently from their own experience.

---

## Projects & Assignments

### Final Project — Ergodic Active Search
A sensor agent with 2D single-integrator dynamics autonomously searches for a hidden rectangular object in an unknown environment. The agent maintains a Bayesian belief over the object's location and steers its trajectory using **ergodic control** — a technique that drives the agent's time-averaged spatial statistics to match its information distribution, maximizing coverage of uncertain regions.

**Key concepts:**
- Ergodic control via Fourier decomposition of belief distributions
- Bayesian inference updated from binary sensor observations
- Online replanning as the belief evolves

### iLQR — Iterative Linear Quadratic Regulator
From-scratch implementation of iLQR for trajectory optimization on a nonlinear unicycle system. Analytically derived system Jacobians, implemented the backward-pass Riccati recursion and forward-pass line search, and swept cost weight variations to characterize trajectory sensitivity.

**Key concepts:**
- Iterative LQR for nonlinear systems
- Analytical Jacobian computation
- Cost weight analysis (position vs. heading vs. control penalty)

### Ergodic Metric
Implementation of the ergodic metric using Fourier basis functions over a 2D search space. Computes spatial distribution coefficients via numerical integration, matches them against trajectory empirical distributions, and quantifies coverage quality with a weighted spectral distance.

### Optimal Control (LQR Variants)
Continuous-time LQR implementations with boundary value problem solvers. Explores trade-offs between state tracking, heading accuracy, and control effort across multiple cost function parameterizations.

---

## Topics Covered
- Ergodic control and information-theoretic motion planning
- Iterative LQR and differential dynamic programming
- Bayesian active learning and belief-space planning
- Fourier analysis for spatial probability representation
- Optimal control theory (continuous and discrete time)

---

## Stack
`Python` `NumPy` `SciPy` `Matplotlib` `Google Colab`

---

*Northwestern University — ME 455, Spring 2025 · Instructor: Prof. Max Muchen Sun*
