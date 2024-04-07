# ---
# jupyter:
#   jupytext:
#     formats: py:percent
#     text_representation:
#       extension: .py
#       format_name: percent
#       format_version: '1.3'
#       jupytext_version: 1.16.1
#   kernelspec:
#     display_name: Python 3 (ipykernel)
#     language: python
#     name: python3
# ---

# %% [markdown]
# # Generate an ideal foot trajectory using the Rockit MPC toolbox
#
# 2024-04-07

# %%
import rockit as rk
import casadi as cs
import numpy as np
# %matplotlib inline
import matplotlib.pyplot as plt


# Parameters
amax = cs.vertcat(0.5, 1.0)
vmax = cs.vertcat(0.3, 0.15)
xmax = 0.05
vf = 0.05
hmin = 0.02
hmax = 0.04


def create_stage(ocp, idx, t0, T, N=10):
    """Create a stage"""
    stage = ocp.stage(t0=t0, T=T)

    # Leg (x, z) position and velocity, y is kept constant
    x = stage.state(2)
    v = stage.state(2)
    # Leg (x, z) acceleration (= 'input')
    a = stage.control(2, order=0)

    # String states and control together via time
    stage.set_der(x, v)
    stage.set_der(v, a)

    # Cost objective
    match idx:
        case 0:
            # Maximize ground contact duration
            stage.add_objective(-stage.T**2)
        case 3:
            # Minor intensive to keep the foot high
            stage.add_objective(-stage.integral(x[1])**2 * 1e0)

    # Path constraints
    # General
    stage.subject_to(-xmax <= (x[0] <= xmax))
    stage.subject_to(0 <= (x[1] <= hmax))
    stage.subject_to(-vmax <= (v <= vmax))
    stage.subject_to(-amax <= (a <= amax))
    # Stage-specific
    match idx:
        case 0:
            stage.subject_to(stage.at_t0(v[0]) == -vf)
            stage.subject_to(stage.t0 == 0)
            stage.subject_to(x[1] == 0)
            stage.subject_to(v[0] == -vf)
            stage.subject_to(v[1] == 0)
        case 1:
            stage.subject_to(stage.at_t0(x[1] == 0))
            stage.subject_to(x[0] > -xmax)
            stage.subject_to(x[1] <= hmin)
            stage.subject_to(v[0] == -vf)
            stage.subject_to(v[1] >= 0)
        case 2:
            stage.subject_to(stage.at_t0(x[1]) == hmin)
            stage.subject_to(stage.at_t0(v[0] == -vf))
            stage.subject_to(v[0] <= 0)
            stage.subject_to(v[1] > 0)
        case 3:
            stage.subject_to(hmin <= (x[1] <= hmax))
            stage.subject_to(stage.at_t0(x[0]) == -xmax)
            stage.subject_to(stage.at_t0(v[0]) >= 0)
        case 4:
            stage.subject_to(stage.at_t0(x[0]) == xmax)
            stage.subject_to(v[0] <= 0)
            stage.subject_to(v[1] < 0)
        case 5:
            stage.subject_to(stage.at_t0(x[1] == hmin))
            stage.subject_to(x[0] < xmax)
            stage.subject_to(x[1] <= hmin)
            stage.subject_to(v[0] == -vf)
            stage.subject_to(v[1] < 0)

    # Set formulation method
    method = rk.MultipleShooting(N=N, M=1, intg='rk')
    stage.method(method)

    return stage, x, v, a


def stitch_stages(ocp, stage1, stage2, stitch_time=True):
    if stitch_time:
        ocp.subject_to(stage1.tf == stage2.t0)
    for i in range(len(stage1.states)):
        ocp.subject_to(stage2.at_t0(stage2.states[i])
                       == stage1.at_tf(stage1.states[i]))


# The main problem
ocp = rk.Ocp()

# Define stages
# Stage 0: ground contact, moving backwards
# Stage 1: going upwards, but still backwards at same speed
# Stage 2: until most backwards point
# Stage 3: move forwards sufficiently high => don't care
# Stage 4: until most forwards point
# Stage 5: going down, already at correct backward velocity
stages = []
stages.append(create_stage(ocp, 0, rk.FreeTime(0), rk.FreeTime(1), N=3))
stages.append(create_stage(ocp, 1, rk.FreeTime(1), rk.FreeTime(2)))
stages.append(create_stage(ocp, 2, rk.FreeTime(2), rk.FreeTime(3), N=5))
stages.append(create_stage(ocp, 3, rk.FreeTime(3), rk.FreeTime(4), N=50))
stages.append(create_stage(ocp, 4, rk.FreeTime(4), rk.FreeTime(5), N=5))
stages.append(create_stage(ocp, 5, rk.FreeTime(5), rk.FreeTime(6)))
# Feet must be in contact half of the time
ocp.subject_to(stages[0][0].T == sum(s[0].T for s in stages[1:]))
for sa, sb in zip(stages[:-1], stages[1:]): stitch_stages(ocp, sa[0], sb[0])
stitch_stages(ocp, stages[-1][0], stages[0][0], stitch_time=False)

# Initial guess
# TODO

# Set solver
ocp.solver('ipopt')

# Solve
try:
    sol = ocp.solve()
except Exception as e:
    print('exception:', e)
    sol = ocp.non_converged_solution




# %%
# Get the results and plot them

sols = [sol(s[0]).sample(s[1], grid='control') for s in stages]
tsx = np.hstack([so[0] for so in sols])
sx = np.vstack([so[1] for so in sols])
sols = [sol(s[0]).sample(s[2], grid='control') for s in stages]
tsv = np.hstack([so[0] for so in sols])
sv = np.vstack([so[1] for so in sols])
sols = [sol(s[0]).sample(s[3], grid='control') for s in stages]
tsa = np.hstack([so[0] for so in sols])
sa = np.vstack([so[1] for so in sols])

print(f'duration: {tsx[-1]:.2f} s')

fig, axs = plt.subplots(3, 1, sharex=True)
axs[0].plot(tsx, sx[:, 0], '-*', label='x')
axs[0].plot(tsx, sx[:, 1], '-', label='z')
axs[0].set_ylabel('p [m]')
axs[1].plot(tsx, sv[:, 0], '-', label='x')
axs[1].plot(tsx, sv[:, 1], '-', label='z')
axs[1].set_ylabel('v [m/s]')
axs[2].plot(tsx, sa[:, 0], '-', label='x')
axs[2].plot(tsx, sa[:, 1], '-', label='z')
axs[2].set_ylabel('a [m/s²]')

# %%
fig, ax = plt.subplots(1, 1, sharex=True)
ax.plot(sx[:, 0], sx[:, 1], '-*')
ax.set_xlabel('px [m]')
ax.set_ylabel('py [m]')
