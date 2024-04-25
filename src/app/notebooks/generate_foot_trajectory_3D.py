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
# 2024-04-20

# %%
import rockit as rk
import casadi as cs
import numpy as np
# %matplotlib widget
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib as mpl
from matplotlib.collections import LineCollection
from mpl_toolkits.mplot3d import Axes3D


# Parameters
amax = cs.vertcat(2.5, 10.0, 5.0)  # total guess this
#amax = cs.vertcat(0.5, 0.5, 1.0)
#vmax = cs.vertcat(0.3, 0.15, 0.15)
vmax = cs.vertcat(0.67, 0.34, 0.34)
xmax = 0.05
ymax = 0.04
#vf = 0.047
vf = 0.10
hmin = 0.02
hmax = 0.04

# not fully supported yet, please keep at zero
walking_axis = 0

def create_stage(ocp, idx, t0, T, N=10):
    """Create a stage"""
    stage = ocp.stage(t0=t0, T=T)

    # Leg (x, y, z) position and velocity, y is kept constant
    p = stage.state(3)
    v = stage.state(3)
    # Leg (x, y, z) acceleration (= 'input')
    a = stage.control(3, order=0)

    # String states and control together via time
    stage.set_der(p, v)
    stage.set_der(v, a)

    # Cost objective
    match idx:
        case 0:
            # Maximize ground contact duration
            stage.add_objective(-stage.T**2)
        case 3:
            stage.add_objective(stage.integral(a[walking_axis])**2 * 1e2)
    stage.add_objective(stage.integral(p[1-walking_axis])**2)

    # Path constraints
    # General
    stage.subject_to(-xmax <= (p[0] <= xmax))
    stage.subject_to(-ymax <= (p[1] <= ymax))
    stage.subject_to(0 <= (p[2] <= hmax))
    stage.subject_to(-vmax <= (v <= vmax))
    stage.subject_to(-amax <= (a <= amax))
    # Stage-specific
    match idx:
        case 0:
            stage.subject_to(stage.at_t0(v[walking_axis]) == -vf)
            stage.subject_to(stage.t0 == 0)
            stage.subject_to(p[2] == 0)
            stage.subject_to(v[walking_axis] == -vf)
            stage.subject_to(v[2] == 0)
        case 1:
            stage.subject_to(stage.at_t0(p[2] == 0))
            stage.subject_to(p[walking_axis] > -xmax)
            stage.subject_to(p[2] <= hmin)
            stage.subject_to(v[walking_axis] == -vf)
            stage.subject_to(v[2] >= 0)
        case 2:
            stage.subject_to(stage.at_t0(p[2]) == hmin)
            stage.subject_to(stage.at_t0(v[walking_axis] == -vf))
            stage.subject_to(v[walking_axis] <= 0)
            stage.subject_to(v[2] > 0)
        case 3:
            stage.subject_to(hmin <= (p[2] <= hmax))
            stage.subject_to(stage.at_t0(p[walking_axis]) == -xmax)
            stage.subject_to(stage.at_t0(v[walking_axis]) >= 0)
            stage.subject_to(a[2] <= 0)
        case 4:
            stage.subject_to(stage.at_t0(p[walking_axis]) == xmax)
            stage.subject_to(v[walking_axis] <= 0)
            stage.subject_to(v[2] < 0)
        case 5:
            stage.subject_to(stage.at_t0(p[2] == hmin))
            stage.subject_to(p[walking_axis] < xmax)
            stage.subject_to(p[2] <= hmin)
            stage.subject_to(v[walking_axis] == -vf)
            stage.subject_to(v[2] < 0)

    # Set initial guess
    # TODO respect walking_axis
    # TODO triggers unexpected behavior in Rockit if used in combination with DirectCollocation
    '''
    match idx:
        case 0:
            stage.set_initial(p, np.linspace([xmax/2, 0, 0], [-xmax/2, 0, 0], N+1).T)
        case 1:
            stage.set_initial(p, np.linspace([-xmax/2, 0, 0], [-xmax*3/4, 0, hmin], N+1).T)
        case 2:
            stage.set_initial(p, np.linspace([-xmax*3/4, 0, hmin], [-xmax, 0, (hmin+hmax)/2], N+1).T)
        case 3:
            stage.set_initial(p, np.linspace([-xmax, 0, (hmin+hmax)/2], [0, 0, hmax], N+1).T)
        case 4:
            stage.set_initial(p, np.linspace([0, 0, (hmin+hmax)/2], [xmax*3/4, 0, hmin], N+1).T)
        case 5:
            stage.set_initial(p, np.linspace([xmax*3/4, 0, hmin], [xmax/2, 0, 0], N+1).T)
    '''

    # Set formulation method
    #method = rk.MultipleShooting(N=N, M=1, intg='rk')
    method = rk.DirectCollocation(N=N, M=1, intg='rk')
    stage.method(method)

    return stage, p, v, a


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
stages.append(create_stage(ocp, 0, rk.FreeTime(0), rk.FreeTime(2), N=1))
stages.append(create_stage(ocp, 1, rk.FreeTime(2), rk.FreeTime(4), N=10))
stages.append(create_stage(ocp, 2, rk.FreeTime(4), rk.FreeTime(6), N=3))
stages.append(create_stage(ocp, 3, rk.FreeTime(6), rk.FreeTime(8), N=30))
stages.append(create_stage(ocp, 4, rk.FreeTime(8), rk.FreeTime(10), N=3))
stages.append(create_stage(ocp, 5, rk.FreeTime(10), rk.FreeTime(12), N=10))
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
tsp = np.hstack([so[0] for so in sols])
sp = np.vstack([so[1] for so in sols])
sols = [sol(s[0]).sample(s[2], grid='control') for s in stages]
tsv = np.hstack([so[0] for so in sols])
sv = np.vstack([so[1] for so in sols])
sols = [sol(s[0]).sample(s[3], grid='control') for s in stages]
tsa = np.hstack([so[0] for so in sols])
sa = np.vstack([so[1] for so in sols])

print(f'duration: {tsp[-1]:.2f} s')

fig, axs = plt.subplots(3, 1, sharex=True, constrained_layout=True)
axs[0].plot(tsp, sp[:, 0], '-*', label='x')
axs[0].hlines(-xmax, 0, tsp[-1], linestyles=':', color='tab:blue')
axs[0].hlines(xmax, 0, tsp[-1], linestyles=':', color='tab:blue')
axs[0].plot(tsp, sp[:, 1], '-*', label='y')
axs[0].hlines(-ymax, 0, tsp[-1], linestyles=':', color='tab:orange')
axs[0].hlines(ymax, 0, tsp[-1], linestyles=':', color='tab:orange')
axs[0].plot(tsp, sp[:, 2], '-*', label='z')
axs[0].hlines(hmin, 0, tsp[-1], linestyles=':', color='tab:green')
axs[0].hlines(hmax, 0, tsp[-1], linestyles=':', color='tab:green')
axs[0].set_ylabel('p [m]')
axs[0].legend(loc='lower left')
axs[1].plot(tsp, sv[:, 0], '-*', label='x')
axs[1].hlines(-float(vmax[0]), 0, tsp[-1], linestyles=':', color='tab:blue')
axs[1].hlines(float(vmax[0]), 0, tsp[-1], linestyles=':', color='tab:blue')
axs[1].plot(tsp, sv[:, 1], '-*', label='y')
axs[1].hlines(-float(vmax[1]), 0, tsp[-1], linestyles=':', color='tab:orange')
axs[1].hlines(float(vmax[1]), 0, tsp[-1], linestyles=':', color='tab:orange')
axs[1].plot(tsp, sv[:, 2], '-*', label='z')
axs[1].hlines(-float(vmax[2]), 0, tsp[-1], linestyles=':', color='tab:green')
axs[1].hlines(float(vmax[2]), 0, tsp[-1], linestyles=':', color='tab:green')
axs[1].set_ylabel('v [m/s]')
axs[2].step(tsp, sa[:, 0], '-', where='post', label='x')
axs[2].hlines(-float(amax[0]), 0, tsp[-1], linestyles=':', color='tab:blue')
axs[2].hlines(float(amax[0]), 0, tsp[-1], linestyles=':', color='tab:blue')
axs[2].step(tsp, sa[:, 1], '-', where='post', label='y')
axs[2].hlines(-float(amax[1]), 0, tsp[-1], linestyles=':', color='tab:orange')
axs[2].hlines(float(amax[1]), 0, tsp[-1], linestyles=':', color='tab:orange')
axs[2].step(tsp, sa[:, 2], '-', where='post', label='z')
axs[2].hlines(-float(amax[2]), 0, tsp[-1], linestyles=':', color='tab:green')
axs[2].hlines(float(amax[2]), 0, tsp[-1], linestyles=':', color='tab:green')
axs[2].set_ylabel('a [m/s²]')
axs[-1].set_xlabel('time [s]')

# %%
fig, ax = plt.subplots(1, 1, sharex=True, constrained_layout=True)
cmap = 'turbo'
points = np.array([sp[:,walking_axis], sp[:,2]]).T.reshape(-1, 1, 2)
segments = np.concatenate([points[:-1], points[1:]], axis=1)
norm = plt.Normalize(0, np.linalg.norm(vmax))
lc = LineCollection(segments, cmap=cmap, norm=norm)
lc.set_array(np.linalg.norm(sv, axis=1))
line = ax.add_collection(lc)
sc = ax.scatter(sp[:, walking_axis], sp[:, 2], c=np.linalg.norm(sv, axis=1), norm=norm, cmap=cmap, marker='*')
ax.set_aspect('equal')
ax.set_xlabel('px [m]')
ax.set_ylabel('pz [m]')
cbar = plt.colorbar(line, ax=ax)
cbar.set_label('velocity [m/s]')

# %%
fig = plt.figure(constrained_layout=True)
ax = fig.add_subplot(111, projection='3d')
cmap = mpl.colormaps['turbo']
cnorm = plt.Normalize(0, np.linalg.norm(vmax))
for i in range(sp.shape[0]-2):
    ax.plot(sp[i:i+2, 0], sp[i:i+2, 1], sp[i:i+2, 2], c=cmap(cnorm(np.linalg.norm((sv[i,:]+sv[i+1,:])/2))))
sc = ax.scatter(sp[:, 0], sp[:, 1], sp[:, 2], c=np.linalg.norm(sv, axis=1), norm=norm, cmap=cmap, marker='*')
ax.set_xlabel('px [m]')
ax.set_ylabel('py [m]')
ax.set_zlabel('pz [m]')
cbar = plt.colorbar(cm.ScalarMappable(norm=cnorm, cmap=cmap), ax=ax)
cbar.set_label('velocity [m/s]')
