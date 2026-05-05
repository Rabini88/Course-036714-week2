import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import solve_continuous_are

# ── Parameters (identical to HTML) ───────────────────────────────────────────
M = 1.0    # cart mass (kg)
m = 0.1    # pole mass (kg)
l = 0.5    # half-pole length (m)
g = 9.81   # gravity
b = 0.1    # cart friction

DT = 0.01  # simulation step (s)
T  = 15.0  # total time (s)

x0 = np.array([0.0, 0.0, 90 * np.pi / 180, 0.0])  # same as HTML

# ── Physics (identical to HTML) ──────────────────────────────────────────────
def derivatives(state, F):
    x, xd, th, thd = state
    costh = np.cos(th)
    sinth = np.sin(th)
    denom = M + m - m * costh**2
    xdd  = (F - b*xd + m*l*thd**2*sinth - m*g*costh*sinth) / denom
    thdd = ((M + m)*g*sinth - costh*(F - b*xd + m*l*thd**2*sinth)) / (l * denom)
    return np.array([xd, xdd, thd, thdd])

def rk4(state, F, dt):
    k1 = derivatives(state, F)
    k2 = derivatives(state + 0.5*dt*k1, F)
    k3 = derivatives(state + 0.5*dt*k2, F)
    k4 = derivatives(state + dt*k3, F)
    return state + (dt/6)*(k1 + 2*k2 + 2*k3 + k4)

# ── LQR (linearised around upright, theta=0) ─────────────────────────────────
A_lin = np.array([
    [0,       1,               0,  0],
    [0,    -b/M,          -m*g/M,  0],
    [0,       0,               0,  1],
    [0,  b/(M*l), (M+m)*g/(M*l),  0]
])
B_lin = np.array([[0], [1/M], [0], [-1/(M*l)]])

Q_lqr = np.diag([1.0, 1.0, 10.0, 10.0])
R_lqr = np.array([[0.1]])

P_sol = solve_continuous_are(A_lin, B_lin, Q_lqr, R_lqr)
K     = (np.linalg.inv(R_lqr) @ B_lin.T @ P_sol).flatten()

print(f"LQR gains K = {np.round(K, 4)}  (F = -K @ state)")

def wrap_angle(th):
    """Wrap angle to [-pi, pi] so LQR sees deviation from upright."""
    return ((th + np.pi) % (2 * np.pi)) - np.pi

def lqr_force(state):
    s = state.copy()
    s[2] = wrap_angle(s[2])   # feed wrapped angle to LQR
    return float(np.clip(-K @ s, -30, 30))

# ── Swing-up controller (energy-based, Åström-Furuta) ────────────────────────
E_upright  = m * g * l   # target energy = PE at upright (theta=0, thetadot=0)
K_SWINGUP  = 2.0         # energy pump gain (lower = smoother, less overshoot)
SWITCH_DEG = 30.0        # hand-off angle  ±30°
SWITCH_VEL = 4.0         # hand-off angular velocity threshold (rad/s)

def swingup_force(state):
    th, thd = state[2], state[3]
    E  = 0.5 * m * l**2 * thd**2 + m * g * l * np.cos(th)
    dE = E - E_upright
    F  = K_SWINGUP * thd * np.cos(th) * dE
    return float(np.clip(F, -30, 30))

def combined_controller(state):
    th_wrapped = wrap_angle(state[2])
    near_upright = abs(th_wrapped) < np.radians(SWITCH_DEG)
    slow_enough  = abs(state[3]) < SWITCH_VEL
    if near_upright and slow_enough:
        return lqr_force(state)
    return swingup_force(state)

# ── Simulate ──────────────────────────────────────────────────────────────────
def simulate(controller):
    state = x0.copy()
    times, xs, thetas, forces, modes = [], [], [], [], []
    for t in np.arange(0, T, DT):
        th_w = wrap_angle(state[2])
        mode = "LQR" if (abs(th_w) < np.radians(SWITCH_DEG) and abs(state[3]) < SWITCH_VEL) else "Swing-up"
        F    = controller(state)
        times.append(t)
        xs.append(state[0])
        thetas.append(np.degrees(th_w))   # wrapped angle for display
        forces.append(F)
        modes.append(mode)
        state = rk4(state, F, DT)
    return (np.array(times), np.array(xs),
            np.array(thetas), np.array(forces), modes)

t, x_sw, th_sw, f_sw, modes = simulate(combined_controller)

# ── Plot ──────────────────────────────────────────────────────────────────────
fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
fig.suptitle(
    f"Swing-up + LQR  |  θ₀ = 90°  |  switch at ±{SWITCH_DEG}°\n"
    f"K_swing = {K_SWINGUP},  K_LQR = {np.round(K, 3)}", fontsize=11)

# shade regions by mode
for ax in axes:
    in_lqr = False
    t_start = 0
    for i, mode in enumerate(modes):
        if mode == "LQR" and not in_lqr:
            t_start = t[i]; in_lqr = True
        elif mode != "LQR" and in_lqr:
            ax.axvspan(t_start, t[i], alpha=0.12, color='green', label='LQR region' if ax is axes[0] else '')
            in_lqr = False
    if in_lqr:
        ax.axvspan(t_start, t[-1], alpha=0.12, color='green')

axes[0].plot(t, x_sw, color='steelblue', lw=1.5)
axes[0].axhline(0, color='k', lw=0.5, ls='--')
axes[0].set_ylabel("Cart position (m)")
axes[0].grid(True)

axes[1].plot(t, th_sw, color='darkorange', lw=1.5)
axes[1].axhline(0,  color='k', lw=0.5, ls='--')
axes[1].axhline( SWITCH_DEG, color='green', lw=1, ls=':', label=f'±{SWITCH_DEG}° switch')
axes[1].axhline(-SWITCH_DEG, color='green', lw=1, ls=':')
axes[1].set_ylabel("Pole angle — wrapped (°)")
axes[1].legend(fontsize=9); axes[1].grid(True)

axes[2].plot(t, f_sw, color='mediumpurple', lw=1.5)
axes[2].axhline(0, color='k', lw=0.5, ls='--')
axes[2].set_ylabel("Control force (N)")
axes[2].set_xlabel("Time (s)")
axes[2].grid(True)

# legend for shading
from matplotlib.patches import Patch
axes[0].legend(handles=[
    Patch(facecolor='green', alpha=0.3, label='LQR active'),
    Patch(facecolor='white', edgecolor='grey', label='Swing-up active')
], fontsize=9)

plt.tight_layout()
plt.savefig("pendulum_sim.png", dpi=120)
plt.show()
print("Plot saved to pendulum_sim.png")
