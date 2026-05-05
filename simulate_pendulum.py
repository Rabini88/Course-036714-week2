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
T  = 10.0  # total time (s)

x0 = np.array([0.0, 0.0, 5 * np.pi / 180, 0.0])  # same as HTML

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

# ── Linearised system around theta=0 (upright) ───────────────────────────────
A = np.array([
    [0,       1,               0,  0],
    [0,    -b/M,          -m*g/M,  0],
    [0,       0,               0,  1],
    [0,  b/(M*l), (M+m)*g/(M*l),  0]
])
B = np.array([[0], [1/M], [0], [-1/(M*l)]])

print("A =\n", np.round(A, 4))
print("B =\n", np.round(B, 4))

# ── LQR via scipy ─────────────────────────────────────────────────────────────
Q = np.diag([1.0, 1.0, 10.0, 10.0])
R = np.array([[0.1]])

P  = solve_continuous_are(A, B, Q, R)
K  = np.linalg.inv(R) @ B.T @ P          # shape (1, 4)
K  = K.flatten()                          # shape (4,)

print(f"\nLQR gains K = {np.round(K, 4)}")
print("Convention: F = -K @ state   (u = -Kx)")

def lqr_force(state):
    F = -K @ state
    return np.clip(F, -30, 30)

# ── Simulate ──────────────────────────────────────────────────────────────────
def simulate(controller, label):
    state = x0.copy()
    times, xs, thetas, forces = [], [], [], []
    for t in np.arange(0, T, DT):
        F = controller(state)
        times.append(t)
        xs.append(state[0])
        thetas.append(np.degrees(state[2]))
        forces.append(F)
        state = rk4(state, F, DT)
    return np.array(times), np.array(xs), np.array(thetas), np.array(forces)

t, x_ol, th_ol, f_ol = simulate(lambda s: 0.0,      "Open loop")
t, x_lq, th_lq, f_lq = simulate(lqr_force,          "LQR")

# ── Plot ──────────────────────────────────────────────────────────────────────
fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
fig.suptitle(f"Inverted Pendulum Simulation\n"
             f"K = {np.round(K, 3)}   (Q=diag(1,1,10,10), R=0.1)", fontsize=11)

axes[0].plot(t, x_ol,  label="Open loop")
axes[0].plot(t, x_lq,  label="LQR")
axes[0].set_ylabel("Cart position (m)")
axes[0].axhline(0, color='k', lw=0.5, ls='--')
axes[0].legend(); axes[0].grid(True)

axes[1].plot(t, th_ol, label="Open loop")
axes[1].plot(t, th_lq, label="LQR")
axes[1].set_ylabel("Pole angle (°)")
axes[1].axhline(0, color='k', lw=0.5, ls='--')
axes[1].legend(); axes[1].grid(True)

axes[2].plot(t, f_ol,  label="Open loop")
axes[2].plot(t, f_lq,  label="LQR")
axes[2].set_ylabel("Control force (N)")
axes[2].set_xlabel("Time (s)")
axes[2].axhline(0, color='k', lw=0.5, ls='--')
axes[2].legend(); axes[2].grid(True)

plt.tight_layout()
plt.savefig("pendulum_sim.png", dpi=120)
plt.show()
print("\nPlot saved to pendulum_sim.png")
