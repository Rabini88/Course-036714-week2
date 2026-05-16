# Inverted Pendulum — Interactive Simulation

An interactive web-based simulation of an inverted pendulum on a cart, featuring both manual control and automatic stabilization using swing-up and LQR control strategies.

**[Live Demo]([https://rabini88.github.io/pendulum/](https://rabini88.github.io/Course-036714-week2/))** | **Keyboard Control:** Arrow keys ← → or tilt your phone

---

## Features

- **Manual Control**: Use arrow keys (or device tilt on mobile) to push the cart
- **Automatic Control**: Two-stage hybrid controller:
  - **Swing-up phase**: Energy-based controller to swing the pole from hanging to upright
  - **Stabilization phase**: LQR controller to balance the pole at the top
- **Real-time Telemetry**: Display of all system states (position, angle, velocities, forces)
- **Live Charts**: Position, angle, and force over time
- **Mobile Tilt Support**: Control via device orientation on mobile devices
- **Responsive Design**: Works on desktop and mobile screens

---

## System Description

### State Vector

The system is described by four state variables:

| Variable | Symbol | Unit | Description |
|----------|--------|------|-------------|
| Cart position | x | m | Horizontal position of cart on track |
| Cart velocity | ẋ | m/s | Cart velocity (horizontal) |
| Pole angle | θ | rad | Angle from vertical (0° = upright, 90° = hanging) |
| Pole angular velocity | θ̇ | rad/s | Angular velocity of pole |

### System Parameters

| Parameter | Symbol | Value | Unit | Description |
|-----------|--------|-------|------|-------------|
| Cart mass | M | 1.0 | kg | Mass of cart |
| Pole mass | m | 0.1 | kg | Mass of pole |
| Pole length | l | 0.5 | m | Half-length (pivot to center of mass) |
| Gravity | g | 9.81 | m/s² | Standard gravity |
| Cart friction | b | 0.1 | N·s/m | Viscous damping coefficient |

### Equations of Motion

The inverted pendulum dynamics are derived from Lagrangian mechanics. Let **F** be the horizontal force applied to the cart.

The accelerations are:

$$\ddot{x} = \frac{F - b\dot{x} + ml\dot{\theta}^2\sin\theta - mg\cos\theta\sin\theta}{\Delta}$$

$$\ddot{\theta} = \frac{(M+m)g\sin\theta - \cos\theta(F - b\dot{x} + ml\dot{\theta}^2\sin\theta)}{l \cdot \Delta}$$

where the denominator is:

$$\Delta = M + m - m\cos^2\theta$$

These equations are integrated using **RK4** (4th-order Runge-Kutta) with step size **DT = 0.01 s**.

---

## Control Strategies

### 1. Swing-up Controller (Energy-based)

The swing-up controller uses energy-based control to swing the pole from the hanging position (θ₀ = 90°) to near-vertical. This follows the **Åström-Furuta** approach.

**Target energy (at upright, θ = 0)**:
$$E_{\text{target}} = mgl$$

**Current energy**:
$$E = \frac{1}{2}ml^2\dot{\theta}^2 + mgl\cos\theta$$

**Energy error**:
$$\Delta E = E - E_{\text{target}}$$

**Control law**:
$$F_{\text{swing}} = K_{\text{swing}} \cdot \dot{\theta} \cdot \cos\theta \cdot \Delta E$$

**Parameters**:
- `K_SWINGUP = 2.0` — Energy pump gain (lower values = smoother, less overshoot)
- Force is clipped to **[-30, +30] N**

### 2. LQR (Linear Quadratic Regulator)

Once the pole is close to vertical, the LQR controller takes over for precise stabilization.

#### Linearization

The dynamics are linearized around the upright equilibrium (θ = 0):

$$A = \begin{bmatrix}
0 & 1 & 0 & 0 \\
0 & -b/M & -mg/M & 0 \\
0 & 0 & 0 & 1 \\
0 & b/(Ml) & (M+m)g/(Ml) & 0
\end{bmatrix}, \quad B = \begin{bmatrix} 0 \\ 1/M \\ 0 \\ -1/(Ml) \end{bmatrix}$$

#### Cost Function

The LQR controller minimizes the quadratic cost:

$$J = \int_0^{\infty} (x^T Q x + u^T R u) \, dt$$

**Weight matrices**:
$$Q = \text{diag}(1, 1, 10, 10), \quad R = [0.1]$$

The diagonal elements penalize:
- Cart position and velocity equally (weight = 1)
- Pole angle and angular velocity heavily (weight = 10) — stabilization is priority

#### Feedback Gain

The LQR solution is computed via the Continuous Algebraic Riccati Equation (CARE) using `scipy.linalg.solve_continuous_are`. The resulting feedback gain vector is:

$$K = [-3.1623, -6.2976, -54.8423, -15.5793]$$

**Control law**:
$$F_{\text{LQR}} = -K \cdot \mathbf{s}$$

where **s** = [x, ẋ, θ_wrapped, θ̇] (angle wrapped to [-π, π]).

Force is clipped to **[-30, +30] N**.

### 3. Switching Logic

The controller switches from swing-up to LQR when:
- Wrapped angle: |θ_wrapped| < 30° (±0.524 rad)
- **AND** angular velocity: |θ̇| < 4 rad/s

Once in LQR mode, the controller remains in LQR until manually switched or reset.

---

## Usage

### Desktop

1. Open `index.html` in a web browser
2. Use **← →** arrow keys to manually push the cart
3. Click **Auto: ON** to engage the automatic hybrid controller
   - Watch as the swing-up phase brings the pole upright
   - The controller will smoothly transition to LQR stabilization
4. Click **Pause** to pause/resume the simulation
5. Click **Reset** to return to initial state (θ₀ = 90°, all velocities = 0)

### Mobile

1. Open the deployed URL on a smartphone or tablet
2. Use arrow buttons (manual force ±10 N)
3. Click **Tilt: ON** to enable device orientation sensing
4. Tilt your device left/right to push the cart
5. Enable **Auto: ON** for automatic control

### Telemetry Display

Real-time values shown:
- **Time**: Simulation elapsed time (s)
- **x**: Cart position (m)
- **ẋ**: Cart velocity (m/s)
- **θ**: Pole angle in degrees (0° = upright, 180° = hanging)
- **θ̇**: Pole angular velocity (°/s)
- **F manual**: Manual control force (N)
- **F auto**: Automatic control force (N)
- **F total**: Combined force (N)
- **Mode**: Current control mode (—, Swing-up, or LQR)

### Charts

Three live time-series plots update during simulation:
- **Cart position** (blue) — shows x(t) over past 5 seconds
- **Pole angle** (orange) — shows θ(t) over past 5 seconds
- **Control force** (purple) — shows F(t) over past 5 seconds

---

## Implementation Details

### Simulation

- **Solver**: 4th-order Runge-Kutta (RK4) with adaptive time stepping
- **Base time step**: 0.01 s
- **History buffer**: 500 samples (~5 seconds at 10 ms/sample)
- **Frame rate**: 60 FPS (standard `requestAnimationFrame`)

### Angle Wrapping

To handle angle wraparound and keep angles in [-π, π]:

```javascript
function wrapAngle(th) {
  return ((th + Math.PI) % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI) - Math.PI;
}
```

This ensures:
- Upright (θ = 0) is the origin for LQR feedback
- Hanging (θ = π) is treated as distinct from θ = -π
- The controller always takes the shortest angular path

### Manual Input

- **Keyboard**: ±10 N per key press (held keys sum)
- **Mobile tilt**: Smoothly mapped from device γ angle
  - Dead zone: ±3° (no control)
  - Full scale: ±45° → ±10 N
  - Linear mapping in between

---

## File Structure

```
.
├── index.html              # Interactive web simulation
├── simulate_pendulum.py    # Python verification script
└── README.md              # This file
```

### Python Script

The `simulate_pendulum.py` script reproduces the simulation offline with matplotlib visualization:

```bash
python simulate_pendulum.py
```

Outputs:
- Plot: `pendulum_sim.png` — trajectories and phase portrait
- Console: LQR gain vector verification

---

## Key Insights

### Why Swing-up First?

The inverted pendulum is an **unstable equilibrium**. The upright position cannot be reached from hanging using a linear controller alone (it's outside the region of attraction). The swing-up phase:
1. Adds energy to the system to bring the pole near vertical
2. Enters the region of attraction for LQR
3. Smoothly transitions control to the linear stabilizer

### Energy Pump

The swing-up gain `K_SWINGUP` modulates the force based on:
- **θ̇ · cos(θ)**: Aligns force with pole motion near vertical (cos(θ) ≈ 1 when upright)
- **ΔE**: Scales force proportionally to energy deficit

### LQR Optimality

The LQR controller minimizes:
- **Control effort** (penalized via R = 0.1)
- **State deviation** from upright (penalized via Q = diag(1, 1, 10, 10))

The high penalty on angle (Q₃₃ = 10) ensures quick stabilization even at the cost of larger forces. This is tuned for the interactive demo; performance/robustness tradeoffs can be adjusted.

---

## References

1. **Åström & Furuta** (1996) — "Swinging up a pendulum by energy control"  
   *Automatica*, 36(2), 287-295

2. **Ogata, K.** (2010) — *Modern Control Engineering* (5th ed.)  
   Chapter on Linear Quadratic Optimal Control

3. **Doyle, J.C., Francis, B.A., & Tannenbaum, A.R.** (1992) — *Feedback Control Theory*  
   Dover Publications

---

## License

Open source. Free to use, modify, and distribute.

---

## Author

Developed as part of AI-assisted control systems coursework.

**Interactive demo:** [See it in action](https://rabini88.github.io/pendulum/)
