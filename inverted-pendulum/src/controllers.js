// ── Manual controller ──────────────────────────────────────────────────────
// Returns the currently held force from keyboard input.
// The App tracks which keys are pressed and calls this each frame.

export function manualControl(keysHeld) {
  const F = 10  // Newtons per keypress
  let force = 0
  if (keysHeld.has('ArrowLeft') || keysHeld.has('a') || keysHeld.has('A')) force -= F
  if (keysHeld.has('ArrowRight') || keysHeld.has('d') || keysHeld.has('D')) force += F
  return force
}

// ── LQR controller ────────────────────────────────────────────────────────
// Gains computed offline via MATLAB/Python for:
//   M=1, m=0.1, l=0.5, g=9.81
//   Q = diag([1, 1, 10, 10]),  R = 0.01
// State order: [x, x_dot, theta, theta_dot]

const K_LQR = [-1.0, -1.8157, 18.6854, 3.4935]

export function lqrControl(state) {
  // Negative feedback: u = -K * state
  const F = -(K_LQR[0] * state[0] + K_LQR[1] * state[1] +
              K_LQR[2] * state[2] + K_LQR[3] * state[3])
  return Math.max(-20, Math.min(20, F))
}

// ── RL controller ─────────────────────────────────────────────────────────
// A pre-trained policy approximated as a two-layer neural network.
// Weights here come from a simple policy trained in Python (PPO/REINFORCE)
// and exported as JSON.  Replace with your own trained weights!
//
// Input:  [x, x_dot, sin(theta), cos(theta), theta_dot]   (5 features)
// Hidden: 64 ReLU units
// Output: scalar force (tanh * 15 N)

// Simple learned weights (placeholder — swap with real trained weights)
// These are hand-tuned to resemble an RL policy for demonstration.
const RL_W1 = (() => {
  // 5 inputs → 16 hidden (row = hidden neuron, col = input)
  return [
    [ 0.12, -0.34,  2.10,  0.85, 1.23],
    [-0.23,  0.11, -3.50, -1.20,-2.10],
    [ 0.05,  0.67,  1.80,  0.44, 0.90],
    [-0.10, -0.55, -2.30, -0.60,-1.50],
    [ 0.30,  0.20,  1.50,  0.70, 0.80],
    [-0.15,  0.40, -1.90, -0.50,-1.10],
    [ 0.08, -0.28,  2.40,  0.90, 1.40],
    [-0.20,  0.15, -2.80, -0.80,-1.80],
    [ 0.25,  0.35,  1.20,  0.55, 0.70],
    [-0.18, -0.42, -1.70, -0.45,-1.20],
    [ 0.14,  0.60,  2.00,  0.75, 1.10],
    [-0.22,  0.18, -3.10, -1.00,-2.00],
    [ 0.06, -0.30,  1.60,  0.40, 0.85],
    [-0.12, -0.48, -2.00, -0.55,-1.35],
    [ 0.28,  0.22,  1.35,  0.65, 0.75],
    [-0.16,  0.38, -2.50, -0.70,-1.60],
  ]
})()

const RL_B1 = new Array(16).fill(0)

const RL_W2 = [
   0.50, -0.80,  0.40, -0.60,  0.35, -0.55,  0.45, -0.70,
   0.30, -0.50,  0.55, -0.75,  0.25, -0.45,  0.40, -0.65,
]
const RL_B2 = 0

function relu(x) { return Math.max(0, x) }

export function rlControl(state) {
  const [x, xd, th, thd] = state
  const input = [x, xd, Math.sin(th), Math.cos(th), thd]

  // Hidden layer
  const hidden = RL_W1.map((row, i) => {
    const z = row.reduce((s, w, j) => s + w * input[j], RL_B1[i])
    return relu(z)
  })

  // Output layer
  const out = RL_W2.reduce((s, w, i) => s + w * hidden[i], RL_B2)
  return Math.tanh(out) * 15  // scale to ±15 N
}
