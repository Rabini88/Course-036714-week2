// Inverted pendulum on a cart — equations of motion via Euler-Lagrange
// State: [x, x_dot, theta, theta_dot]
// theta = 0 means pole pointing UP (unstable equilibrium)
// Positive force F pushes cart right

export const PARAMS = {
  M: 1.0,   // cart mass (kg)
  m: 0.1,   // pole mass (kg)
  l: 0.5,   // half-pole length (m)
  g: 9.81,  // gravity (m/s²)
  b: 0.1,   // cart friction
  cartLimit: 2.4,  // track half-length (m)
  angleLimit: Math.PI * 0.95,
}

export function derivatives(state, F, params = PARAMS) {
  const { M, m, l, g, b } = params
  const [, xd, th, thd] = state
  const costh = Math.cos(th)
  const sinth = Math.sin(th)
  const denom = M + m - m * costh * costh

  const xdd = (F - b * xd + m * l * thd * thd * sinth - m * g * costh * sinth) / denom
  const thdd = ((M + m) * g * sinth - costh * (F - b * xd + m * l * thd * thd * sinth)) / (l * denom)

  return [xd, xdd, thd, thdd]
}

export function rk4Step(state, F, dt, params = PARAMS) {
  const k1 = derivatives(state, F, params)
  const k2 = derivatives(state.map((v, i) => v + 0.5 * dt * k1[i]), F, params)
  const k3 = derivatives(state.map((v, i) => v + 0.5 * dt * k2[i]), F, params)
  const k4 = derivatives(state.map((v, i) => v + dt * k3[i]), F, params)
  return state.map((v, i) => v + (dt / 6) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]))
}

export function initialState() {
  return [0, 0, 5 * Math.PI / 180, 0]  // x=0, v=0, theta=5°, theta_dot=0
}
