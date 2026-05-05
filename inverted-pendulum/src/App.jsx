import { useEffect, useRef, useState, useCallback } from 'react'
import './App.css'
import { rk4Step, initialState, PARAMS } from './physics.js'

const DT = 0.01          // simulation step (s)
const HISTORY_LEN = 500  // ~5 s of history at DT=0.01

// ── Drawing ──────────────────────────────────────────────────────────────────

function drawSim(canvas, state) {
  const ctx = canvas.getContext('2d')
  const W = canvas.width, H = canvas.height
  ctx.clearRect(0, 0, W, H)
  ctx.fillStyle = '#1a1d27'
  ctx.fillRect(0, 0, W, H)

  const scale = W / (2 * PARAMS.cartLimit * 2.4)
  const groundY = H * 0.70
  const cx = W / 2 + state[0] * scale

  // Track
  ctx.strokeStyle = '#374151'
  ctx.lineWidth = 2
  ctx.beginPath()
  ctx.moveTo(0, groundY + 18)
  ctx.lineTo(W, groundY + 18)
  ctx.stroke()

  // Cart
  const cW = 60, cH = 28
  ctx.fillStyle = '#3b82f6'
  ctx.beginPath()
  ctx.roundRect(cx - cW / 2, groundY - cH, cW, cH, 5)
  ctx.fill()

  // Wheels
  for (const dx of [-18, 18]) {
    ctx.fillStyle = '#1e293b'
    ctx.beginPath()
    ctx.arc(cx + dx, groundY + 4, 9, 0, Math.PI * 2)
    ctx.fill()
    ctx.strokeStyle = '#60a5fa'
    ctx.lineWidth = 2
    ctx.stroke()
  }

  // Pole (theta=0 → upright)
  const poleLen = PARAMS.l * 2 * scale
  const px = cx + poleLen * Math.sin(state[2])
  const py = groundY - cH / 2 - poleLen * Math.cos(state[2])

  ctx.strokeStyle = '#f97316'
  ctx.lineWidth = 6
  ctx.lineCap = 'round'
  ctx.beginPath()
  ctx.moveTo(cx, groundY - cH / 2)
  ctx.lineTo(px, py)
  ctx.stroke()

  // Bob
  ctx.fillStyle = '#fbbf24'
  ctx.beginPath()
  ctx.arc(px, py, 9, 0, Math.PI * 2)
  ctx.fill()
}

function drawChart(canvas, data, color, yMin, yMax, yLabel) {
  const ctx = canvas.getContext('2d')
  const W = canvas.width, H = canvas.height
  ctx.clearRect(0, 0, W, H)
  ctx.fillStyle = '#1a1d27'
  ctx.fillRect(0, 0, W, H)

  // Zero line
  const zeroFrac = (0 - yMin) / (yMax - yMin)
  const zeroY = H - zeroFrac * H
  ctx.strokeStyle = '#374151'
  ctx.lineWidth = 1
  ctx.setLineDash([4, 4])
  ctx.beginPath()
  ctx.moveTo(0, zeroY)
  ctx.lineTo(W, zeroY)
  ctx.stroke()
  ctx.setLineDash([])

  if (data.length < 2) return

  ctx.strokeStyle = color
  ctx.lineWidth = 2
  ctx.beginPath()
  data.forEach((v, i) => {
    const x = (i / (HISTORY_LEN - 1)) * W
    const y = H - (Math.max(yMin, Math.min(yMax, v)) - yMin) / (yMax - yMin) * H
    i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y)
  })
  ctx.stroke()

  // Current value
  const last = data[data.length - 1]
  ctx.fillStyle = color
  ctx.font = '12px monospace'
  ctx.fillText(`${yLabel} = ${last.toFixed(3)}`, 8, 16)
}

// ── App ───────────────────────────────────────────────────────────────────────

export default function App() {
  const [paused, setPaused] = useState(false)
  const [time, setTime] = useState(0)

  const simCanvas = useRef(null)
  const posCanvas = useRef(null)
  const angCanvas = useRef(null)

  const stateRef = useRef(initialState())
  const histRef = useRef({ pos: [], ang: [] })
  const simTimeRef = useRef(0)
  const accRef = useRef(0)
  const lastTsRef = useRef(null)
  const pausedRef = useRef(false)

  useEffect(() => { pausedRef.current = paused }, [paused])

  const reset = useCallback(() => {
    stateRef.current = initialState()
    histRef.current = { pos: [], ang: [] }
    simTimeRef.current = 0
    accRef.current = 0
    lastTsRef.current = null
    setTime(0)
    setPaused(false)
    pausedRef.current = false
  }, [])

  useEffect(() => {
    let alive = true
    let raf

    function tick(ts) {
      if (!alive) return
      raf = requestAnimationFrame(tick)
      if (pausedRef.current || !simCanvas.current) return

      if (lastTsRef.current === null) { lastTsRef.current = ts; return }
      const realElapsed = Math.min((ts - lastTsRef.current) / 1000, 0.05)
      lastTsRef.current = ts
      accRef.current += realElapsed

      while (accRef.current >= DT) {
        accRef.current -= DT
        // Open loop: F = 0
        stateRef.current = rk4Step(stateRef.current, 0, DT)
        simTimeRef.current += DT

        const h = histRef.current
        h.pos.push(stateRef.current[0])
        h.ang.push(stateRef.current[2] * 180 / Math.PI)
        if (h.pos.length > HISTORY_LEN) { h.pos.shift(); h.ang.shift() }
      }

      const s = stateRef.current
      setTime(+simTimeRef.current.toFixed(2))
      drawSim(simCanvas.current, s)
      drawChart(posCanvas.current, histRef.current.pos, '#60a5fa', -2.5, 2.5, 'x (m)')
      drawChart(angCanvas.current, histRef.current.ang, '#f97316', -200, 200, 'θ (°)')
    }

    raf = requestAnimationFrame(tick)
    return () => { alive = false; cancelAnimationFrame(raf) }
  }, [])

  const s = stateRef.current
  return (
    <div className="app">
      <h1>Inverted Pendulum — Open Loop</h1>
      <p className="subtitle">x₀ = 0 m, ẋ₀ = 0, θ₀ = 5°, θ̇₀ = 0 · no control force applied</p>

      <div className="canvas-wrapper">
        <canvas ref={simCanvas} width={900} height={260} />
      </div>

      <div className="controls">
        <button className="btn-pause" onClick={() => setPaused(p => !p)}>
          {paused ? 'Resume' : 'Pause'}
        </button>
        <button className="btn-reset" onClick={reset}>Reset</button>
      </div>

      <div className="telemetry">
        <div className="tel-item"><div className="tel-label">Time (s)</div><div className="tel-value">{time.toFixed(2)}</div></div>
        <div className="tel-item"><div className="tel-label">x (m)</div><div className="tel-value">{s[0].toFixed(3)}</div></div>
        <div className="tel-item"><div className="tel-label">ẋ (m/s)</div><div className="tel-value">{s[1].toFixed(3)}</div></div>
        <div className="tel-item"><div className="tel-label">θ (°)</div><div className="tel-value">{(s[2] * 180 / Math.PI).toFixed(2)}</div></div>
        <div className="tel-item"><div className="tel-label">θ̇ (°/s)</div><div className="tel-value">{(s[3] * 180 / Math.PI).toFixed(1)}</div></div>
      </div>

      <div className="charts">
        <div className="chart-box">
          <div className="chart-title">Cart position — x (m)</div>
          <canvas ref={posCanvas} width={500} height={130} />
        </div>
        <div className="chart-box">
          <div className="chart-title">Pole angle — θ (°)</div>
          <canvas ref={angCanvas} width={500} height={130} />
        </div>
      </div>
    </div>
  )
}
