/**
 * view/graphs.js — GraphView
 *
 * Real-time streaming graphs using Chart.js.
 * Shows 4 traces: position setpoint + actual, position error, velocity, PWM.
 * Window = last 30 s of data. Updates at most 20 Hz (every 50 ms RAF throttle).
 */

const WINDOW_S   = 30;   // seconds of history to show
const MAX_POINTS = 1500; // 30 s × 50 Hz

export class GraphView {
  /**
   * @param {string} canvasId  - id of the <canvas> element
   */
  constructor(canvasId) {
    this._canvas = document.getElementById(canvasId);
    this._chart  = null;
    this._dirty  = false;
    this._rafId  = null;

    this._build();
    this._startUpdateLoop();
  }

  _build() {
    const ctx = this._canvas.getContext("2d");

    this._chart = new Chart(ctx, {
      type: "line",
      data: {
        datasets: [
          {
            label:       "Pos setpoint (°)",
            data:        [],
            borderColor: "#F97316",
            borderDash:  [5, 3],
            borderWidth: 1.5,
            pointRadius: 0,
            tension:     0,
            yAxisID:     "yPos",
          },
          {
            label:       "Pos actual (°)",
            data:        [],
            borderColor: "#111827",
            borderWidth: 2,
            pointRadius: 0,
            tension:     0,
            yAxisID:     "yPos",
          },
          {
            label:       "Error (°)",
            data:        [],
            borderColor: "#EF4444",
            borderWidth: 1,
            pointRadius: 0,
            tension:     0,
            yAxisID:     "yErr",
          },
          {
            label:       "Velocity (°/s)",
            data:        [],
            borderColor: "#3B82F6",
            borderWidth: 1.5,
            pointRadius: 0,
            tension:     0,
            yAxisID:     "yVel",
            hidden:      false,
          },
          {
            label:       "PWM (%)",
            data:        [],
            borderColor: "#A855F7",
            borderWidth: 1,
            pointRadius: 0,
            tension:     0,
            yAxisID:     "yPwm",
            hidden:      true,   // hidden by default, toggle via legend
          },
        ],
      },
      options: {
        animation:  false,
        responsive: true,
        maintainAspectRatio: false,
        interaction: { mode: "index", intersect: false },
        plugins: {
          legend: { position: "top", labels: { boxWidth: 12, font: { size: 11 } } },
          tooltip: {
            callbacks: {
              label: ctx => `${ctx.dataset.label}: ${ctx.parsed.y.toFixed(2)}`,
            },
          },
        },
        scales: {
          x: {
            type:   "linear",
            title:  { display: true, text: "Time (s)", font: { size: 11 } },
            ticks:  { maxTicksLimit: 8, font: { size: 10 } },
          },
          yPos: {
            type:     "linear",
            position: "left",
            title:    { display: true, text: "Position (°)", font: { size: 11 } },
            ticks:    { font: { size: 10 } },
          },
          yErr: {
            type:     "linear",
            position: "right",
            title:    { display: true, text: "Error (°)", font: { size: 10 } },
            grid:     { drawOnChartArea: false },
            ticks:    { font: { size: 10 } },
          },
          yVel: {
            type:     "linear",
            position: "right",
            title:    { display: false },
            grid:     { drawOnChartArea: false },
            display:  false,   // hidden axis; visible when dataset shown
          },
          yPwm: {
            type:     "linear",
            position: "right",
            min: -100, max: 100,
            title:    { display: false },
            grid:     { drawOnChartArea: false },
            display:  false,
          },
        },
      },
    });
  }

  /**
   * Push latest history arrays from RobotState into chart datasets.
   * Called on every robot state update (subscriber callback).
   * @param {import('../model/robot.js').RobotState} state
   */
  update(state) {
    const h = state.hist;
    if (h.t.length === 0) return;

    // Compute x-axis relative time (shift to last 30 s)
    const tEnd   = h.t[h.t.length - 1];
    const tStart = Math.max(h.t[0], tEnd - WINDOW_S);

    const startIdx = h.t.findIndex(v => v >= tStart);
    const tSlice  = h.t.slice(startIdx);
    const tBase   = tSlice[0] ?? 0;

    const mk = (arr) => tSlice.map((_, i) => ({ x: tSlice[i] - tBase, y: arr[startIdx + i] }));

    this._chart.data.datasets[0].data = mk(h.pos_setpt);
    this._chart.data.datasets[1].data = mk(h.pos_actual);
    this._chart.data.datasets[2].data = mk(h.pos_error);
    this._chart.data.datasets[3].data = mk(h.vel_actual);
    this._chart.data.datasets[4].data = mk(h.pwm);

    this._dirty = true;
  }

  clearHistory() {
    this._chart.data.datasets.forEach(d => (d.data = []));
    this._dirty = true;
  }

  // ── RAF-throttled chart.update() call (prevents over-rendering) ──────────
  _startUpdateLoop() {
    const loop = () => {
      if (this._dirty) {
        this._chart.update("none");   // "none" = skip animation
        this._dirty = false;
      }
      this._rafId = requestAnimationFrame(loop);
    };
    this._rafId = requestAnimationFrame(loop);
  }

  destroy() {
    if (this._rafId) cancelAnimationFrame(this._rafId);
    this._chart?.destroy();
  }
}
