/**
 * workers/sim.worker.js — Offline cascade PID + motor simulation
 *
 * Runs in a Web Worker so heavy computation doesn't block the UI.
 *
 * Mirrors Due's algorithm from control.c:
 *   Velocity loop  @ 1 kHz  : PID(vel_cmd - vel_actual) → pwm
 *   Position loop  @ 500 Hz : PID(ref_pos - pos_actual) → vel_cmd
 *   S-curve trajectory generator produces ref_pos / ref_vel
 *
 * Motor model (1st-order electrical, 2nd-order mechanical):
 *   τ_e * di/dt + i = (pwm*Vs - Ke*ω) / R      (simplified: τ_e ≈ 0 → i ≈ (pwm*Vs - Ke*ω) / R)
 *   J * dω/dt = Kt*N*i - b*ω - τ_load
 *
 * Message protocol:
 *   IN:  { type: "run", params: GainsAndMotor, moves: MoveDeg[] }
 *   OUT: { type: "result", t[], pos[], setpt[], vel[], pwm[], settled[] }
 *   OUT: { type: "progress", pct: 0-100 }
 */

// ── Motor / system parameters (default = Due's values) ────────────────────────
const DEF_MOTOR = {
  J:  0.027,       // kg·m²  equivalent inertia
  b:  0.5,         // N·m·s  viscous friction
  N:  50,          // gear ratio
  Kt: 0.00747,     // N·m/A
  Ke: 0.0083,      // V·s/rad
  R:  2.8,         // Ω armature resistance
  Vs: 24,          // supply voltage
};

// ── S-curve trajectory generator ──────────────────────────────────────────────
class SCurve {
  constructor(vmax, amax, jmax, dt) {
    this.vmax = vmax; this.amax = amax; this.jmax = jmax; this.dt = dt;
    this.p = 0; this.v = 0; this.a = 0;
    this.pTarget = 0; this.active = false;
  }

  setTarget(disp) {
    this.pTarget = disp;
    this.p = 0; this.v = 0; this.a = 0;
    this.active = Math.abs(disp) > 1e-6;
    // Compute phase durations (simplified trapezoidal-accel profile)
    const vmax = Math.min(this.vmax, Math.sqrt(Math.abs(disp) * this.amax));
    this._dir  = Math.sign(disp);
    this._T1   = vmax / this.amax;                // jerk-up
    this._T2   = Math.abs(disp) / vmax - this._T1; // cruise (may be 0)
    if (this._T2 < 0) this._T2 = 0;
    this._T3   = this._T1;                        // jerk-down
    this._t    = 0;
    this._vmax = vmax;
  }

  update() {
    if (!this.active) { return { p: this.pTarget, v: 0, active: false }; }
    this._t += this.dt;
    const t = this._t, T1 = this._T1, T2 = this._T2, T3 = this._T3;
    const dir = this._dir, vm = this._vmax, am = this.amax;
    let v, p;

    if (t <= T1) {
      v = am * t * dir;
      p = 0.5 * am * t * t * dir;
    } else if (t <= T1 + T2) {
      const dt2 = t - T1;
      v = vm * dir;
      p = (0.5 * am * T1 * T1 + vm * dt2) * dir;
    } else if (t <= T1 + T2 + T3) {
      const dt3 = t - T1 - T2;
      v = (vm - am * dt3) * dir;
      p = (0.5 * am * T1 * T1 + vm * T2 + vm * dt3 - 0.5 * am * dt3 * dt3) * dir;
    } else {
      this.active = false;
      return { p: this.pTarget, v: 0, active: false };
    }
    return { p, v, active: true };
  }
}

// ── Simulation core ────────────────────────────────────────────────────────────
function runSimulation(params, moves) {
  const DT    = 1e-3;       // 1 kHz
  const POS_EVERY = 2;      // position PID every 2 ticks (500 Hz)

  // Gains
  const kp_pos = params.kp_pos ?? 2.0;
  const ki_pos = params.ki_pos ?? 0.7;
  const kd_pos = params.kd_pos ?? 0.2;
  const kp_vel = params.kp_vel ?? 10.0;
  const ki_vel = params.ki_vel ?? 0.01;
  const kd_vel = params.kd_vel ?? 0.0;
  const vmax   = params.v_max  ?? 3.0;
  const amax   = params.a_max  ?? 12.56;
  const jmax   = params.j_max  ?? 10.0;
  const deadband = (params.pos_deadband_deg ?? 2.0) * Math.PI / 180;

  // Motor
  const M = { ...DEF_MOTOR, ...(params.motor ?? {}) };

  // State
  let pos   = 0;   // rad
  let vel   = 0;   // rad/s
  let posI  = 0;   // pos PID integral
  let velI  = 0;   // vel PID integral
  let velPrev = 0;
  let velCmd  = 0;
  let settled = false;

  const scurve = new SCurve(vmax, amax, jmax, DT);

  // Output arrays (pre-allocated estimate)
  const tArr = [], posArr = [], setptArr = [], velArr = [], pwmArr = [];

  let t = 0;
  let posTick = 0;
  let startRad = 0;

  for (const moveDeg of moves) {
    const targetRad = moveDeg * Math.PI / 180;
    const disp = targetRad - pos;
    startRad = pos;
    scurve.setTarget(disp);

    let hardStop = false;
    settled = false;

    // Run until settled + 1 extra second (100 ms enough usually, cap at 10 s)
    const MAX_TICKS = 10000;
    let afterSettleTicks = 0;

    for (let tick = 0; tick < MAX_TICKS; tick++) {
      // 1. Advance trajectory
      const traj = scurve.update();
      const idealPos = startRad + traj.p;
      const idealVel = traj.v;

      // 2. Position PID @ 500 Hz
      if (posTick++ % POS_EVERY === 0) {
        const posErr = idealPos - pos;
        if (!traj.active && Math.abs(posErr) <= deadband) {
          velCmd  = 0;
          hardStop = true;
          settled  = true;
        } else {
          hardStop = false;
          settled  = false;
          posI += posErr * (DT * POS_EVERY);
          const maxI = ki_pos > 0 ? vmax / ki_pos : 0;
          posI = Math.max(-maxI, Math.min(maxI, posI));
          const velCorr = kp_pos * posErr + ki_pos * posI - kd_pos * vel;
          velCmd = Math.max(-vmax, Math.min(vmax, idealVel + velCorr));
        }
      }

      // 3. Velocity PID @ 1 kHz
      let pwm = 0;
      if (!hardStop) {
        const velErr = velCmd - vel;
        velI += velErr * DT;
        const maxVI = ki_vel > 0 ? 100 / ki_vel : 0;
        velI = Math.max(-maxVI, Math.min(maxVI, velI));
        const d_out = kd_vel * (velErr - velPrev) / DT;
        velPrev = velErr;
        pwm = kp_vel * velErr + ki_vel * velI + d_out;
        pwm = Math.max(-1, Math.min(1, pwm));   // normalised -1..+1
      }

      // 4. Motor model  (i ≈ (pwm*Vs - Ke*vel) / R,  torque = Kt*N*i - b*vel)
      const i_motor = (pwm * M.Vs - M.Ke * vel) / M.R;
      const torque  = M.Kt * M.N * i_motor - M.b * vel;
      const alpha   = torque / (M.J * M.N * M.N);
      vel += alpha * DT;
      pos += vel * DT;

      t += DT;

      // Record every 5 ticks (200 Hz) to keep array size reasonable
      if (tick % 5 === 0) {
        tArr.push(t);
        posArr.push(pos * 180 / Math.PI);
        setptArr.push(targetRad * 180 / Math.PI);
        velArr.push(vel);
        pwmArr.push(pwm);
      }

      if (settled) {
        afterSettleTicks++;
        if (afterSettleTicks > 200) break;   // record 200 ms after settling
      }
    }
  }

  return { t: tArr, pos: posArr, setpt: setptArr, vel: velArr, pwm: pwmArr };
}

// ── Worker message handler ─────────────────────────────────────────────────────
self.onmessage = function (e) {
  const { type, params, moves } = e.data;

  if (type === "run") {
    const result = runSimulation(params, moves);
    self.postMessage({ type: "result", ...result });
  }
};
