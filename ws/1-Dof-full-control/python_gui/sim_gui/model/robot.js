/**
 * model/robot.js — RobotState model
 *
 * Single source of truth for the robot's current state.
 * Updated by WebSocketController when server sends telemetry.
 * Views subscribe via .subscribe() to get notified on change.
 */

export class RobotState {
  constructor() {
    // ── Position & motion ──────────────────────────────────────────────────
    this.pos_deg    = 0;      // position (degrees, wrapped 0–360 for display)
    this.pos_cumul  = 0;      // cumulative angle (unbounded, used for arm angle)
    this.vel_dps    = 0;      // velocity (deg/s)
    this.vel_rads   = 0;      // velocity (rad/s)
    this.accel_dps2 = 0;      // acceleration (deg/s²)
    this.current_mA = 0;      // motor current (mA)

    // ── Status ─────────────────────────────────────────────────────────────
    this.homed       = false;
    this.state_id    = 0;
    this.state_name  = "INIT";
    this.status_bits = 0;     // 0x01=homing 0x02=pick 0x04=place 0x08=auto
    this.estop       = false;
    this.reed_up     = false;
    this.reed_down   = false;
    this.reed_grip   = false;
    this.connected   = false;
    this.sim_mode    = true;

    // ── Graph history (ring buffer, 1500 samples = 30 s @ 50 Hz) ──────────
    this.HIST_SIZE = 1500;
    this.hist = {
      t:           [],   // timestamp (s, monotonic)
      pos_actual:  [],   // deg
      pos_setpt:   [],   // deg (from sequence model or P2P target)
      pos_error:   [],   // deg
      vel_actual:  [],   // deg/s
      pwm:         [],   // % (from simulation or estimated)
    };

    // ── Setpoint (updated by sequence/P2P commands) ────────────────────────
    this.pos_setpt_deg = 0;

    // ── Subscribers ────────────────────────────────────────────────────────
    this._subs = [];
    this._lastTs = 0;
  }

  /**
   * Apply a telemetry packet received from server.
   * @param {Object} pkt - parsed JSON from WebSocket
   */
  applyPacket(pkt) {
    this.pos_deg     = pkt.pos_deg    ?? this.pos_deg;
    this.pos_cumul   = pkt.pos_cumul  ?? this.pos_cumul;
    this.vel_dps     = pkt.vel_dps    ?? this.vel_dps;
    this.vel_rads    = pkt.vel_rads   ?? this.vel_rads;
    this.accel_dps2  = pkt.accel_dps2 ?? this.accel_dps2;
    this.current_mA  = pkt.current_mA ?? this.current_mA;
    this.homed       = pkt.homed      ?? this.homed;
    this.state_id    = pkt.state_id   ?? this.state_id;
    this.state_name  = pkt.state_name ?? this.state_name;
    this.status_bits = pkt.status_bits ?? this.status_bits;
    this.estop       = pkt.estop      ?? this.estop;
    this.reed_up     = pkt.reed_up    ?? this.reed_up;
    this.reed_down   = pkt.reed_down  ?? this.reed_down;
    this.reed_grip   = pkt.reed_grip  ?? this.reed_grip;
    this.connected   = pkt.connected  ?? this.connected;
    this.sim_mode    = pkt.sim_mode   ?? this.sim_mode;

    // Append to history ring buffer
    const t = pkt.ts ?? (this._lastTs + 0.02);
    this._lastTs = t;
    this._pushHist(t, this.pos_deg, this.pos_setpt_deg, this.vel_dps, 0);

    this._notify();
  }

  /**
   * Update position setpoint (called when a move command is issued).
   * @param {number} deg - target position in degrees
   */
  setSetpoint(deg) {
    this.pos_setpt_deg = deg;
  }

  /** Push one sample into history, evict oldest if full. */
  _pushHist(t, pos, setpt, vel, pwm) {
    const h = this.hist;
    const err = setpt - pos;
    h.t.push(t);
    h.pos_actual.push(pos);
    h.pos_setpt.push(setpt);
    h.pos_error.push(err);
    h.vel_actual.push(vel);
    h.pwm.push(pwm);

    if (h.t.length > this.HIST_SIZE) {
      h.t.shift(); h.pos_actual.shift(); h.pos_setpt.shift();
      h.pos_error.shift(); h.vel_actual.shift(); h.pwm.shift();
    }
  }

  /** Append simulation result arrays to history. */
  appendSimHistory(result) {
    const { t, pos, setpt, vel, pwm } = result;
    const h = this.hist;
    const base = (h.t.length > 0 ? h.t[h.t.length - 1] + 0.001 : 0);
    for (let i = 0; i < t.length; i++) {
      h.t.push(base + t[i]);
      h.pos_actual.push(pos[i]);
      h.pos_setpt.push(setpt[i]);
      h.pos_error.push(setpt[i] - pos[i]);
      h.vel_actual.push(vel[i] * 180 / Math.PI);
      h.pwm.push(pwm[i] * 100);
    }
    // Trim
    while (h.t.length > this.HIST_SIZE * 4) {
      Object.values(h).forEach(a => a.shift());
    }
    this._notify();
  }

  clearHistory() {
    Object.values(this.hist).forEach(a => (a.length = 0));
  }

  /** Register a callback to be called on every state update. */
  subscribe(fn) { this._subs.push(fn); }

  _notify() { this._subs.forEach(fn => fn(this)); }
}
