/**
 * view/stats.js — StatsView
 *
 * Renders the STATS panel: Position, Speed, Accel, Gripper, Mode, Emergency.
 * Updates the DOM elements in-place; no full re-render.
 */

const STATE_COLORS = {
  IDLE:         "#22C55E",   // green
  AUTO:         "#3B82F6",   // blue
  SEQUENCE:     "#A855F7",   // purple
  HOMING_FAST:  "#F59E0B",   // amber
  HOMING_BACKOFF:"#F59E0B",
  HOMING_SLOW:  "#F59E0B",
  MANUAL:       "#F97316",   // orange
  MANUAL_MB:    "#F97316",
  TEST:         "#EC4899",   // pink
  EMER:         "#EF4444",   // red
  INIT:         "#9CA3AF",   // grey
  default:      "#9CA3AF",
};

export class StatsView {
  /**
   * @param {Object} elMap  - { pos, speed, accel, gripper, mode, emergency, heartbeat }
   *                          Each value is a DOM element id string.
   */
  constructor(elMap) {
    this._el = {};
    for (const [k, id] of Object.entries(elMap)) {
      this._el[k] = document.getElementById(id);
    }
  }

  /**
   * Update all stats from robot state.
   * @param {import('../model/robot.js').RobotState} state
   */
  update(state) {
    this._set("pos",   `${state.pos_deg.toFixed(1)}`);
    this._set("speed", `${(state.vel_dps * Math.PI / 180).toFixed(2)}`);
    this._set("accel", `${(state.accel_dps2 * Math.PI / 180).toFixed(2)}`);

    // Orange only when robot is actively moving or in a motion state
    const motionStates = ["AUTO","SEQUENCE","HOMING_FAST","HOMING_BACKOFF","HOMING_SLOW","MANUAL","MANUAL_MB"];
    const stateName    = state.state_name.replace("(sim)", "").trim();
    const isMoving     = Math.abs(state.vel_dps) > 0.5;
    const isActive     = motionStates.includes(stateName) || isMoving;
    this._el.pos  ?.classList.toggle("is-active", isActive);
    this._el.speed?.classList.toggle("is-active", isMoving);
    this._el.accel?.classList.toggle("is-active", Math.abs(state.accel_dps2) > 1);

    // Gripper status from reed switches
    const gripStr = state.reed_grip  ? "Closed"
                  : state.reed_up    ? "Up / Open"
                  : state.reed_down  ? "Down / Open"
                  : "Idle / Open";
    this._set("gripper", gripStr);

    // Mode / state
    this._set("mode", stateName);
    if (this._el.mode) {
      this._el.mode.style.color = STATE_COLORS[stateName] ?? STATE_COLORS.default;
    }

    // Emergency
    const emerStr = state.estop ? "ACTIVE" : "—";
    this._set("emergency", emerStr);
    if (this._el.emergency) {
      this._el.emergency.style.color = state.estop ? "#EF4444" : "#9CA3AF";
    }

    // Heartbeat / connection
    if (this._el.heartbeat) {
      const txt = state.connected ? "Normal" : state.sim_mode ? "Simulation" : "Lost";
      const col = state.connected ? "#22C55E" : state.sim_mode ? "#F59E0B" : "#EF4444";
      this._el.heartbeat.textContent = txt;
      this._el.heartbeat.style.color = col;
    }
  }

  _set(key, val) {
    if (this._el[key]) this._el[key].textContent = val;
  }
}
