/**
 * model/simulator.js — SimulatorModel
 *
 * Wraps the Web Worker (sim.worker.js).
 * Accepts a list of moves (degrees) and PID gains,
 * returns simulation results to the caller via Promise.
 */

export class SimulatorModel {
  constructor() {
    this._worker = new Worker('./workers/sim.worker.js');
    this._resolve = null;
    this._reject  = null;

    this._worker.onmessage = (e) => {
      const { type } = e.data;
      if (type === "result" && this._resolve) {
        this._resolve(e.data);
        this._resolve = null;
        this._reject  = null;
      } else if (type === "progress" && this._onProgress) {
        this._onProgress(e.data.pct);
      }
    };

    this._worker.onerror = (err) => {
      if (this._reject) {
        this._reject(err);
        this._resolve = null;
        this._reject  = null;
      }
    };
  }

  /**
   * Run a simulation for the given sequence of moves.
   *
   * @param {number[]} movesDeg  - array of target positions in degrees
   * @param {Object}   gains     - PID and trajectory gains (kp_vel, ki_vel, kp_pos, …)
   * @param {Function} onProgress - optional callback(pct 0-100)
   * @returns {Promise<{t, pos, setpt, vel, pwm}>}
   */
  run(movesDeg, gains, onProgress) {
    if (this._resolve) {
      return Promise.reject(new Error("Simulation already in progress"));
    }
    this._onProgress = onProgress ?? null;
    return new Promise((resolve, reject) => {
      this._resolve = resolve;
      this._reject  = reject;
      this._worker.postMessage({
        type:   "run",
        params: gains,
        moves:  movesDeg,
      });
    });
  }
}
