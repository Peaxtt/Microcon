/**
 * controller/events.js — EventController
 *
 * Bridges user actions (from ControlsView events) to server commands (via WebSocketController)
 * and model updates (SequenceModel, SimulatorModel, RobotState).
 *
 * This is the main "wiring" layer — it keeps View and Model decoupled.
 */

export class EventController {
  /**
   * @param {import('./ws.js').WebSocketController}          ws
   * @param {import('../model/robot.js').RobotState}         robot
   * @param {import('../model/sequence.js').SequenceModel}   seq
   * @param {import('../model/simulator.js').SimulatorModel} sim
   * @param {import('../view/controls.js').ControlsView}     controls
   * @param {import('../view/disk.js').DiskView}             disk
   * @param {import('../view/graphs.js').GraphView}          graphs
   */
  constructor(ws, robot, seq, sim, controls, disk, graphs) {
    this._ws       = ws;
    this._robot    = robot;
    this._seq      = seq;
    this._sim      = sim;
    this._controls = controls;
    this._disk     = disk;
    this._graphs   = graphs;

    this._simRunning = false;

    this._bindControls();
    this._bindSeqModel();
    this._bindRobotModel();
  }

  // ── ControlsView event bindings ────────────────────────────────────────────

  _bindControls() {
    const cv = this._controls;

    // Any generic Modbus command (gripper, jog, home, p2p, gains, etc.)
    cv.addEventListener("cmd", (e) => {
      this._ws.sendCmd(e.detail);
      // Track setpoint for graphs when a move is commanded
      if (e.detail.type === "p2p" && e.detail.hole !== undefined) {
        const deg = e.detail.hole * 5;
        this._robot.setSetpoint(deg);
      }
    });

    // Random pair generation
    cv.addEventListener("randomize", (e) => {
      const pairs = this._seq.generateRandom(e.detail.n, 180, 90);
      this._controls.renderPairs(pairs);
    });

    // Preview path on disk (no movement, just visual)
    cv.addEventListener("preview", (e) => {
      this._disk.setPairs(e.detail.pairs);
    });

    // Offline simulation
    cv.addEventListener("simulate", async (e) => {
      if (this._simRunning) return;
      this._simRunning = true;
      this._showSimStatus("⏳ Simulating…");

      const movesDeg = [];
      for (const p of e.detail.pairs) {
        movesDeg.push(p.pickDeg);
        movesDeg.push(p.placeDeg);
      }

      try {
        const result = await this._sim.run(movesDeg, e.detail.gains);
        this._robot.appendSimHistory(result);
        this._showSimStatus("✅ Simulation complete");
      } catch (err) {
        this._showSimStatus("❌ Simulation error");
        console.error(err);
      } finally {
        this._simRunning = false;
      }
    });

    // Send sequence to real robot
    cv.addEventListener("startSequence", (e) => {
      this._ws.sendCmd({
        type:  "sequence",
        pairs: e.detail.pairs.map(p => [p.pick, p.place]),
        n:     e.detail.pairs.length,
      });
      // Update setpoint to first pick position
      if (e.detail.pairs.length > 0) {
        this._robot.setSetpoint(e.detail.pairs[0].pickDeg);
      }
    });

    // User edited pair table manually
    cv.addEventListener("pairsEdited", (e) => {
      this._seq.setFromDeg(e.detail.map(p => [p.pickDeg || 0, p.placeDeg || 0]));
    });
  }

  // ── SequenceModel updates ──────────────────────────────────────────────────

  _bindSeqModel() {
    this._seq.subscribe((seq) => {
      this._controls.renderPairs(seq.pairs);
      this._disk.setPairs(seq.pairs);
    });
  }

  // ── RobotState updates → view cascade ─────────────────────────────────────

  _bindRobotModel() {
    // Robot state drives disk arm angle
    this._robot.subscribe((state) => {
      this._disk.setAngle(state.pos_deg);
    });
  }

  // ── Utility ───────────────────────────────────────────────────────────────

  _showSimStatus(msg) {
    const el = document.getElementById("sim-status");
    if (el) { el.textContent = msg; el.style.opacity = "1"; }
  }
}
