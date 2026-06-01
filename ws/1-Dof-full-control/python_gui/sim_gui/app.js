/**
 * app.js — Application entry point
 *
 * Instantiates all MVC components and wires them together.
 * Load order: models → views → controllers → event bindings.
 *
 * How to use:
 *   1. Start server: python python_gui/server.py [--port COM4]
 *   2. Open this file in browser:  python_gui/sim_gui/index.html
 *
 * Simulation mode (no --port):  server generates fake motion for testing.
 * Hardware mode   (--port COMx): server reads real Modbus registers at 50 Hz.
 */

// ── Model imports ──────────────────────────────────────────────────────────────
import { RobotState }     from './model/robot.js';
import { SequenceModel }  from './model/sequence.js';
import { SimulatorModel } from './model/simulator.js';

// ── View imports ───────────────────────────────────────────────────────────────
import { DiskView }     from './view/disk.js';
import { GraphView }    from './view/graphs.js';
import { StatsView }    from './view/stats.js';
import { ControlsView } from './view/controls.js';

// ── Controller imports ─────────────────────────────────────────────────────────
import { WebSocketController } from './controller/ws.js';
import { EventController }     from './controller/events.js';

// ─────────────────────────────────────────────────────────────────────────────

function main() {
  // ── 1. Models ──────────────────────────────────────────────────────────────
  const robot    = new RobotState();
  const sequence = new SequenceModel();
  const sim      = new SimulatorModel();

  // ── 2. Views ───────────────────────────────────────────────────────────────
  const disk = new DiskView("disk-container");

  const graphs = new GraphView("graph-canvas");

  const stats = new StatsView({
    pos:       "stat-pos",
    speed:     "stat-speed",
    accel:     "stat-accel",
    gripper:   "stat-gripper",
    mode:      "stat-mode",
    emergency: "stat-emer",
    heartbeat: "stat-heartbeat",
  });

  const controls = new ControlsView();

  // ── 3. Subscribe views to robot model ──────────────────────────────────────
  robot.subscribe((state) => {
    stats.update(state);
    graphs.update(state);
    // disk.setAngle is handled by EventController to avoid duplication
  });

  // ── 4. Controllers ─────────────────────────────────────────────────────────
  const ws = new WebSocketController(robot);

  // Log connection status changes
  ws.onStatusChange((status) => {
    const log   = document.getElementById("log-panel");
    const now   = new Date().toLocaleTimeString("th", { hour12: false });
    const color = status === "connected" ? "var(--green)"
                : status === "error"     ? "var(--red)"
                : "var(--ink-muted)";
    if (log) {
      log.innerHTML =
        `<span style="color:var(--ink-muted)">[${now}]</span> ` +
        `<span style="color:${color}">${
          status === "connected"    ? "Connected to server" :
          status === "disconnected" ? "Disconnected — retrying…" :
          "Connection error"
        }</span>`;
    }
  });

  const events = new EventController(ws, robot, sequence, sim, controls, disk, graphs);

  // ── 5. Misc bindings ───────────────────────────────────────────────────────

  // Clear graph button
  document.getElementById("btn-clear-graph")?.addEventListener("click", () => {
    robot.clearHistory();
    graphs.clearHistory();
  });

  // Initialise pair table with 3 empty rows on load
  sequence.generateRandom(3, 180, 90);

  // ── 6. Dev helper: expose globals for browser console debugging ────────────
  Object.assign(window, { robot, sequence, sim, ws, disk, graphs, controls });
}

// Run after DOM is ready
if (document.readyState === "loading") {
  document.addEventListener("DOMContentLoaded", main);
} else {
  main();
}
