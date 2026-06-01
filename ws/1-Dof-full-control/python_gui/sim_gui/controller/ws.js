/**
 * controller/ws.js — WebSocketController
 *
 * Manages the WebSocket connection to server.py.
 * Parses incoming telemetry JSON and updates the RobotState model.
 * Exposes sendCmd() for sending command objects to the server.
 */

const WS_URL        = "ws://localhost:8765";
const RECONNECT_MS  = 2000;

export class WebSocketController {
  /**
   * @param {import('../model/robot.js').RobotState} robotState
   */
  constructor(robotState) {
    this._state   = robotState;
    this._ws      = null;
    this._queue   = [];       // commands buffered while disconnected
    this._connecting = false;
    this._onStatusChange = null;

    this._connect();
  }

  /** Send a command object to server.py. Queued if not yet connected. */
  sendCmd(cmd) {
    if (this._ws?.readyState === WebSocket.OPEN) {
      this._ws.send(JSON.stringify(cmd));
    } else {
      this._queue.push(cmd);
    }
  }

  /** Register a callback to receive connection status changes. */
  onStatusChange(fn) { this._onStatusChange = fn; }

  // ── Connection management ──────────────────────────────────────────────────

  _connect() {
    if (this._connecting) return;
    this._connecting = true;

    const ws = new WebSocket(WS_URL);
    this._ws = ws;

    ws.onopen = () => {
      this._connecting = false;
      this._onStatusChange?.("connected");
      // Flush queued commands
      while (this._queue.length) {
        ws.send(JSON.stringify(this._queue.shift()));
      }
    };

    ws.onmessage = (evt) => {
      try {
        const pkt = JSON.parse(evt.data);
        this._state.applyPacket(pkt);
      } catch (e) {
        console.warn("[ws] parse error:", e);
      }
    };

    ws.onerror = () => {
      this._onStatusChange?.("error");
    };

    ws.onclose = () => {
      this._connecting = false;
      this._onStatusChange?.("disconnected");
      // Auto-reconnect
      setTimeout(() => this._connect(), RECONNECT_MS);
    };
  }
}
