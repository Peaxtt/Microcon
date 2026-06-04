/**
 * view/controls.js — ControlsView
 *
 * Manages all user-facing controls in the right panel:
 *   MANUAL tab  : gripper buttons, jog CCW/CW, tuning sliders
 *   AUTO tab    : pick-and-place pair table, random gen, P2P section
 *   TEST MODE   : placeholder
 *
 * Emits events via EventTarget so controllers can listen without tight coupling.
 */

// Actuator button ids and their "active" condition based on reed state
// Each button highlights independently based on its own reed signal only
const ACTUATOR_BUTTONS = [
  { id: "btn-up",    active: (s) => s.reed_up   },   // cylinder UP reed
  { id: "btn-down",  active: (s) => s.reed_down },   // cylinder DOWN reed
  { id: "btn-close", active: (s) =>  s.reed_grip },  // gripper CLOSED reed
  { id: "btn-open",  active: (s) => !s.reed_grip },  // gripper OPEN = no grip reed
];

export class ControlsView extends EventTarget {
  constructor() {
    super();
    this._gainState = {
      kp_vel: 10.0, ki_vel: 0.01, kd_vel: 0.0,
      kp_pos:  2.0, kd_pos:  0.2, ki_pos: 0.7,
      v_max:   3.0, a_max:  12.56,
      max_speed:    0.4,
      seq_dwell_ms: 200,   // matches firmware default
    };
    this._pairs = [];     // [{pick, place, pickDeg, placeDeg}]
    this._nTarget = 3;
    this._jogDeg  = 10;

    this._bindTabs();
    this._bindManual();
    this._bindAuto();
    this._bindTuning();
  }

  // ── Tab switching ──────────────────────────────────────────────────────────
  _bindTabs() {
    document.querySelectorAll(".tab-btn").forEach(btn => {
      btn.addEventListener("click", () => {
        document.querySelectorAll(".tab-btn").forEach(b => b.classList.remove("active"));
        document.querySelectorAll(".tab-panel").forEach(p => p.classList.add("hidden"));
        btn.classList.add("active");
        const panelId = btn.dataset.tab + "-panel";
        document.getElementById(panelId)?.classList.remove("hidden");
      });
    });
  }

  // ── MANUAL tab ─────────────────────────────────────────────────────────────
  _bindManual() {
    const emit = (type, detail) => this.dispatchEvent(new CustomEvent(type, { detail }));

    // Gripper buttons: UP / DOWN / OPEN / CLOSE / PICK / PLACE
    const btns = {
      "btn-up":    () => emit("cmd", { type: "gripper", val: "up"    }),
      "btn-down":  () => emit("cmd", { type: "gripper", val: "down"  }),
      "btn-open":  () => emit("cmd", { type: "gripper", val: "open"  }),
      "btn-close": () => emit("cmd", { type: "gripper", val: "close" }),
      "btn-pick":  () => emit("cmd", { type: "write_reg", reg: 0x03, val: 1 }),
      "btn-place": () => emit("cmd", { type: "write_reg", reg: 0x03, val: 2 }),
    };
    for (const [id, fn] of Object.entries(btns)) {
      document.getElementById(id)?.addEventListener("click", fn);
    }

    // Jog CCW / CW
    document.getElementById("btn-jog-ccw")?.addEventListener("click", () => {
      emit("cmd", { type: "jog", deg: this._jogDeg });
    });
    document.getElementById("btn-jog-cw")?.addEventListener("click", () => {
      emit("cmd", { type: "jog", deg: -this._jogDeg });
    });
    const jogInput = document.getElementById("jog-deg");
    jogInput?.addEventListener("change", () => {
      this._jogDeg = parseFloat(jogInput.value) || 10;
    });

    // SET HOME / HOME buttons
    document.getElementById("btn-set-home")?.addEventListener("click", () => {
      emit("cmd", { type: "set_home" });
    });
    document.getElementById("btn-home")?.addEventListener("click", () => {
      emit("cmd", { type: "home" });
    });

    // STOP button
    document.getElementById("btn-stop")?.addEventListener("click", () => {
      emit("cmd", { type: "stop" });
    });
  }

  // ── AUTO tab ───────────────────────────────────────────────────────────────
  _bindAuto() {
    const emit = (type, detail) => this.dispatchEvent(new CustomEvent(type, { detail }));

    // Number of targets spinner
    const nInput = document.getElementById("n-target");
    if (nInput) {
      nInput.value = this._nTarget;
      nInput.addEventListener("change", () => {
        this._nTarget = Math.max(1, Math.min(8, parseInt(nInput.value) || 3));
        this._renderPairTable();
      });
    }

    // Random shuffle button
    document.getElementById("btn-random")?.addEventListener("click", () => {
      const tDeg = parseFloat(document.getElementById("target-deg")?.value) || 500;
      const vDeg = parseFloat(document.getElementById("var-deg")?.value) || 45;
      emit("randomize", { n: this._nTarget, targetDeg: tDeg, varDeg: vDeg });
    });

    // Preview path button
    document.getElementById("btn-preview")?.addEventListener("click", () => {
      emit("preview", { pairs: this._pairs });
    });

    // Simulate button
    document.getElementById("btn-simulate")?.addEventListener("click", () => {
      emit("simulate", { pairs: this._pairs, gains: this._gainState });
    });

    // START button (send to real robot)
    document.getElementById("btn-start-seq")?.addEventListener("click", () => {
      emit("startSequence", { pairs: this._pairs });
    });

    // Gripper checkbox
    const grpCheck = document.getElementById("gripper-check");
    grpCheck?.addEventListener("change", () => {
      emit("gripperToggle", { enabled: grpCheck.checked });
    });

    // P2P section
    document.getElementById("btn-p2p-go")?.addEventListener("click", () => {
      const holeInput = document.getElementById("p2p-hole");
      emit("cmd", { type: "p2p", hole: parseInt(holeInput?.value || 0) });
    });
    document.querySelectorAll(".p2p-step").forEach(btn => {
      btn.addEventListener("click", () => {
        const delta = parseInt(btn.dataset.delta || 0);
        const input = document.getElementById("p2p-hole");
        if (input) input.value = (parseInt(input.value || 0) + delta);
        emit("cmd", { type: "p2p", hole: parseInt(input?.value || 0) });
      });
    });
  }

  // ── Tuning panel ───────────────────────────────────────────────────────────
  _bindTuning() {
    const PARAM_DEFS = [
      { id: "kp_vel",       min: 0,   max: 30,   step: 0.1,   label: "kp_vel" },
      { id: "ki_vel",       min: 0,   max: 2,    step: 0.001, label: "ki_vel" },
      { id: "kp_pos",       min: 0,   max: 10,   step: 0.1,   label: "kp_pos" },
      { id: "kd_pos",       min: 0,   max: 2,    step: 0.01,  label: "kd_pos" },
      { id: "v_max",        min: 0.5, max: 15,   step: 0.1,   label: "v_max (rad/s)" },
      { id: "a_max",        min: 1,   max: 80,   step: 0.5,   label: "a_max (rad/s²)" },
      { id: "max_speed",    min: 0.1, max: 1,    step: 0.01,  label: "max_speed" },
      // seq_dwell_ms: shared between sim reed delay and board register
      { id: "seq_dwell_ms", min: 50,  max: 3000, step: 50,    label: "seq_dwell (ms)" },
    ];

    const container = document.getElementById("tuning-panel");
    if (!container) return;

    for (const p of PARAM_DEFS) {
      const row = document.createElement("div");
      row.className = "tuning-row";
      row.innerHTML = `
        <label>${p.label}</label>
        <input type="range" id="sl-${p.id}"
               min="${p.min}" max="${p.max}" step="${p.step}"
               value="${this._gainState[p.id] ?? p.min}">
        <input type="number" id="nv-${p.id}"
               min="${p.min}" max="${p.max}" step="${p.step}"
               value="${this._gainState[p.id] ?? p.min}">
      `;
      container.appendChild(row);

      const slider = row.querySelector(`#sl-${p.id}`);
      const numbox = row.querySelector(`#nv-${p.id}`);

      const sync = (src, dst) => () => {
        const v = parseFloat(src.value);
        if (!isNaN(v)) { this._gainState[p.id] = v; dst.value = v; }
      };
      slider.addEventListener("input",  sync(slider, numbox));
      numbox.addEventListener("change", sync(numbox, slider));
    }

    document.getElementById("btn-apply-gains")?.addEventListener("click", () => {
      this.dispatchEvent(new CustomEvent("cmd", {
        detail: { type: "gains", ...this._gainState }
      }));
    });
  }

  // ── Pair table render ──────────────────────────────────────────────────────
  renderPairs(pairs) {
    this._pairs = pairs;
    this._renderPairTable();
  }

  /**
   * Update actuator button highlight colors from robot state.
   * Called by EventController on every robot state update.
   * @param {import('../model/robot.js').RobotState} state
   */
  updateActuatorState(state) {
    for (const { id, active } of ACTUATOR_BUTTONS) {
      const btn = document.getElementById(id);
      if (!btn) continue;
      if (active(state)) {
        btn.classList.add("btn-active");
      } else {
        btn.classList.remove("btn-active");
      }
    }
  }

  _renderPairTable() {
    const tbody = document.getElementById("pair-table-body");
    if (!tbody) return;
    tbody.innerHTML = "";
    const n = this._nTarget;

    for (let i = 0; i < n; i++) {
      const p = this._pairs[i] ?? { pick: 0, place: 0, pickDeg: 0, placeDeg: 0 };
      const tr = document.createElement("tr");
      tr.innerHTML = `
        <td>${i + 1}</td>
        <td>
          <input type="number" class="hole-input" data-row="${i}" data-col="pick"
                 value="${p.pick}" min="-71" max="71">
          <span class="hole-deg">${p.pickDeg.toFixed(0)}°</span>
        </td>
        <td>
          <input type="number" class="hole-input" data-row="${i}" data-col="place"
                 value="${p.place}" min="-71" max="71">
          <span class="hole-deg">${p.placeDeg.toFixed(0)}°</span>
        </td>
        <td>${Math.abs(p.placeDeg - p.pickDeg).toFixed(0)}°</td>
      `;
      tbody.appendChild(tr);
    }

    // Re-bind input changes
    tbody.querySelectorAll(".hole-input").forEach(inp => {
      inp.addEventListener("change", () => {
        const r = parseInt(inp.dataset.row);
        const c = inp.dataset.col;
        if (!this._pairs[r]) this._pairs[r] = { pick: 0, place: 0, pickDeg: 0, placeDeg: 0 };
        this._pairs[r][c] = parseInt(inp.value) || 0;
        // Recalculate degrees on edit
        this.dispatchEvent(new CustomEvent("pairsEdited", { detail: this._pairs }));
      });
    });
  }
}
