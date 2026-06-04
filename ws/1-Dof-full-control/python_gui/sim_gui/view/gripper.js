/**
 * view/gripper.js — GripperView
 *
 * Industrial pneumatic gripper simulation.
 * Visual: cylinder housing → piston rod → mounting flange → base plate → jaws.
 * State driven by RobotState.reed_down (isDown) and .reed_grip (isClosed).
 * CSS transitions handle all animation (0.38s cubic-bezier, no rAF needed).
 */

const GRIPPER_CSS = `
.gv-root {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 10px;
  padding: 6px 0 4px;
  font-family: var(--mono, monospace);
  user-select: none;
}

/* ── Scene container ─────────────────────────────── */
.gv-scene {
  position: relative;
  display: flex;
  flex-direction: column;
  align-items: center;
  width: 110px;
  height: 175px;
  padding-top: 6px;
}

/* ── Cylinder housing (fixed to shaft) ───────────── */
.gv-housing {
  width: 28px;
  height: 56px;
  background: linear-gradient(160deg,
    oklch(0.30 0.006 220) 0%,
    oklch(0.40 0.005 220) 35%,
    oklch(0.36 0.006 220) 65%,
    oklch(0.26 0.006 220) 100%);
  border-radius: 4px 4px 2px 2px;
  position: relative;
  flex-shrink: 0;
  z-index: 3;
  box-shadow:
    0 4px 12px oklch(0 0 0 / 0.45),
    inset 0 1px 0 oklch(1 0 0 / 0.08),
    inset -1px 0 0 oklch(0 0 0 / 0.20);
}

/* Left pneumatic port */
.gv-housing::before {
  content: '';
  position: absolute;
  left: -6px;
  top: 10px;
  width: 7px;
  height: 10px;
  background: oklch(0.38 0.005 220);
  border-radius: 2px 0 0 2px;
  box-shadow: -1px 0 4px oklch(0 0 0 / 0.40);
}

/* Right pneumatic port */
.gv-housing::after {
  content: '';
  position: absolute;
  right: -6px;
  top: 30px;
  width: 7px;
  height: 10px;
  background: oklch(0.38 0.005 220);
  border-radius: 0 2px 2px 0;
  box-shadow: 1px 0 4px oklch(0 0 0 / 0.40);
}

/* ── Piston rod (chrome, visible between housing and assembly) ── */
.gv-rod {
  width: 8px;
  height: 13px;
  background: linear-gradient(to right,
    oklch(0.70 0.005 200),
    oklch(0.92 0.002 200),
    oklch(0.70 0.005 200));
  flex-shrink: 0;
  z-index: 2;
  margin-top: -1px;
}

/* ── Moving assembly: flange + base + jaws ───────── */
.gv-assembly {
  display: flex;
  flex-direction: column;
  align-items: center;
  transform: translateY(0);
  transition: transform 0.38s cubic-bezier(0.4, 0, 0.2, 1);
  z-index: 1;
}
.gv-assembly.is-down { transform: translateY(32px); }

/* Mounting flange (wider than base) */
.gv-flange {
  width: 76px;
  height: 7px;
  background: linear-gradient(to bottom,
    oklch(0.58 0.004 200),
    oklch(0.46 0.005 200));
  border-radius: 2px 2px 0 0;
  box-shadow: 0 1px 4px oklch(0 0 0 / 0.30);
}

/* Base plate */
.gv-base {
  width: 66px;
  height: 15px;
  background: linear-gradient(to bottom,
    oklch(0.72 0.004 200),
    oklch(0.58 0.004 200));
  border-bottom: 2px solid oklch(0.40 0.005 220);
  box-shadow:
    0 2px 6px oklch(0 0 0 / 0.35),
    inset 0 1px 0 oklch(1 0 0 / 0.06);
  display: flex;
  align-items: center;
  justify-content: center;
}

.gv-base-piston {
  width: 7px;
  height: 7px;
  background: oklch(0.82 0.003 200);
  border-radius: 50%;
  box-shadow: inset 0 1px 2px oklch(0 0 0 / 0.35);
}

/* ── Jaw container ───────────────────────────────── */
.gv-jaws {
  display: flex;
  justify-content: space-between;
  width: 66px;
}

/* Individual jaw */
.gv-jaw {
  width: 22px;
  height: 56px;
  position: relative;
  background: linear-gradient(to bottom,
    oklch(0.27 0.06 232) 0%,
    oklch(0.32 0.08 236) 55%,
    oklch(0.24 0.06 232) 100%);
  box-shadow: 0 4px 10px oklch(0 0 0 / 0.45);
  transition:
    transform 0.38s cubic-bezier(0.4, 0, 0.2, 1),
    background 0.30s;
  overflow: hidden;
}

/* Grip serration texture at jaw face */
.gv-jaw::before {
  content: '';
  position: absolute;
  bottom: 0;
  left: 0;
  right: 0;
  height: 20px;
  background: repeating-linear-gradient(
    0deg,
    oklch(0 0 0 / 0)    0px,
    oklch(0 0 0 / 0)    2px,
    oklch(0 0 0 / 0.28) 2px,
    oklch(0 0 0 / 0.28) 3px
  );
  z-index: 1;
}

/* Jaw tip cap */
.gv-jaw::after {
  content: '';
  position: absolute;
  bottom: 0;
  left: 0;
  right: 0;
  height: 7px;
  background: oklch(0.20 0.04 232);
  border-radius: 0 0 3px 3px;
  z-index: 2;
}

/* Open positions */
.gv-jaw-left  { border-radius: 2px 0 0 3px; transform: translateX(-10px); }
.gv-jaw-right { border-radius: 0 2px 3px 0; transform: translateX(10px); }

/* Closed: jaws move inward, color shifts orange (grip force) */
.gv-jaw.is-closed {
  background: linear-gradient(to bottom,
    oklch(0.36 0.10 40) 0%,
    oklch(0.44 0.13 38) 55%,
    oklch(0.32 0.10 40) 100%);
}
.gv-jaw-left.is-closed  { transform: translateX(9px); }
.gv-jaw-right.is-closed { transform: translateX(-9px); }

/* ── Reed indicator dots ─────────────────────────── */
.gv-reeds {
  display: flex;
  gap: 14px;
  align-items: center;
}
.gv-reed-item {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 3px;
}
.gv-dot {
  width: 8px;
  height: 8px;
  border-radius: 50%;
  background:  oklch(0.88 0.004 200);
  border:      1.5px solid oklch(0.78 0.006 200);
  transition:  background 0.14s, border-color 0.14s, box-shadow 0.14s;
}
.gv-dot.on {
  background:   oklch(0.60 0.17 145);
  border-color: oklch(0.50 0.17 145);
  box-shadow:   0 0 5px oklch(0.60 0.17 145 / 0.55);
}
.gv-reed-label {
  font-size:      9px;
  color:          oklch(0.55 0.006 200);
  letter-spacing: 0.03em;
}

/* ── Status text ─────────────────────────────────── */
.gv-status {
  font-size:      10px;
  font-weight:    700;
  color:          oklch(0.35 0.008 200);
  letter-spacing: 0.04em;
  white-space:    nowrap;
}

/* ── Reduced motion ──────────────────────────────── */
@media (prefers-reduced-motion: reduce) {
  .gv-assembly,
  .gv-jaw { transition: none !important; }
}
`;

let cssInjected = false;

export class GripperView {
  /** @param {string} containerId */
  constructor(containerId) {
    this._el = document.getElementById(containerId);
    if (!cssInjected) {
      const s = document.createElement("style");
      s.textContent = GRIPPER_CSS;
      document.head.appendChild(s);
      cssInjected = true;
    }
    this._build();
  }

  // ── DOM construction ───────────────────────────────────────────────────────

  _build() {
    this._el.innerHTML = `
      <div class="gv-root">

        <div class="gv-scene">
          <div class="gv-housing"></div>
          <div class="gv-rod"></div>
          <div class="gv-assembly" id="gv-assembly">
            <div class="gv-flange"></div>
            <div class="gv-base"><div class="gv-base-piston"></div></div>
            <div class="gv-jaws">
              <div class="gv-jaw gv-jaw-left"  id="gv-jaw-l"></div>
              <div class="gv-jaw gv-jaw-right" id="gv-jaw-r"></div>
            </div>
          </div>
        </div>

        <div class="gv-status" id="gv-status">UP · OPEN</div>

        <div class="gv-reeds">
          <div class="gv-reed-item">
            <div class="gv-dot" id="gv-dot-up"></div>
            <div class="gv-reed-label">UP</div>
          </div>
          <div class="gv-reed-item">
            <div class="gv-dot" id="gv-dot-down"></div>
            <div class="gv-reed-label">DOWN</div>
          </div>
          <div class="gv-reed-item">
            <div class="gv-dot" id="gv-dot-grip"></div>
            <div class="gv-reed-label">GRIP</div>
          </div>
        </div>

      </div>
    `;

    this._assembly = document.getElementById("gv-assembly");
    this._jawL     = document.getElementById("gv-jaw-l");
    this._jawR     = document.getElementById("gv-jaw-r");
    this._statusEl = document.getElementById("gv-status");
    this._dotUp    = document.getElementById("gv-dot-up");
    this._dotDown  = document.getElementById("gv-dot-down");
    this._dotGrip  = document.getElementById("gv-dot-grip");
  }

  // ── State update ───────────────────────────────────────────────────────────

  /**
   * @param {import('../model/robot.js').RobotState} state
   */
  update(state) {
    const isDown   = state.reed_down;
    const isClosed = state.reed_grip;

    this._assembly.classList.toggle("is-down",   isDown);
    this._jawL.classList.toggle("is-closed", isClosed);
    this._jawR.classList.toggle("is-closed", isClosed);

    this._setDot(this._dotUp,   state.reed_up);
    this._setDot(this._dotDown, state.reed_down);
    this._setDot(this._dotGrip, state.reed_grip);

    if (this._statusEl) {
      const z = isDown   ? "DOWN" : "UP";
      const g = isClosed ? "CLOSED" : "OPEN";
      this._statusEl.textContent = `${z} · ${g}`;
    }
  }

  _setDot(el, on) { if (el) el.classList.toggle("on", on); }

  destroy() {}
}
