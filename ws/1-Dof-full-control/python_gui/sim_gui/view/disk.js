/**
 * view/disk.js — DiskView
 *
 * Renders the 72-hole circular workspace as an SVG.
 * Animates the arm at 60 fps using requestAnimationFrame with lerp smoothing.
 *
 * Mirrors the BaseSystem "WORKSPACE" panel visual design:
 *   - 72 grey dots around the ring (every 5°)
 *   - Orange arm line from centre
 *   - Coloured hole highlights: pick=orange, place=blue, both=purple
 *   - Tick mark and "0" label at top
 *   - Current position dot near rim
 */

const HOLES      = 72;
const LERP_ALPHA = 0.18;   // smoothing factor (lower = smoother/laggier)
const SVG_NS     = "http://www.w3.org/2000/svg";

// Colour palette matching BaseSystem
const COLORS = {
  arm:       "#F97316",   // orange-500
  hole_pick: "#F97316",   // orange
  hole_place:"#3B82F6",   // blue-500
  hole_both: "#A855F7",   // purple-500
  hole_curr: "#111827",   // near-black
  hole_base: "#D1D5DB",   // grey-300
  zero_mark: "#111827",
  label:     "#6B7280",   // grey-500
};

export class DiskView {
  /**
   * @param {string} containerId  - id of the host <div> element
   */
  constructor(containerId) {
    this._container = document.getElementById(containerId);
    this._displayAngle = 0;   // current rendered angle (lerped)
    this._targetAngle  = 0;   // target from robot state

    // Hole highlight sets (hole indices 0-71)
    this._pickHoles  = new Set();
    this._placeHoles = new Set();

    this._rafId = null;
    this._svg   = null;
    this._armEl = null;
    this._dotEl = null;
    this._holeEls = [];

    this._build();
    this._startLoop();
  }

  // ── Public API ─────────────────────────────────────────────────────────────

  /** Set target arm angle (degrees, 0 = top, CW positive). */
  setAngle(deg) {
    this._targetAngle = deg;
  }

  /**
   * Update hole highlights for the current sequence.
   * @param {Array<{pickDeg, placeDeg}>} pairs
   */
  setPairs(pairs) {
    this._pickHoles.clear();
    this._placeHoles.clear();
    for (const p of pairs) {
      const ph = Math.round(p.pickDeg  / 5) % HOLES;
      const lh = Math.round(p.placeDeg / 5) % HOLES;
      if (this._placeHoles.has(ph)) this._placeHoles.delete(ph), this._pickHoles.add(ph);
      else this._pickHoles.add(ph);
      if (this._pickHoles.has(lh)) this._pickHoles.delete(lh), this._placeHoles.add(lh);
      else this._placeHoles.add(lh);
    }
    this._refreshHoleColors();
  }

  clearPairs() {
    this._pickHoles.clear();
    this._placeHoles.clear();
    this._refreshHoleColors();
  }

  // ── SVG construction ───────────────────────────────────────────────────────

  _build() {
    const size  = this._container.clientWidth || 320;
    const cx    = size / 2;
    const cy    = size / 2;
    const rOuter = size * 0.44;   // hole ring radius
    const rArm   = size * 0.38;

    const svg = document.createElementNS(SVG_NS, "svg");
    svg.setAttribute("viewBox", `0 0 ${size} ${size}`);
    svg.setAttribute("width",  "100%");
    svg.setAttribute("height", "100%");
    svg.style.display = "block";

    // Background circle
    const bg = document.createElementNS(SVG_NS, "circle");
    bg.setAttribute("cx", cx); bg.setAttribute("cy", cy);
    bg.setAttribute("r", rOuter + size * 0.03);
    bg.setAttribute("fill", "#F9FAFB"); bg.setAttribute("stroke", "#E5E7EB");
    bg.setAttribute("stroke-width", "1");
    svg.appendChild(bg);

    // 72 hole dots
    this._holeEls = [];
    for (let i = 0; i < HOLES; i++) {
      const angleDeg = i * 5 - 90;   // 0° at top
      const rad = angleDeg * Math.PI / 180;
      const x = cx + rOuter * Math.cos(rad);
      const y = cy + rOuter * Math.sin(rad);

      const c = document.createElementNS(SVG_NS, "circle");
      c.setAttribute("cx", x.toFixed(1));
      c.setAttribute("cy", y.toFixed(1));
      c.setAttribute("r",  size * 0.012);
      c.setAttribute("fill", COLORS.hole_base);
      c.dataset.hole = i;
      svg.appendChild(c);
      this._holeEls.push(c);
    }

    // 0° tick mark at top
    const tickLen = size * 0.04;
    const tick = document.createElementNS(SVG_NS, "line");
    tick.setAttribute("x1", cx); tick.setAttribute("y1", cy - rOuter - size * 0.01);
    tick.setAttribute("x2", cx); tick.setAttribute("y2", cy - rOuter - tickLen);
    tick.setAttribute("stroke", COLORS.zero_mark); tick.setAttribute("stroke-width", 2);
    svg.appendChild(tick);

    // "0" label
    const lbl = document.createElementNS(SVG_NS, "text");
    lbl.setAttribute("x", cx); lbl.setAttribute("y", cy - rOuter - tickLen - size * 0.01);
    lbl.setAttribute("text-anchor", "middle"); lbl.setAttribute("font-size", size * 0.04);
    lbl.setAttribute("fill", COLORS.label); lbl.textContent = "0";
    svg.appendChild(lbl);

    // Arm (line from centre — rotated via transform)
    const armG = document.createElementNS(SVG_NS, "g");
    armG.setAttribute("transform", `rotate(0, ${cx}, ${cy})`);

    const arm = document.createElementNS(SVG_NS, "line");
    arm.setAttribute("x1", cx); arm.setAttribute("y1", cy);
    arm.setAttribute("x2", cx); arm.setAttribute("y2", cy - rArm);
    arm.setAttribute("stroke", COLORS.arm);
    arm.setAttribute("stroke-width", size * 0.012);
    arm.setAttribute("stroke-linecap", "round");
    armG.appendChild(arm);

    // Tip dot
    const tip = document.createElementNS(SVG_NS, "circle");
    tip.setAttribute("cx", cx); tip.setAttribute("cy", cy - rArm);
    tip.setAttribute("r", size * 0.022); tip.setAttribute("fill", COLORS.arm);
    armG.appendChild(tip);

    svg.appendChild(armG);
    this._armEl = armG;
    this._cx = cx; this._cy = cy;

    this._container.appendChild(svg);
    this._svg = svg;
  }

  // ── Animation loop ─────────────────────────────────────────────────────────

  _startLoop() {
    const step = () => {
      // Lerp display angle toward target (handles wraparound)
      let diff = this._targetAngle - this._displayAngle;
      // Normalise diff to (-180, 180) so arm always takes shortest arc
      while (diff >  180) diff -= 360;
      while (diff < -180) diff += 360;
      this._displayAngle += diff * LERP_ALPHA;

      // Update arm transform
      if (this._armEl) {
        this._armEl.setAttribute(
          "transform",
          `rotate(${this._displayAngle.toFixed(2)}, ${this._cx}, ${this._cy})`
        );
      }

      this._rafId = requestAnimationFrame(step);
    };
    this._rafId = requestAnimationFrame(step);
  }

  destroy() {
    if (this._rafId) cancelAnimationFrame(this._rafId);
  }

  // ── Hole colour update ─────────────────────────────────────────────────────

  _refreshHoleColors() {
    for (let i = 0; i < HOLES; i++) {
      const isPick  = this._pickHoles.has(i);
      const isPlace = this._placeHoles.has(i);
      let color = COLORS.hole_base;
      if (isPick && isPlace) color = COLORS.hole_both;
      else if (isPick)       color = COLORS.hole_pick;
      else if (isPlace)      color = COLORS.hole_place;
      if (this._holeEls[i]) this._holeEls[i].setAttribute("fill", color);
    }
  }
}
