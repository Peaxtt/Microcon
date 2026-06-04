/**
 * model/sequence.js — SequenceModel
 *
 * Manages pick-and-place pair data.
 *
 * Hole index convention (matches firmware start_move_hole):
 *   - magnitude |raw| = absolute hole on the disk (0–71)
 *   - SIGN forces motor direction:  + = CCW,  - = CW
 *   - hole 0 = home = 0°,  hole N = N × 5°
 *
 *   e.g. raw = -69 → hole 69 = 345°, motor turns CW to reach it.
 *        From hole 15 (75°), CW to hole 69 (345°) = 90° clockwise.
 */

const HOLES     = 72;    // holes per revolution
const DEG_HOLE  = 5;     // degrees per hole

/**
 * Convert signed hole index → absolute disk position in degrees.
 * Sign only encodes direction; magnitude is the absolute hole.
 * @param {number} raw - signed hole index from register
 * @returns {number} absolute target angle (degrees, 0–355)
 */
export function holeToDeg(raw) {
  return Math.abs(Math.round(raw)) * DEG_HOLE;
}

/**
 * Compute the signed displacement (degrees) to travel from posDeg to a hole,
 * forcing the direction given by the sign of raw.
 *   raw >= 0 : CCW (positive displacement)
 *   raw <  0 : CW  (negative displacement)
 * @param {number} raw    - signed hole index
 * @param {number} posDeg - current absolute position (cumulative degrees)
 * @returns {number} signed displacement to add to posDeg
 */
export function holeDisplacement(raw, posDeg) {
  const targetAbs = holeToDeg(raw);
  let posMod = posDeg % 360;
  if (posMod < 0) posMod += 360;

  if (raw >= 0) {
    // CCW: always forward
    return ((targetAbs - posMod) % 360 + 360) % 360;
  } else {
    // CW: always backward
    return -(((posMod - targetAbs) % 360 + 360) % 360);
  }
}

/**
 * Convert absolute degrees to hole index (0-71).
 */
export function degToHole(deg) {
  return Math.round(deg / DEG_HOLE) % HOLES;
}

export class SequenceModel {
  constructor() {
    /** @type {{pick: number, place: number, pickDeg: number, placeDeg: number}[]} */
    this.pairs = [];
    this._subs = [];
  }

  /**
   * Set pairs from raw signed register values (as sent by BaseSystem).
   * @param {number[][]} rawPairs - [[pickRaw, placeRaw], ...]
   */
  setFromRaw(rawPairs) {
    this.pairs = rawPairs.map(([pr, pl]) => ({
      pick:     pr,                 // signed: magnitude = hole, sign = direction
      place:    pl,
      pickDeg:  holeToDeg(pr),      // absolute disk position
      placeDeg: holeToDeg(pl),
    }));
    this._notify();
  }

  /**
   * Set pairs from absolute degree values (e.g. from disk click UI).
   * @param {number[][]} degPairs - [[pickDeg, placeDeg], ...]
   */
  setFromDeg(degPairs) {
    this.pairs = degPairs.map(([pd, pl]) => ({
      pick:     degToHole(pd),
      place:    degToHole(pl),
      pickDeg:  pd,
      placeDeg: pl,
    }));
    this._notify();
  }

  /** Calculate shortest signed distance from 'from' to 'to' in holes (-36 to +36) */
  _shortestArcHoles(from, to) {
    let diff = (to - from) % HOLES;
    if (diff < 0) diff += HOLES; // now diff is 0 to 71 (positive CCW distance)

    if (diff > HOLES / 2) {
      return diff - HOLES; // CW distance
    } else if (diff === HOLES / 2) {
      return diff; // Tie -> CCW
    }
    return diff; // CCW distance
  }

  /** Check if 'testHole' is strictly inside the arc from 'start' to 'end' taking 'step' (+1 or -1) */
  _isHoleInArc(start, end, step, testHole) {
    if (start === end) return false;
    let curr = (start + step + HOLES) % HOLES;
    while (curr !== end) {
      if (curr === testHole) return true;
      curr = (curr + step + HOLES) % HOLES;
    }
    return false;
  }

  /**
   * Generate N collision-safe random pairs matching BaseSystem's "Generate & Validate" logic.
   * 1. Total distance must be within [targetDeg - varDeg, targetDeg + varDeg].
   * 2. All moves (Home->Pick, Pick->Place) use shortest arc.
   * 3. Collision only happens if Pick->Place shortest arc passes over an existing rod.
   *    (Empty moves are safe as arm is UP).
   */
  generateRandom(n, targetDeg = 500, varDeg = 45) {
    n = Math.max(1, Math.min(8, n));
    const MAX_ITERS = 10000;
    
    let closestValidSequence = null;
    let minError = Infinity;

    for (let iter = 0; iter < MAX_ITERS; iter++) {
      // 1. Generate N unique Pick holes and N unique Place holes (2N distinct holes total)
      const usedHoles = new Set();
      const sample = () => {
        let h;
        do { h = Math.floor(Math.random() * HOLES); } while (usedHoles.has(h));
        usedHoles.add(h);
        return h;
      };

      const picks = [];
      const places = [];
      for (let i = 0; i < n; i++) picks.push(sample());
      for (let i = 0; i < n; i++) places.push(sample());

      // 2. Collision Safety Check (Pick -> Place only)
      let safe = true;
      const arcs = []; // Store shortest arc step (+1 or -1) for Pick->Place
      
      for (let i = 0; i < n; i++) {
        const pick = picks[i];
        const place = places[i];
        
        const arcToPlace = this._shortestArcHoles(pick, place);
        const step = arcToPlace >= 0 ? 1 : -1;
        arcs.push(step);

        // Rods that exist during this move:
        // Unpicked rods (j > i) and Placed rods (k < i)
        const rods = [];
        for (let j = i + 1; j < n; j++) rods.push(picks[j]);
        for (let k = 0; k < i; k++)     rods.push(places[k]);

        // If any rod is strictly inside the shortest arc, it's a crash!
        for (const rod of rods) {
          if (this._isHoleInArc(pick, place, step, rod)) {
            safe = false;
            break;
          }
        }
        if (!safe) break;
      }

      if (!safe) continue; // Reject unsafe path

      // 3. Calculate Total Travel Distance (using shortest arcs for all segments)
      let totalTravelHoles = 0;
      let pos = 0; // BaseSystem assumes sequence starts from Home (0)
      
      for (let i = 0; i < n; i++) {
        // Move empty to Pick
        totalTravelHoles += Math.abs(this._shortestArcHoles(pos, picks[i]));
        pos = picks[i];
        
        // Move with rod to Place
        totalTravelHoles += Math.abs(this._shortestArcHoles(pos, places[i]));
        pos = places[i];
      }

      const totalTravelDeg = totalTravelHoles * DEG_HOLE;
      const error = Math.abs(totalTravelDeg - targetDeg);

      // Keep track of the safest closest sequence in case we never hit the strict bounds
      if (error < minError) {
        minError = error;
        closestValidSequence = { picks, places, arcs };
      }

      // Check strict Distance Constraint
      if (totalTravelDeg >= targetDeg - varDeg && totalTravelDeg <= targetDeg + varDeg) {
        break; // Found perfect match!
      }
    }

    const best = closestValidSequence;
    if (!best) {
      console.warn("Failed to generate any valid sequence. This should be mathematically impossible with 72 holes.");
      return []; // Extremely edge case
    }

    // 4. Assemble pairs with correct signs to enforce shortest arcs in server.py
    const result = [];
    let pos = 0;
    
    for (let i = 0; i < n; i++) {
      const p = best.picks[i];
      const t = best.places[i];

      // Sign Pick index for shortest arc from current 'pos'
      const pickDiff = this._shortestArcHoles(pos, p);
      let pickSigned = p;
      if (pickDiff < 0) {
        pickSigned = p === 0 ? -HOLES : -p;
      }
      pos = p;

      // Sign Place index for shortest arc from 'p'
      const placeDiff = this._shortestArcHoles(pos, t);
      let placeSigned = t;
      if (placeDiff < 0) {
        placeSigned = t === 0 ? -HOLES : -t;
      }
      pos = t;

      result.push({
        pick: pickSigned,
        place: placeSigned,
        pickDeg: p * DEG_HOLE,
        placeDeg: t * DEG_HOLE,
        dir: placeDiff >= 0 ? "CCW" : "CW" // For UI
      });
    }

    this.pairs = result;
    this._notify();
    return result;
  }

  /**
   * Returns the movement plan as an array of steps (for path preview on disk).
   * Each step: { from, to, type }  where type = "move" | "pick" | "place"
   */
  getPathSteps() {
    const steps = [];
    for (const p of this.pairs) {
      steps.push({ from: null,       to: p.pickDeg,  type: "goto_pick"  });
      steps.push({ from: p.pickDeg,  to: p.pickDeg,  type: "pick"       });
      steps.push({ from: p.pickDeg,  to: p.placeDeg, type: "goto_place" });
      steps.push({ from: p.placeDeg, to: p.placeDeg, type: "place"      });
    }
    return steps;
  }

  /**
   * Export pairs in the register format expected by server.py "sequence" command.
   * Returns [[pickRaw, placeRaw], ...]
   */
  toRaw() {
    return this.pairs.map(p => [p.pick, p.place]);
  }

  subscribe(fn) { this._subs.push(fn); }
  _notify()     { this._subs.forEach(fn => fn(this)); }
}
