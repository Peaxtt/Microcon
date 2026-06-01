/**
 * model/sequence.js — SequenceModel
 *
 * Manages pick-and-place pair data.
 * Converts hole indices ↔ degrees using the same formula as firmware:
 *
 *   pick  >= 0 : absolute left-side hole → pick × 5°
 *   pick  <  0 : right-side hole → (72 + pick) × 5°
 *   place >= 0 : absolute → place × 5°
 *   place <  0 : relative to pick → (pick_hole + 72 + place) × 5°
 *
 * Also handles random valid placement generation.
 */

const HOLES     = 72;    // holes per revolution
const DEG_HOLE  = 5;     // degrees per hole

/**
 * Convert (possibly negative) hole index to absolute degrees.
 * @param {number} raw        - signed hole index from register
 * @param {number} pickHole   - absolute pick hole (used only for place calculation)
 * @param {boolean} isPlace   - true = use relative formula for negative values
 */
export function holeToDeg(raw, pickHole = 0, isPlace = false) {
  raw = Math.round(raw);
  if (raw >= 0) {
    return raw * DEG_HOLE;
  }
  // Negative index
  if (isPlace) {
    // relative: go |raw| holes right from pick → (pickHole + 72 + raw) × 5
    return (pickHole + HOLES + raw) * DEG_HOLE;
  } else {
    // pick: wrap around disk → (72 + raw) × 5
    return (HOLES + raw) * DEG_HOLE;
  }
}

/**
 * Convert absolute degrees to hole index (0-71).
 * Returns nearest hole index.
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
    this.pairs = rawPairs.map(([pr, pl]) => {
      const pickHole  = pr >= 0 ? pr : (HOLES + pr);
      const placeHole = pl >= 0 ? pl : (pickHole + HOLES + pl);
      return {
        pick:     pr,
        place:    pl,
        pickDeg:  pickHole  * DEG_HOLE,
        placeDeg: placeHole * DEG_HOLE,
      };
    });
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

  /**
   * Generate N random pairs within a degree range.
   * Ensures: no two picks or places on the same hole, move ≤ maxMoveDeg.
   *
   * @param {number} n          - number of pairs
   * @param {number} centerDeg  - center of the target zone (degrees)
   * @param {number} varDeg     - ±variance around center
   * @param {number} maxMoveDeg - max angular distance between pick and place (default 180°)
   */
  generateRandom(n, centerDeg = 180, varDeg = 90, maxMoveDeg = 180) {
    const used = new Set();
    const randHole = () => {
      const minH = Math.max(0, Math.round((centerDeg - varDeg) / DEG_HOLE));
      const maxH = Math.min(HOLES - 1, Math.round((centerDeg + varDeg) / DEG_HOLE));
      let h, attempts = 0;
      do {
        h = minH + Math.floor(Math.random() * (maxH - minH + 1));
        attempts++;
      } while (used.has(h) && attempts < 200);
      used.add(h);
      return h;
    };

    const pairs = [];
    for (let i = 0; i < n; i++) {
      let ph, lh, move, tries = 0;
      do {
        ph   = randHole();
        lh   = randHole();
        move = Math.abs(ph - lh) * DEG_HOLE;
        if (move > 180) move = 360 - move;   // shortest arc
        tries++;
      } while ((move > maxMoveDeg || ph === lh) && tries < 100);

      pairs.push({
        pick:     ph,
        place:    lh,
        pickDeg:  ph * DEG_HOLE,
        placeDeg: lh * DEG_HOLE,
      });
    }
    this.pairs = pairs;
    this._notify();
    return pairs;
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
