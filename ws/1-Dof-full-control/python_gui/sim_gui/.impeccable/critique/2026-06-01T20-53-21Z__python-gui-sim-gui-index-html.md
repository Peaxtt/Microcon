---
target: python_gui/sim_gui/index.html
total_score: 22
p0_count: 0
p1_count: 2
timestamp: 2026-06-01T20-53-21Z
slug: python-gui-sim-gui-index-html
---
## Design Health Score

| # | Heuristic | Score | Key Issue |
|---|-----------|-------|-----------|
| 1 | Visibility of System Status | 3 | Stat values show 0.0 in orange before first data |
| 2 | Match System / Real World | 3 | Domain terms correct; P2P unabbreviated would help |
| 3 | User Control and Freedom | 2 | No cancel for in-progress sequence; no confirmation before START |
| 4 | Consistency and Standards | 3 | Duplicate reed indicators — rdot-* in HTML vs gv-dot-* in GripperView, one is dead |
| 5 | Error Prevention | 2 | No hole range shown; no warning before START SEQUENCE |
| 6 | Recognition Rather Than Recall | 2 | Valid hole range unknown; Kp/Ki/Kd unexplained; tab state not in status |
| 7 | Flexibility and Efficiency | 2 | No keyboard shortcuts for jog; no power-user path |
| 8 | Aesthetic and Minimalist Design | 3 | Eyebrow labels on every card; orange on all stats at rest |
| 9 | Error Recovery | 1 | "Disconnected — retrying…" in small mono log; no recovery guidance |
| 10 | Help and Documentation | 1 | No tooltips; TEST MODE tab says "send via BaseSystem v1.1" |
| **Total** | | **22/40** | **Acceptable — significant improvements needed** |

## Anti-Patterns Verdict

LLM: Not AI-generated at first glance. Orange discipline and three-column layout are intentional. Fails on uniform card-title eyebrow (100% saturation of the Absolute Ban pattern) and flat type scale.

Detector: 2 findings — flat-type-hierarchy (11/12/13px, ratio 1.09-1.2) and em-dash-overuse (7 instances, mostly [--:--:--] placeholder).

## Priority Issues

[P1] Dead reed indicators in PNEUMATIC card — rdot-* elements never wired in app.js; gv-dot-* in GripperView update correctly. Remove rdot-* from HTML.

[P1] Orange stat values at rest violate "one orange" — .stat-value always orange including at 0.0 idle. Default to --ink; apply --primary only when robot is active.

[P2] Flat typography — 11/12/13px no hierarchy. Raise stat values to 15-16px; target 1.3 ratio.

[P2] All-caps tracked eyebrow on every card — Absolute Ban pattern at 100% frequency. Replace with border-top or increased padding.

[P3] No confirmation before START SEQUENCE — two-step button (arm then confirm).

## Persona Red Flags

Alex: No keyboard jog shortcuts. No keyboard START trigger.

Sam: Reed dots color-only. No ARIA live regions. Inputs have no semantic label associations.

Kai (Lab Student): Hole range not shown. Homed requirement invisible. TEST MODE tab is placeholder.

## Minor Observations

--muted at oklch(0.52) is ~4.2:1 at 11px — below WCAG AA. Darken to 0.48. btn-active uses !important. Em-dash finder mostly hits [--:--:--] placeholder pattern.
