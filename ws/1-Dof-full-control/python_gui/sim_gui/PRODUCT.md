# Product

## Register

product

## Users

Engineering students and control-team members testing a 1-DOF pick-and-place robot arm.
Context: lab session, laptop on workbench, no robot present yet — verifying pick/place paths
and PID gains before hardware test day.

## Product Purpose

Simulate and visualize the robot's motion, actuator states, and control response offline.
Verify that commanded hole positions, sequence order, and gripper timing are correct before
running on real hardware. Success = team catches errors in simulation, not during hardware test.

## Brand Personality

Precise. Functional. Calm under load.

The interface should feel like a professional oscilloscope or CNC controller panel —
every element is there because it earns its place. Orange comes from BaseSystem
(the real operator interface); we inherit it to signal continuity.

## Anti-references

- Consumer dashboards with gradients and hero imagery
- Dark-mode-everything for no reason
- Rounded cards with drop shadows stacked inside more rounded cards
- Cluttered toolbars with icons that need tooltips to understand

## Design Principles

1. **Control room, not playground** — neutral white surface, colour only in data and actions
2. **Status at a glance** — reed dots and arm position must be readable in peripheral vision
3. **One orange** — F97316 is the system accent; use it only for active/moving/commanded states
4. **No decoration** — borders, spacing, and typography do all the layout work
5. **Sim = real** — visual language mirrors BaseSystem so behaviour is predictable on hardware

## Accessibility & Inclusion

WCAG AA minimum. All interactive targets ≥ 36px. Status indicators must not rely on colour alone.
