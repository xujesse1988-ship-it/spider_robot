# English versions of the HTML reports

Every file here is a full English translation of the same-named file in [`html/`](../). The Chinese originals are the maintained source; if the two differ, the Chinese version wins. Each page is self-contained (only Google Fonts are loaded from the network) and opens directly in a browser.

## Experiment reports (in date order)

| File | What it is |
|---|---|
| [press-lurch-20260819.html](press-lurch-20260819.html) | First real climb on the glass door: why a leg gets "dragged down" before lifting (press reaction bending the leg chains, not cup slip) |
| [vent-snap-20260820.html](vent-snap-20260820.html) | Stepping-in-place experiment quantified: 83% of the slip happens at the instant the vented seal ruptures; the vent-snap ratchet and the zero-force handover idea |
| [handover-delta-calib-20260824.html](handover-delta-calib-20260824.html) | δ calibration in three runs (δ = 0/12/24): bounce is linear in δ, extrapolated δ* = 28–42 mm per leg; bounce ≠ stored energy |
| [handover-ab-20260825.html](handover-ab-20260825.html) | Three-run A/B/C on the wall (baseline / handover / handover + weights): −42% and −61% three-round slip, replicate #1 |
| [handover-ab-n3-20260826.html](handover-ab-n3-20260826.html) | n=3 Latin-square main results: −43% / −61% total slip, −82% / −88% at the rupture instant, paired tests, deviation log |
| [stiffness-fit-20260826.html](stiffness-fit-20260826.html) | Parallel-spring model fit of the handover segment: ~90% of the residual is common-mode sinking, not stiffness inequality; gate decision on the stiffness-aware share |
| [dprime-20260831.html](dprime-20260831.html) | D′ discrimination experiment: handover-segment sinking is proportional to the amount apportioned, not to time (the κ law); weights cut κ by 44% |
| [pi-crash-ground-loop-20260904.html](pi-crash-ground-loop-20260904.html) | Case file: Raspberry Pi 5 dying at servo-relay closure, traced to a charging pulse entering through the USB ground loop |

## Design drawings and wiring (P2–P4)

| File | What it is |
|---|---|
| [fkp-glass-poses.html](fkp-glass-poses.html) | P2: the FKP leg triangle relative to the glass in the lift-off and press poses |
| [p2-rig-diagram.html](p2-rig-diagram.html) | P2: single-leg wall test rig geometry (side view, to scale) |
| [coxa-pedestal-mount.html](coxa-pedestal-mount.html) | P2: printed coxa pedestal / one-piece carriage for the rig, with the 2026-07-19 measured corrections |
| [theta-measure-diagram.html](theta-measure-diagram.html) | Single-leg calibration: which angle θ / α / γ is and how to measure it |
| [tibia-three-view.html](tibia-three-view.html) | Tibia printed part in three views: knee axis = servo output shaft, l3 = 123.7 mm (dispute closed) |
| [p4-system-diagram.html](p4-system-diagram.html) | P4: whole-robot electrical + pneumatic connection topology, relay wiring, valve state semantics |
| [p4-pi-wiring.html](p4-pi-wiring.html) | P4: Pi 5 pin-level wiring (40-pin header, USB, power routes) |
| [p4-divider-board.html](p4-divider-board.html) | P4: 7-channel divider board build drawings (schematic, perfboard layout, checklist) |
| [p4-pneumatic-electrical.html](p4-pneumatic-electrical.html) | P4: terminal-level wiring of the pneumatic subsystem (dual 12 V rails, MOSFET board, grounds, current budget) |
| [p4-bay-v1-assembly.html](p4-bay-v1-assembly.html) | P4: interactive 3D assembly view of the V1 equipment bay and whole robot |
| [tracking-targets-printable.html](tracking-targets-printable.html) | A4 print page of vision tracking targets (1:1) for the video pipeline |

## Explainers

| File | What it is |
|---|---|
| [gait-to-servo.html](gait-to-servo.html) | From a walk command to 18 servos: the full climb_walk chain (phase clock, IK, calibration, serial packet) |
| [servo-primer-20260821.html](servo-primer-20260821.html) | Servo primer: what is inside, how it reads an angle, how it misbehaves under load, what to buy |
| [knowledge-map-20260821.html](knowledge-map-20260821.html) | Knowledge map: the thirteen areas this project needed, what to learn first, six lessons not in the books |
| [dev-timeline-20260821.html](dev-timeline-20260821.html) | Development timeline 2026-07-21 to 08-21, compiled from the chat sessions |

The English pages were produced by extracting every Chinese text segment from the original (text nodes, attributes, JavaScript strings, source comments) and injecting translations back without touching markup, CSS, SVG geometry or embedded images. Some SVG labels are longer in English than in Chinese and may sit closer to neighboring elements than in the original.
