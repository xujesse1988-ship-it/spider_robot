> English translation of [`docs/weight-log.md`](../weight-log.md). The Chinese original is the maintained source; if they differ, the Chinese version wins.

# Weight log

> Hard upper limit of the wall-climbing budget: whole robot (including the adhesion system) **2.5kg**; if the whole robot is over 2.2kg at the end of P3, start a weight-reduction effort.
> Record every weighing. The trend matters more than any single point.

| Date | Stage | Item | Measured weight | Budget / reference | Notes |
|---|---|---|---|---|---|
| 2026-07-13 | P0 | Single leg (3 servos + printed parts) | 260g | ~230g | 30g over budget |
| | P0 | left-tibia-suction one-piece part + door cover | | ~45g | Replaces left-tibia (~13g) + the old suction foot (~15g); about +17g per leg (because the cavity wraps the full shape of the cup's metal fitting) |
| | P1 | Suction foot assembly (cup + metal fitting) | | ~40g | |
| | P1 | Pump + 2 valves + tubing (test rig) | | ~150g | |
| | P3 | Whole robot (no adhesion system) | | ≤1.9kg | Start weight reduction above 2.2kg |
| 2026-09-04 | P4 | Whole robot (with adhesion system) | **3537g** | ≤2.5kg | 1037g over the hard limit (+41%); includes the battery (confirmed by the user 09-04), i.e. the real hanging mass while climbing |

## 2026-09-04 conclusions (filled back into P4-GUIDE step 7)

- Whole robot (with adhesion system) measured **3537g**, 1037g heavier than the 2.5kg hard limit set in P0. But the real robot has already done every wall test since 08-19 at this weight (n=3 Latin square, D′ discrimination, long runs); the limit itself was a conservative P0 estimate.
- Adhesion math redone with the measured weight (P4-GUIDE step 0, item 2): load 3.537kg × 9.81 = **34.7N**. Measured shear of a single cup on vertical glass 15N → 75N with five feet in stance, safety factor **2.16** (it was 2.7 with the 2.8kg estimate); with one foot lost, four feet give 60N, safety factor **1.73** (the original "still >2" no longer holds).
- Open question: raise the budget limit to somewhere near the measured value, or start cutting weight. Before the P5 payload upgrade at least one of the two must happen: cut weight, or switch to 40mm cups (BOM alternative, normal force ×1.8).
