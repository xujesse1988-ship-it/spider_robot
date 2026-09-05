> English translation of [`docs/RELATED-WORK.md`](../RELATED-WORK.md). The Chinese original is the maintained source; if they differ, the Chinese version wins.

# Related Work deep dive (T5) — search report and the novelty boundary

Date: 2026-08-26. Method: six parallel search lines ((1) the classic band of force
redistribution in walking machines, (2) the suction-cup camp + Japanese literature,
(3) internal forces in multi-limbed climbing + the admittance neighborhood, (4) dry-adhesive
peeling, (5) cross-cutting: the elastic-energy-release phenomenon + feedforward compensation
methodology, (6) Chinese-language mine sweeping), ~350 search/fetch actions in total;
tools = WebSearch + arXiv + Semantic Scholar/OpenAlex/Crossref API + J-STAGE + Google Patents.
Known limits: the S2 search endpoint returned 429 across the board (OpenAlex/Crossref filled
in); IEEE/MDPI/PMC mostly 403, so the bulk of the verification is at abstract level with a
little at full-text level (the LORIS full-text PDF, the full text of the 2023 科学通报
(Chinese Science Bulletin) review, the full text of the 2023 机械工程学报 (Journal of
Mechanical Engineering) review, 6 PDFs from 机器人 (Robot), the full text of JSME 関東支部
(Kanto Branch) 19A15, and so on); CNKI/Wanfang/VIP full texts are behind the wall and were not
reached. **Items that must be checked against the full text by hand before citing are listed
in §7.**

---

## 0. Conclusions at a glance (the novelty boundary, contribution by contribution)

**Overall verdict: no direct collision.** No verified work satisfies two or more of
"adhesion-based wall climbing + quantified slip at the instant of detachment + open-loop
sensor-free pre-detachment unloading" at the same time — under the strictest reading the
Stickybot patent does cover two of them, "adhesive wall climbing + pre-detachment unloading",
but it is force-sensor closed loop + dry adhesive + no slip quantification, which makes it a
same-idea near neighbor that has to be demarcated head-on, not a collision.

- **Contribution 1 (phenomenon + quantification) — safe, a first can be claimed** (with "to
  the best of our knowledge"): the first time-resolved quantification of "release of stored
  elastic energy at the instant of vacuum seal rupture → ratchet-like whole-robot position
  loss" (83–93% concentrated in the <100ms after rupture; per cycle, per leg, at mm scale).
  Three qualifiers lock it down: **the instant of vacuum seal rupture / whole-robot position
  loss (mm, per cycle) / (the remedy) open loop and sensor-free**. Every precedent is either
  qualitative or on a different basis: Stickybot T-RO 2008 records qualitatively that "the
  detachment-force transient propagates through the body and makes the remaining feet detach
  prematurely"; the 2023 科学通报 (Chinese Science Bulletin) review (Dai group) names
  "transient forces and large detachment forces transmitted to the trunk producing trunk
  shudder" and lists "reducing the detachment impact" as an open direction; the dry-adhesive
  camp quantifies foot force (N) and body vibration (m/s²), the suction-cup camp quantifies
  cavity pressure (潘雷 (Pan Lei) 2005) — nobody has quantified whole-robot displacement along
  the wall.
- **Contribution 2 (model) — safe**: the conceptual lineage of internal forces has to cite
  Kumar & Waldron 1988 (internal-force decomposition in closed chains) and the DLR Crawler
  (the proposition that "a position-controlled legged machine accumulates internal forces");
  but "a displacement-domain parallel-spring model of the elastic internal forces during
  handover + stiffness-weighted equilibrium + several verified predictions" has no precedent.
- **Contribution 3 (remedy) — the wording has to be narrowed**: ❌ do not write "the first to
  propose pre-detachment unloading" — the Stickybot patent US7762362B2 states outright
  "unload tangentially before lift-off, relax the accumulated force and elastic deformation,
  prevent the transient force from disturbing the other feet" (force sensors + stiffness
  controller, closed loop); LORIS (ICRA 2024) has an explicit Unload step in its gait state
  machine (online force optimization constrains the contact force of the gripper about to be
  released to zero). ✅ what can be claimed: **the first conversion of the pre-detachment
  unloading condition from the force domain to the displacement domain (per-leg offline
  calibration of the returned δ + mean-invariant apportionment across the six leg commands),
  realized purely open loop on a position-servo platform with no force, tactile or current
  feedback whatsoever, and closed out by quantified rupture-segment slip (−82%/−88%)**. None
  of the six lines found a precedent for "an offline calibration table + a purely kinematic
  pre-displacement that zeros multi-contact internal forces".
- **Warning about a coincidence of units**: FTFOF (arXiv 2504.19448) reports "vibration
  amplitude −82%" (m/s²), which happens to be the same number as our group B rupture-segment
  −82% (mm) but in different units; the text has to call this out.

**Top-6 nearest neighbors (demarcate each one head-on when writing)**: the Stickybot patent +
T-RO 2008, LORIS 2024, Ota 2006 (the only precedent for deformation compensation on an
adhesion platform, but it compensates static gravity deformation), TITAN XI 2005 (same means —
displacement-domain calibration plus feedforward — but the target is foot placement accuracy),
Chen 1999 ("force-sensor-free" refers only to computing the setpoints), and the
Wang21/Wen25/FTFOF dry-adhesive detachment cluster.

---

## 1. Group A: detachment management in climbing robots (the direct neighbors of both the phenomenon and the remedy)

- 🔴 Cutkosky & Kim (filed 2007 / granted 2010) "Climbing with dry adhesives" — US patent US7762362B2 — https://patents.google.com/patent/US7762362B2/en — states outright that "before a foot is lifted it must be unloaded tangentially to relax the accumulated force and elastic deformation and prevent transient forces", control law x_cmd=x_ff+(kP+kI/s)·C·(f_s−f_d) — the earliest formal statement of the same idea; demarcation = Hall force-sensor closed loop vs. zero-sensor open loop, dry adhesive vs. vacuum seal rupture, no slip quantification vs. slip as the primary metric.
- 🔴 Kim, Spenko, Trujillo, Heyneman, Santos & Cutkosky (2008) "Smooth Vertical Surface Climbing With Directional Adhesion" — IEEE T-RO 24(1):65–74 — DOI 10.1109/TRO.2007.909786 — Stickybot, directional adhesion + tangential force control on glass, records that in the isotropic-pad era "the large detachment-force transient propagated through the body and made the other feet detach/slip prematurely" — a qualitative precedent for the phenomenon plus a material-level remedy (near-zero-force detachment through directional adhesion); no position-loss quantification. **Original checked (09-01; the authors' public MIT version is stored at paper/references/kim2008smooth.pdf)**: verbatim quote "This large detachment force was the main limitation of the isotropic patches, producing oscillations that frequently caused the other feet to slip"; also "The unloading step for the anisotropic patches is accomplished rapidly and results in negligible detachment force" — the Related Work in paper/main.tex now quotes the original sentences (it previously put quotation marks around a paraphrase of a search abstract = a misquotation risk, now gone). Sister papers: Kim et al. ICRA 2007 "Whole body adhesion" (best student paper, the origin of the force control) and Santos et al. ICRA 2007 (directional adhesive structures; the quote "isotropic pads need a large pull-off force" comes from a search abstract and has not been checked verbatim against the original (?)).
- 🔴 Nadan, Backus & Johnson (2024) "LORIS: A Lightweight Free-Climbing Robot for Extreme Terrain Exploration" — ICRA 2024 — https://www.ri.cmu.edu/app/uploads/2024/04/LORIS__A_Lightweight_Free_Climbing_Robot_for_Extreme_Terrain_Exploration.pdf (full text checked) — a quadruped with passive microspine grippers whose gait state machine has an explicit Unload step: "constrain the contact force on the gripper to be zero in the force optimization and wait for the contact force to converge" — the same target state as the zero-force handover; demarcation = proprioception + online force optimization in closed loop vs. an offline δ table open loop, microspines on rock vs. vacuum suction cups, no slip quantification.
- 🔴 Boscariol, Henrey, Li & Menon (2013) "Optimal gait for bioinspired climbing robots using dry adhesion: A quasi-static investigation" — J. Bionic Engineering 10(1) — DOI 10.1016/S1672-6529(13)60193-6 — points out that preloading or detaching one foot redistributes the contact forces over the whole robot and that a bad distribution makes the whole robot fall off; a quasi-static model optimizes the gait offline — covers two elements, "adhesive wall climbing + pre-detachment load management"; demarcation = rigid-body contact-force planning (in simulation) vs. displacement-domain manipulation of the elastic internal forces in the leg chains (on real hardware), with no δ calibration and no slip quantification.
- 🟠 Wang, Xiong, Duan, Wang & Dai (2021) "Compliant Detachment of Wall-Climbing Robot Unaffected by Adhesion State" — Applied Sciences 11(13):5860 — DOI 10.3390/app11135860 — online impedance control drives the peel angle to π, "essentially eliminating the sudden jump in normal adhesion force at the end of detachment" — the same awareness of the "jump at the instant of detachment" problem; dry adhesive, closed-loop impedance, foot-force basis.
- 🟠 Wen, Zheng, Jing, Guo & Chen (2025) "Force–Position Coordinated Compliance Control in the Adhesion/Detachment Process of Space Climbing Robot" — Aerospace 12(1):20 — DOI 10.3390/aerospace12010020 — real-time foot-force feedback + rotational peeling, pull-off force −65.5%, "reducing the disturbance to the body and the supporting legs" — the paper whose statement of the problem comes closest to ours; force-feedback closed loop, microgravity suspension, no body-slip quantification.
- 🟠 Xiao, Nie, Hao & Li (2025) "An End-to-End Framework for Optimizing Foot Trajectory and Force in Dry Adhesion Legged Wall-Climbing Robots" (FTFOF) — arXiv:2504.19448 — a three-segment C² Bezier foot trajectory + GRU force prediction in a multi-objective optimization, peak detachment force 7.49→5.40N (−28%), IMU vibration 327→58 m/s² (−82%) — it only shapes the trajectory of the foot being detached, does not redistribute internal forces inside the support set, and depends on force measurements for training; open-/closed-loop execution details unchecked (§7); none of its 21 references quantify detachment slip, and its single citing paper is unrelated. ⚠ the −82% coincidence of units.
- 🟠 Imai, Uno & Yoshida (2024) "Admittance Control-based Floating Base Reaction Mitigation for Limbed Climbing Robots" — CLAWAR 2024 — arXiv:2409.13218 — a force sensor on every gripper drives admittance control to suppress disturbance reaction forces and prevent **accidental** detachment; ClimbLab simulation only (HubRobo model) — the 08-24 judgment that this is the "closest benchmark" holds and the demarcation is even cleaner: closed-loop F/T vs. open loop, preventing accidents vs. the instant of deliberate venting, simulation vs. real hardware with n=3, no slip quantification; its 21 refs + 4 citations (all skimmed) contain no pre-detachment internal-force redistribution.
- 🟠 Ribeiro, Uno, Imai, Murase & Yoshida (2023) "RAMP: Reaction-Aware Motion Planning of Multi-Legged Robots for Locomotion in Microgravity" — ICRA 2023 — arXiv:2301.07996 — feedforward planning of low-reaction swing trajectories to prevent accidental gripper detachment (air-bearing experiments) — likewise feedforward with no force feedback, but it manages swing inertial reaction forces, not statically stored elastic internal forces.
- 🟠 Murphy, Kute, Mengüç & Sitti (2011) "Waalbot II: Adhesion Recovery and Improved Performance of a Climbing Robot using Fibrillar Adhesives" — IJRR 30(1) — DOI 10.1177/0278364910382862 — a magnetically latched ankle + a slotted-lever passive peeling mechanism peels at low force through uneven loading — mechanism-level, sensor-free minimization of the peel force; no quantification of the disturbance to body pose.
- 🟠 Parness et al. (2017) "LEMUR 3: A limbed climbing robot for extreme terrain mobility in space" — ICRA 2017 — DOI 10.1109/ICRA.2017.7989643 — a single-axis force sensor at each limb tip manages attachment/detachment forces — the representative platform for sensor-closed-loop detachment-force management.
- 🟠 Yoshida & Ma (2010) "Design of a wall-climbing robot with passive suction cups" — IEEE ROBIO 2010 — DOI 10.1109/ROBIO.2010.5723554 — passive suction cups detach at low force by pulling the rim to peel it — the "mechanical pressure relief" idea of the passive-cup camp (a mechanical version of the same spirit as "slow venting"); no slip quantification.
- 🟡 Wang, Wang, Song, Zong, Zhang, Ji, Manoonpong & Dai (2023 (?)) "A Neural Coordination Strategy for Attachment and Detachment of a Climbing Robot Inspired by Gecko Locomotion" — Cyborg and Bionic Systems 4:0008 — DOI 10.34133/cbsystems.0008 — a CPG + lagged delay lines coordinate the attach/release timing of a hybrid pneumatic-electric drive (rotational detachment lowers the pull-off force) — the coordination/structure route to smoothing detachment; sources disagree on the year, 2022 or 2023 (§7).
- 🟡 Xiao et al. (2024) "MST-G: Micro Suction Tape Gripper Climbing Robot with Active Detachment Capability" — Sensors 24(23):7790 — DOI 10.3390/s24237790 — micro-suction tape + a linear motor for mechanism-level active detachment — here "active detachment" is an enabling mechanism, not disturbance mitigation; an earlier work from the same group as FTFOF.
- 🟡 Spenko, Haynes et al. (2008) "Biologically inspired climbing with a hexapedal robot" (RiSE) — J. Field Robotics — DOI 10.1002/rob.20238 — a feed-forward cyclic gait + attachment/detachment trajectory shaping tuned per surface — a precedent for open-loop trajectory shaping (local to the foot), with no internal-force accounting and no slip metric; whether it quantifies per-cycle slip has not been checked page by page (?).
- 🟡 Shao, Chen, Zang & Li (2026) "Slip-Adaptive Neural Control of Gecko-Inspired Adhesive Robots" — Advanced Intelligent Systems — DOI 10.1002/aisy.202501168 — joint-torque proprioception estimates the adhesion force and detects/recovers from slip — the basis is slip from adhesion failure during stance, not the instant of release; a sensing-closed-loop counterpoint.

## 2. Group B: the classic band of force redistribution in walking machines → modern WBC (the chain of evidence that execution happens in the force domain)

Core conclusion: since 1986, "smoothly unloading a leg's force to zero before lifting it" has
been a classic walking-machine topic, but the whole band, right through to modern WBC/MPC,
all of it **executes in the force domain** (force/torque sensors in closed loop, or the force
setpoint handed to a drive layer that can track force, such as hydraulics or torque motors);
there is no precedent for "position servos + an offline calibrated displacement table zeroing
internal forces purely open loop".

- 🟠 Waldron (1986) "Force and motion management in legged locomotion" — IEEE J. Robotics and Automation RA-2(4):214–220 — the foundation of force/motion coordination in walking machines (ASV background) — the source citation for the classic band.
- 🟠 Kumar & Waldron (1988) "Force distribution in closed kinematic chains" — IEEE J. Robotics and Automation 4(6):657–664 — multiple supporting legs = a closed kinematic chain, decomposed into equilibrating forces + interaction (internal) forces — the ancestor of this paper's "internal force" concept; their internal force is a rigid-body statically indeterminate variable, ours comes from measured compliance and is manipulated in the displacement domain.
- 🟠 Kumar & Waldron (1990) "Force distribution in walking vehicles" — ASME J. Mechanical Design 112(1):90–99 (?) — force distribution applied to walking-vehicle gaits (including leg lift-off and touchdown events) — ⚠ the full text is behind a 403, so whether it states outright that "the force is exactly zero at the moment of lift-off" is unchecked (§7); do not claim it on their behalf when citing.
- 🟠 Klein & Kittivatcharapong (1990) "Optimal force distribution for the legs of a walking machine with friction cone constraints" — IEEE T-RA 6(1):73–85 — optimal force distribution under friction-cone constraints — a must-cite in the classic band; entirely in the force domain.
- 🟠 Gorinevsky & Shneider (1990) "Force Control in Locomotion of Legged Vehicles over Rigid and Soft Surfaces" — IJRR 9(2):4–23 — force feedback bolted onto a purely position-controlled hexapod to achieve force-controlled walking — direct evidence that "the classic solution for managing contact forces on a position-controlled legged machine = add force sensors in closed loop", which sets off our route by contrast.
- 🟠 Chen, Cheng, Yang, Kung & Sun (1999) "Optimal force distribution in multilegged vehicles" — Robotica 17(2):159–172 — Compact-QP force distribution + a smoothing segment across the stance↔swing transition that removes the discontinuity in the commanded force; the abstract says explicitly that the smoothing scheme "does not require force sensors" — ⚠ a wording trap: what is sensor-free is only the **setpoint computation**, and execution still needs a force-tracking drive layer; rigid-body framework, no elastic storage, no adhesion.
- 🟠 Zhou, Low & Zielinska (2000) "An efficient foot-force distribution algorithm for quadruped walking robots" — Robotica 18(4):403–413 — the FriCoM method, with force distribution continuous throughout a crawl gait — the representative of the "force-transition continuity" demand.
- 🟠 Erden & Leblebicioğlu (2007) "Torque Distribution in a Six-Legged Robot" — IEEE T-RO 23(1):179–186 — QP distribution in the torque domain over a hexapod's supporting legs — the modern hexapod classic; fully actuated torque execution.
- 🟠 Görner & Hirzinger et al. (2009) "The DLR Crawler: evaluation of gaits and control of an actively compliant six-legged walking robot" — Industrial Robot — DOI 10.1108/01439910910957101 (volume/issue unchecked (?); sister paper at IROS 2008) — joint torque sensors + compliant control, stating outright that "compliant control reduces internal forces compared with pure joint position control" — this spells out exactly the proposition this paper has to demarcate against: they kill internal forces with torque sensors, we kill them open loop with a calibration table.
- 🟡 Cheng & Orin (1990) "Efficient algorithm for optimal force distribution—the compact-dual LP method" — IEEE T-RA 6(2):178–187 — an efficient LP solution (the solution can jump over time, which is exactly what the "continuity" works target) — background.
- 🟡 Marhefka & Orin (1998) "Quadratic optimization of force distribution in walking machines" — IEEE ICRA 1998:477–483 — QP force distribution minimizing motor power — background.
- 🟡 Gardner (1991/1992) "Force distribution in walking machines over rough terrain" and others — ASME J. Dyn. Sys. Meas. Control 113(4):754– / Robotica 1992 (volume/issue unchecked (?)) — efficient force distribution for arbitrary contact normals on uneven ground — completes the classic band.
- 🟡 Song & Waldron (1989) "Machines That Walk: The Adaptive Suspension Vehicle" — MIT Press monograph — the systematic monograph on the OSU ASV hexapod — general background; the hydraulic execution details have not been verified from a primary source (§7).
- 🟡 Sentis, Park & Khatib (2010) "Compliant Control of Multicontact and Center-of-Mass Behaviors in Humanoid Robots" — IEEE T-RO — virtual linkage explicitly models and controls internal tensions in multi-contact — one of the sources of the internal-force regulation concept; whole-body force control.
- 🟡 Righetti, Buchli, Mistry, Kalakrishnan & Schaal (2013) "Optimal distribution of contact forces with inverse-dynamics control" — IJRR 32(3):280–298 — DOI 10.1177/0278364912469821 — analytic optimization of contact and internal forces through torque redundancy — the representative theory of modern force-domain internal-force management.
- 🟡 Focchi, del Prete, Havoutis et al. (2017) "High-slope terrain locomotion for torque-controlled quadruped robots" — Autonomous Robots 41:259–272 — DOI 10.1007/s10514-016-9573-1 — optimal distribution of support forces on a 50° V-shaped slope to prevent slipping — the force-domain distribution work whose steep slope comes closest to wall conditions.
- 🟡 Bellicoso, Gehring, Hwangbo, Fankhauser & Hutter (2016) "Perception-less terrain adaptation through whole body control and hierarchical optimization" — IEEE-RAS Humanoids 2016:558–564 — ANYmal, hierarchical QP, smooth support transitions — the representative of contact transitions in modern WBC.
- 🟡 Di Carlo, Wensing, Katz, Bledt & Kim (2018) "Dynamic Locomotion in the MIT Cheetah 3 Through Convex Model-Predictive Control" — IROS 2018:7440–7447 — DOI 10.1109/IROS.2018.8594448 — convex MPC plans the ground-reaction-force sequence (the force timing at the start and end of a contact phase) — the modern representative of "contact forces ramp according to plan"; a torque-controlled platform.
- 🟡 Bretl (2006) "Motion Planning of Multi-Limbed Robots Subject to Equilibrium Constraints: The Free-Climbing Robot Problem" — IJRR 25(4):317–342 — the progenitor of planning for multi-limbed free climbing under equilibrium constraints — planning-layer background; does not touch detachment transients.
- 🟡 Shirai, Lin, Schperberg, Tanaka, Kato, Vichathorn & Hong (2022) "Simultaneous Contact-Rich Grasping and Locomotion via Distributed Optimization…" — IROS 2022 — arXiv:2207.01418 — ADMM plans grasping and locomotion together, on real hardware at 45° — planning-layer contact-force distribution.
- 🟡 Caccavale et al. (2008 (?)) "Six-DOF Impedance Control of Dual-Arm Cooperative Manipulators" — IEEE/ASME TMECH — DOI 10.1109/TMECH.2008.2002816 — internal-force impedance in dual-arm coordination — the source of the internal-force conceptual lineage in 2409.13218; the author order has not been checked one by one (?).
- 🟡 Dallmann et al. (2017) "A load-based mechanism for inter-leg coordination in insects" — Proc. Royal Society B 284(1868):20171755 — https://royalsocietypublishing.org/rspb/article/284/1868/20171755 — in stick insects, loading a neighboring leg mechanically unloads the leg itself, and the unloading signal (campaniform sensilla) triggers swing — the biological version of "unload first, then lift" (via a load-sensing reflex); it gives the zero-force handover a neat biological motivation and sets off our sensor-free open loop by contrast.
- 🟡 Kurazume, Yoneda & Hirose (2001/2002 (?)) "Feedforward and feedback dynamic trot gait control for quadruped walking vehicle" — ICRA 2001 / Autonomous Robots — DOI 10.1023/A:1014045326702 — sway compensation: shifting the body/ZMP in advance in the position domain to transfer the support load — the demarcation sentence: "pre-laying displacement to transfer the **gravity load** is a static-gait tradition; pre-laying displacement to zero the **elastic internal forces** in an over-constrained adhesion system is a new proposition".

## 3. Group C: displacement-domain calibrated feedforward compensating for compliance (same method, different target)

- 🟠 Doi, Hodoshima, Hirose, Fukuda, Okamoto & Mori (2005) "Development of a quadruped walking robot to work on steep slopes, TITAN XI (walking motion with compensation for compliance)" — IROS 2005 — DOI 10.1109/IROS.2005.1545498 — a 7-ton hydraulic quadruped for steep slopes: the compliance of the leg system is calibrated by hand in advance and the bending deformation is compensated feedforward during walking to restore foot placement accuracy — **the nearest precedent for displacement domain + offline calibration + feedforward**; the target is position accuracy, not zeroing internal forces, and there is no adhesion and no detachment; must be cited and demarcated.
- 🟠 Ota, Kuga & Yoneda (2006) "Deformation compensation for continuous force control of a wall climbing quadruped with reduced-DOF" — ICRA 2006:468–474 — DOI 10.1109/ROBOT.2006.1641755 — a reduced-DOF blower-adhesion quadruped whose "compensating motion" corrects the body bending under gravity so that the motion stays smooth (abstract level, full text not obtained, §7) — **the only precedent for deformation compensation on an adhesion platform**; it compensates static gravity deformation and does not touch detachment transients, zeroing internal forces, or vent timing; must be cited and demarcated precisely.
- 🟠 Wang, Zhang & Fuhlbrigge (2009) "Improving Machining Accuracy with Robot Deformation Compensation" — IROS 2009 — DOI 10.1109/IROS.2009.5353988 — a joint stiffness model + feedforward correction of the commanded trajectory to compensate the deflection under machining forces — the most classic methodologically isomorphic precedent (offline stiffness model → pure position feedforward); a continuous machining force, not internal forces at a contact switch.
- 🟠 Klimchik, Bondarenko, Pashkevich, Briot & Furet (2014) "Compliance error compensation in robotic-based milling" — LNEE — arXiv:1409.6231 — a nonlinear stiffness model + offline trajectory modification — the same family; it gives "calibration + feedforward compensation of compliance" a mature lineage.
- 🟠 Honda patent, Takenaka et al. (2005) "Floor shape estimation system of legged mobile robot" — US6922609/US6920374 — https://patents.google.com/patent/US6922609 — deformation compensation on a legged robot: feedforward cancellation of the elastic deformation of the compliance mechanism and the foot sole — the direct precedent for feedforward deflection compensation on a legged platform; the deformation comes from a model + the desired ground reaction force (supplied by a closed-loop stabilizer), and it is a patent, not a paper.
- 🟠 Wang, Weng, Wang, Wang, Wang, Dai & Jusufi (2024) "Wall-Climbing Performance of Gecko-inspired Robot with Soft Feet and Digits enhanced by Gravity Compensation" — Bioinspir. Biomim. — DOI 10.1088/1748-3190/ad5899 / arXiv:2405.02639 — a leg stiffness model + feedforward gravity compensation (QP foot forces → commanded positions), success rate 3/10→10/10 — the precedent for "stiffness model + feedforward position correction" on a wall-climbing platform; the target is body posture and adhesion angle, and it does not touch detachment unloading or slip.

## 4. Group D: platform lineages and alternative routes (the background citation pool)

**The vacuum / suction-cup camp** (conclusion: nobody quantifies body slip at the instant of
release, and there is no precedent for vent-timing compensation; the countermeasures fall into
only two kinds, "never detach" and "pressure closed-loop redundancy"):
- 🟡 Hirose, Nagakubo & Toyama (1991) "Machine that can walk and climb on floors, walls and ceilings" (NINJA-I) — ICAR 1991 — ieeexplore 240585 — parallel-linkage legs + multiple VM-valve suction cups — the progenitor of legged vacuum climbing; the valves serve "keeping the adhesion alive", and nothing on detachment transients was found (full text not obtained (?)).
- 🟡 Hirose & Kawabe (1998) "Ceiling walk of quadruped wall climbing robot NINJA-II" — CLAWAR 1998 (no DOI) — ceiling walking — full text unavailable (?).
- 🟡 Nishi (1988/1992), the suction-cup biped wall-climbing series — ISARC 1988 (DOI 10.22260/isarc1988/0065) / Mechatronics 2(6) — a classic platform, no quantification of detachment transients.
- 🟡 Zhang, Zhang, Zong, Wang & Liu (2006) "Sky Cleaner 3" — IEEE RAM 13(1) — DOI 10.1109/MRA.2006.1598051 — pneumatic cylinders + alternating suction-cup groups for window cleaning — no discussion of release slip found at abstract level (?).
- 🟡 Xiao & Sadegh (2007) "City-Climber" — InTech — DOI 10.5772/5090 — a wheeled rotor-driven vacuum design with an imperfect seal — the route that dodges the rupture problem by "not needing a perfect seal".
- 🟡 Hillenbrand, Schmidt & Berns (2008) "CROMSCI" — Industrial Robot 35(3) — DOI 10.1108/01439910810868552 — seven controllable vacuum chambers + a pressure closed loop against leakage transients — the representative "sensors + valve timing" countermeasure; its object is continuous leakage, not gait detachment.
- 🟡 Longo & Muscato (2004) "Alicia3" — Industrial Robot 31(2) — DOI 10.1108/01439910410522838 — three sliding-suction modules — the representative of sliding vacuum adhesion.
- 🟡 Miyake & Ishihara (2006/2009), the WallWalker series — ISARC 2006 / IROS 2009 (ieeexplore 4913279) — sliding suction cups + liquid-seal lubrication — the Japanese representative of sliding suction.
- 🟡 Yue, Bloomfield-Gadêlha & Rossiter (2024) "Snail-inspired water-enhanced soft sliding suction for climbing robots" — Nature Communications 15 — DOI 10.1038/s41467-024-48293-2 — water-lubricated sliding suction that can slide without ever releasing the seal; **its introduction explicitly lists the drawback of discrete gaits, "repeatedly destroying and rebuilding the adhesion"** — cited for problem motivation and as the contrast case for the avoidance route (it needs continuous power and water).
- 🟡 Kim, Kim, Yang, Lee et al. (2008) "Development of a wall-climbing robot using a tracked wheel mechanism" — J. Mech. Sci. Tech. 22 — DOI 10.1007/s12206-008-0413-x — 24 suction cups on a track chain, with mechanical valves attaching and releasing in position order — the hardware precedent for "mechanically valved vent timing" (a continuous track, no slip quantification).
- 🟡 福田敏男 (Fukuda Toshio) et al. (1992) 「吸盤装着クローラ形壁面走行ロボット」 — 日本機械学会論文集 C 編 (Transactions of the JSME, Series C) (volume/issue unchecked (?)) — a tracked design with ducts and mechanical valves that open and close automatically — the early Japanese representative.
- 🟡 Wang, Wang, Zong & Li (2010) "Principle and experiment of vibrating suction method" — Vacuum 85(1) — DOI 10.1016/j.vacuum.2010.04.010 — vibrating suction continuously regenerates the vacuum — the adhesion-maintenance route.
- 🟡 Shi, Xu, Xu & Jiang (2022), a 6-DOF humanoid vacuum-adhesion biped — Mechatronics 87 — DOI 10.1016/j.mechatronics.2022.102889 — a recent footed vacuum platform (abstract unavailable (?)).
- 🟡 Wang, Bao, Zhang & Yang (2009), a compliant pneumatic wall climber — J. Cent. South Univ. Tech. 16 — DOI 10.1007/s11771-009-0160-x — the same class of platform, "highly compliant leg chains + suction cups"; no detachment handling found (?).
- 🟡 潘雷 (Pan Lei), 赵言正 (Zhao Yanzheng), 钱志源 (Qian Zhiyuan), 付庄 (Fu Zhuang), 曹其新 (Cao Qixin) (2005) "Adhesion Characteristics of a Wall-Climbing Robot with Double Negative-Pressure Suckers" — 上海交通大学学报 (Journal of Shanghai Jiaotong University) 39(6):873–876 — a fluid-network model gives the **cavity-pressure dynamic response** under three conditions (fan sudden start, obstacle, sudden stop) and verifies it experimentally — the Chinese-language work that comes closest to a quantitative treatment of the "release/failure transient"; its basis is cavity pressure, not body displacement.
- 🟡 管贻生 (Guan Yisheng) group (2010/2013) "W-Climbot: A modular biped wall-climbing robot" — IROS 2010 (ieeexplore 5589064) / IEEE-ASME TMECH 2013 — 5 joint modules + vacuum at both ends, an inchworm biped — the classic Chinese series closest in form to this work; no quantification of detachment slip (full text not checked page by page (?)).
- 🟡 Li, Zhang, Huang…Xu (2025) "Development of a New Biped Robot With Adaptive Suction Modules for Curved-Surface Climbing" — IEEE RA-L (ieeexplore 11027566; the same group's T-ASE 2025, DOI 10.1109/TASE.2024.3390030) — a University of Macau biped that climbs curved surfaces by vacuum adhesion — the most recent platform in the same journal using the same adhesion method; worth citing.
- 🟡 王斌锐 (Wang Binrui) et al. (2014) "Design and stability analysis of a biped three-DOF wall-climbing robot on curved surfaces" — 机器人 (Robot) 36(3) — DOI 10.3724/SP.J.1218.2014.00349 — **static** constraints against tipping over and sliding off — representative of the prevailing Chinese-language basis "sliding off = a static instability criterion", which by contrast highlights the gap that this paper's per-cycle dynamic displacement basis fills (the same group's 2018 paper in 智能系统学报 (CAAI Transactions on Intelligent Systems) gave the same (?) verification conclusion).
- 🟡 吉田壱平 (Yoshida Ippei) et al. (2023) 「壁面移動を目的とした吸盤脚型ロボット『GeckoPus』の開発」 — ROBOMECH 2023 — DOI 10.1299/jsmermd.2023.2P2-G13 — a suction-cup-footed quadruped — a recent Japanese work; nothing on detachment at abstract level. Also: Kawasaki & Kikuchi (2014), a passive-suction-cup hexapod (venue unchecked (?)) / 清水・菊池 (Shimizu & Kikuchi) (2019) JSME 関東支部 (Kanto Branch) 19A15 (full text read; their "slip" = kinematic slip while turning); Yamasaki et al. (2015) ROBOMECH (DOI 10.1299/jsmermd.2015._2a2-o10_1).
**Dry-adhesive / microspine / gripper platforms**:
- 🟡 Unver, Uneri, Aydemir & Sitti (2006) "Geckobot" — ICRA 2006 — DOI 10.1109/ROBOT.2006.1642050 — an elastomer-adhesion quadruped + a peeling mechanism + an active tail — its motivation section contains the earliest one-sentence comparison of "suction cups need a large detachment force and are inefficient vs. low-force peeling of elastomers" (the quote comes from a search abstract (?)).
- 🟡 Henrey et al. (2014) "Abigaille-III" — J. Bionic Engineering — a hexapod with two-level dry adhesive — the same morphology with different physics, for contrast.
- 🟡 Birkmeyer, Gillies & Fearing (2012) "Dynamic climbing of near-vertical smooth surfaces" (CLASH) — IROS 2012 (ieeexplore 6385775) — a remote-center ankle reduces the peel torque, dynamic climbing of 70° at 4–12Hz — mechanism plus stride frequency leave detachment "no time" to disturb anything.
- 🟡 Provancher, Jensen-Segal & Fehlberg (2010) "ROCR" — IEEE/ASME TMECH (ieeexplore 5546974) — a pendulum with two claws climbing a carpeted wall dynamically — zero-disturbance claw detachment is a free lunch from the physics of hooks (for contrast: the problem simply does not exist in the claw line).
- 🟡 Yu et al. (2024), a variable-stiffness adhesive gripper (microgravity) — Advanced Intelligent Systems — DOI 10.1002/aisy.202400043 — material- and mechanism-level minimization of the detachment force (abstract level (?)).
- 🟡 Uno et al. (2021) "HubRobo" — IEEE-RAS Humanoids — DOI 10.1109/HUMANOIDS47582.2021.9555799 — a quadruped with passive spine grippers — platform background.
- 🟡 Tanaka et al. (2022/2023) "SCALER" — IROS 2022 / journal version — arXiv:2207.01180 / 2312.04856 — climbing with GOAT underactuated grippers — platform background. Checked and not a conflict: Uno et al. (2026) LIMBERO (arXiv:2603.16531; the abstract has nothing on detachment or internal forces).
**Reviews**:
- 🟡 Nansai & Mohan (2016) — Robotics 5(3) — DOI 10.3390/robotics5030014; Tao, Gong & Ding (2023) "Climbing robots for manufacturing" — NSR 10(5) — DOI 10.1093/nsr/nwad042 (no dedicated section on detachment slip); 马吉良 (Ma Jiliang) et al. (2023) 机械工程学报 (Journal of Mechanical Engineering) 59(5) — DOI 10.3901/JME.2023.05.011 (full text checked: nothing); **裴香丽 (Pei Xiangli)…戴振东 (Dai Zhendong) (2023) review in 科学通报 (Chinese Science Bulletin) (full text checked: it names "transient forces… trunk shudder" citing Stickybot and lists "reducing the detachment impact" as an open problem — valuable both as a motivation citation and as negative evidence on the Chinese-language side)**; the MDPI Robotics 11(6):143 (2022) gecko review (contains a qualitative comparison of the large detachment force of suction cups); the Electronics 14(14):2810 (2025) review.

## 5. Group E: contrasting physics and sensing alternatives

- 🟠 Tang, Chi, Sun et al. (2020) "Leveraging elastic instabilities for amplified performance: spine-inspired high-speed and high-force soft robots" — Science Advances 6:eaaz6912 — arXiv:1810.08571 — snap-through bistability stores and releases elastic energy in tens of milliseconds to drive crawling — the flagship contrast, **the same physics in the opposite direction**: they exploit the instantaneous release for propulsion, we identify it as a parasitic disturbance and bleed it off in advance. Companion: the bistable jumper (2024, Science Robotics, DOI 10.1126/scirobotics.adm8484, energy released for take-off in <15ms, authors unchecked (?)).
- 🟠 Wahrburg, Bös, Listmann, Dai, Matthias & Ding (2018) "Motor-Current-Based Estimation of Cartesian Contact Forces and Torques…" — IEEE T-ASE (ieeexplore 7914641) — a Kalman filter over current plus kinematics estimates the contact force — the representative alternative route of "estimating force without a force sensor"; it depends on an accurate friction/dynamics model and is not viable on cheap servos with high-friction leg chains — cited as motivation for this paper's open-loop route (it answers "why not close the loop on current").
- 🟠 (authors unchecked (?)) (2025), a continuum model of crawling robots — J. Mech. Phys. Solids (sciencedirect S0022509625000109) — models and predicts the per-cycle backward slippage caused by dry friction and the resulting speed loss — "per-cycle position loss" has already been named and quantified in the crawling lineage; the mechanism there is insufficient friction anchoring, not the instantaneous release of stored elastic energy.
- 🟡 Rafsanjani et al. (2018) "Kirigami skins make a simple soft actuator crawl" — Science Robotics 3:eaar7555 — anisotropic friction gives a "ratchet" that crawls in one direction — a terminological counterpoint, where "ratchet" is used in the positive sense.
- 🟡 Endlein & Federle (2013) "Rapid preflexes in smooth adhesive pads of insects prevent sudden detachment" — Proc. Royal Society B — DOI 10.1098/rspb.2012.2868 — insect adhesive pads passively expand the contact within 1ms to prevent accidental sudden detachment — biology has a dedicated passive mechanism even against "accidental snap detachment", which by contrast shows that deliberate rupture-type detachment has to be managed by engineering.
- 🟡 Tian, Pesika, Zeng…Autumn, Israelachvili (2006) "Adhesion and friction in gecko toe attachment and detachment" — PNAS 103(51) — DOI 10.1073/pnas.0608841103; Autumn et al. (2006) "Frictional adhesion: a new angle on gecko attachment" — J. Exp. Biology 209:3569 — the mechanics of the gecko peel angle, fast low-force detachment in ~20ms — evolution solved "fast, low-disturbance detachment"; dry-adhesive robots inherit this physics, **vacuum suction cups cannot — the only option is to return the energy before rupture** (a good sentence for the discussion section).
- 🟡 (authors unchecked (?)) (2023) "Modeling multi-legged robot locomotion with slipping and its experimental validation" — arXiv:2310.20669 — modeling slip in legged locomotion on the ground — background.
- 🟡 Pratt & Williamson (1995) "Series Elastic Actuators" — IROS 1995 — general background on elastic energy storage; nothing in the SEA lineage specifically quantifies "the disturbance from stored energy released at the instant contact breaks" (probably nonexistent; the prosthetics/exoskeleton branch was not swept).

## 6. Skeleton for writing the Related Work section (paragraph level)

1. **Platform lineages in one sentence**: adhesion splits into three lines — vacuum/negative
   pressure, dry adhesive, and gripper/microspine (cite 2–3 representatives each + the
   Nansai16/Tao23 reviews); this work belongs to the legged vacuum line (NINJA →
   Sky Cleaner → W-Climbot → the University of Macau RA-L25).
2. **What is already known about the detachment disturbance**: qualitative foreshadowing
   (Stickybot 2008 on transient propagation; 科学通报 (Chinese Science Bulletin) 2023 on
   "trunk shudder / an open problem") → the force/vibration quantification cluster (Wang21
   peel-angle impedance, Wen25 rotational peeling −65.5%, FTFOF force −28%/vibration −82%,
   Waalbot II / Yoshida & Ma mechanical peeling, MST-G) → the gap: nobody quantifies
   **whole-robot position loss**, and on the vacuum-rupture channel there is only
   cavity-pressure quantification (潘雷 (Pan Lei) 05). ⚠ call out the −82% coincidence of
   units. Lands on contribution 1.
3. **The conceptual lineage of pre-detachment unloading**: the Stickybot patent (the first
   statement, force domain, closed loop) → the LORIS Unload step (online force optimization)
   → LEMUR3/Imai24 (sensor-based management) → RAMP (feedforward, but managing inertial
   reaction forces) → Boscariol13 (quasi-static planning). All of them force domain / closed
   loop / planning layer; this work = displacement domain + offline calibration + open loop +
   slip validation on real hardware. Lands on the narrowed wording of contribution 3.
4. **The classic band of walking-machine force redistribution → modern WBC**: from
   Waldron86/K&W88/Klein90 on, through Chen99 (note that its "sensor-free" refers only to the
   setpoints), Zhou00 and Erden07, up to Righetti13/Focchi17/Bellicoso16/DiCarlo18 — **all of
   them executed in the force domain**; the DLR Crawler says outright that position control
   accumulates internal forces and that its solution is torque sensors. Dallmann17's insects,
   which "swing only after the neighboring leg has taken the load and unloaded this one",
   serve as a biological footnote to the general rule (via load receptors; we are sensor-free
   and open loop). Lands on: displacement-domain open loop is a new solution to this classic
   problem on a "position servo + strong compliance" platform.
5. **Demarcation against the same means, displacement-domain feedforward**: TITAN XI
   (calibrated compliance → foot placement accuracy), Ota06 (compensating static gravity
   deformation on an adhesion platform), milling deflection compensation
   (Wang09/Klimchik14), the Honda patent, sway compensation (geometric transfer of the
   gravity load), Jusufi24 (posture / adhesion angle) — the same mathematical form, but not
   one of them used for "zeroing internal forces before detachment", and not one of them
   dealing with the instantaneous release of stored elastic energy.
6. **Closing out the alternative routes**: sliding suction dodges detachment altogether
   (Yue24, at the cost of continuous power); closing the loop on current-estimated force
   (Wahrburg18, not viable on cheap servos); slow venting (ruled out by this project's 08-20
   data); snap-through used in reverse (Tang20). → The position of this work: zero-sensor
   open loop, a software retrofit, and a closed circle of phenomenon-model-remedy.

## 7. Items that must be checked against the full text by hand before citing (in order of importance)

1. **Kumar & Waldron 1990** (ASME JMD), the body text: does it state outright that "the force
   is exactly zero / is smoothly driven to zero at the moment of lift-off"? If a reviewer
   knows the body text well, failing to cite that argument is a risk; check it before settling
   the citation wording.
2. **The body of FTFOF arXiv:2504.19448**: open-/closed-loop execution details (the PDF is
   >10MB, read the HTML version https://arxiv.org/html/2504.19448v2) — this directly affects
   how strongly this paper can word its "the only open-loop one" claim.
3. **NWPU's "Vacuum adhesion performance analysis and motion switching strategy of a
   wall-climbing robot"** (机械科学与技术 (Mechanical Science and Technology);
   journals.nwpu.edu.cn is behind anti-scraping protection, so only the title was visible) —
   the title contains "motion switching strategy"; re-check by hand once CNKI access is
   available.
4. **The body of Ota ICRA 2006** (implementation details of the deformation compensation;
   currently abstract level only).
5. **The citation trees of Stickybot T-RO 2008 and TITAN XI** (not run, S2 was rate-limited):
   check whether anyone has already carried "pre-detachment unloading / calibrated
   feedforward" over to a suction-cup or open-loop platform.
6. The final CLAWAR 2024 version of Imai vs. arXiv v1 (did they add real-hardware results?
   unlikely).
7. CNKI master's and doctoral full texts (to rule out Chinese-language quantitative
   precedents completely; as things stand, reverse lookups in three Chinese review full texts
   found nothing — indirect evidence).
8. The Dai group's sciengine paper "Adhesion performance of gecko-inspired robot foot pads and
   design of attachment/detachment trajectories under simulated microgravity" (DNS failed, not
   read; a suspected close relative on "detachment trajectory design" (?)).
9. Frontiers Robotics & AI 2022 "Adaptive robot climbing with magnetic feet in unknown
   slippery structure" (online redistribution of the desired ground reaction forces; the title
   was seen, the paper was not opened).
10. Fill in missing bibliographic data: the year of Cyborg cbsystems.0008 (2022 or 2023), the
    venue of Kawasaki & Kikuchi 2014, the volume/issue of Fukuda 1992, the volume/issue of
    Görner's DLR Crawler, the original Japanese title of Yamasaki 2015, verbatim originals of
    the Santos ICRA07 and Geckobot quotes, a final check of the Waalbot II volume/issue, and
    the author names for the bistable jumper / the JMPS crawling paper / 2310.20669.
11. Low priority: the hydraulic execution details in the ASV monograph, the Russian
    (Devjanin/Gurfinkel) and German (Schneider & Schmucker) force-control lines, the LAURON
    series, the body of ROCR, the prosthetics/exoskeleton SEA branch, and Autumn's detachment
    energetics (if a biology footnote is needed).

## 8. Overall assessment of the residual risk

- **Behind the CNKI wall is the only dark area that cannot be ruled out completely** (the body
  text of Chinese master's and doctoral theses); the mitigating evidence is that reverse
  lookups in the full texts of three 2021–2023 Chinese reviews found no work of the
  "quantified detachment slip" kind, so the risk is low. The finished text should use "to the
  best of our knowledge" throughout.
- The S2 search endpoint was rate-limited for the whole session, so relevance ranking was
  missing and there is a very small chance that an obscure title was missed (OpenAlex,
  Crossref and WebSearch were used as three fallbacks).
- Old conference papers (CLAWAR 1998, ISARC) and platform papers whose body text is
  unavailable may contain a sentence or two of **qualitative** description such as "the body
  moves slightly on release" — that does not affect the conclusion "no quantification and no
  open-loop unloading precedent", but do not write anything like "the first to point this out
  qualitatively".
