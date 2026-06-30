# MDP Algorithm Module — Handoff to Claude Code

**Course:** SC2079/CE3004/CZ3004 Multidisciplinary Design Project, NTU Special Term 2025/26
**My role:** P1 — Algorithm / Path Planning / Simulator
**Today's date:** 29 June 2026. Project window: ~23 Jun – 23 Jul 2026. **Week-3 checklist deadline is the immediate pressure point.**
**Status:** Starting algorithm code from zero today. Infrastructure (RPi networking) is separately handled — see §7. This handoff is scoped to MY module only.

---

## 1. What this project actually is

Four-person team builds an autonomous robot car that navigates a physical 200cm×200cm arena, recognises target images mounted on obstacles, and reports results to an Android tablet. Grading is **checklist-based, progressive, per-item** (20% of grade, Week 3 deadline) plus a live competition, plus an individual quiz (per-member accountability — know your own module cold).

**Team split:**
- **P1 (me):** Algorithm / Path Planning / Simulator — pure software, runs on PC, lowest hardware dependency, should be near-complete by end of Week 1.
- **P2:** Android (Samsung Tab 7 Lite, Android Studio) — Bluetooth control, arena visualization GUI.
- **P3:** Image Recognition (PC/RPi-side) — detect/classify the 15-image pool.
- **P4:** Hardware (RPi 4 + STM32F103RCT6 firmware) — **the critical path / single point of failure.** Everyone else's integration blocks on this.

---

## 2. My module specifically — what I'm building and to what spec

### Checklist items I own (B.1–B.3, from the official assessment doc):
| Item | Requirement |
|---|---|
| **B.1** | Simulator displays the 200×200cm arena, start zone, obstacle locations, image positions. Shows robot moving forward/back/turning on a grid map. |
| **B.2** | Simulator demonstrates a Hamiltonian-path algorithm: robot visits each image position once from the start zone, recognising images within the time limit (partial credit for partial recognition — N of 5 recognized within time = N accepted). |
| **B.3** | Simulator demonstrates the **shortest-time** Hamiltonian path specifically (not just any valid path). |

### Arena ground truth (confirmed from official docs — use these numbers, not generic NTU slide defaults):
- **Movement area:** 200cm × 200cm, virtual boundaries.
- **Start zone:** 40×40cm, bottom-left corner of the arena.
- **Obstacles:** **5 in competition** (pool of up to 15 candidate images, but only 5 obstacles/images shown per run). Each obstacle has a 10×10cm footprint, axis-aligned. One face carries the target image; the other three faces show a generic "bull's-eye" marker (not the target).
- **Robot footprint:** 20cm × 21cm (this is the team's actual robot — **do not use the generic 30×30cm figure from the NTU algorithm-briefing slides**, that's a different reference robot. Confirm actual measured dimensions with P4 if uncertain, but 20×21cm is what's documented in the assessment checklist).
- **Camera:** fixed, front-centre on the robot. Best recognition distance from target ≈ **20cm**.
- **Time limit:** 6 minutes for the run. Full marks = all 5 images recognised within time; ties broken by completion time.
- **Steering:** the car has **Ackermann-style steering (front servo + rear Hall-encoder motors)**, NOT differential drive. No point-turns. Turning radius ~25cm (estimate from earlier project notes — not yet empirically measured on the real car). **Build this as a configurable parameter, not hardcoded** — confirmed decision from prior session.

### Target-approach geometry (from the algorithm briefing — use this for waypoint generation):
- Obstacle position notation: `(h, k, F)` where `(h,k)` is the obstacle's grid cell and `F ∈ {N,S,E,W}` is which face has the target image.
- To recognise a South-facing image at `(a, b, S)`, the robot should be positioned roughly at `(a−10, b−45)` facing the obstacle. (Adjust signs/offsets per face direction — this example is given for S; derive N/E/W analogously by rotating the offset.) Centres don't need to align exactly, some tolerance is fine.
- Robot pose representation: bottom-left grid cell + facing angle, e.g. `(x, y, θ)`.

### Grid resolution — CONFIRM before assuming:
- Two candidate resolutions appear across project docs: **20×20 grid at 10×10cm/cell**, or **40×40 grid at 5×5cm/cell**. Earlier project decision favored 10cm cells with continuous-space Dubins planning for sub-cell precision (collision-checking on an inflated robot footprint, not naive grid blocking) — **car-length-as-a-unit was explicitly evaluated and rejected** due to quantization error and steering-arc complexity. Default to **10×10cm/20×20 grid** unless team's `MDP_Functional_Specification.md` says otherwise — check that file in project knowledge if available.

### Algorithm structure (two-stage problem, per official briefing):
1. **Hamiltonian path / visit ordering:** with only 5 obstacles, this is trivially small — **brute-force all 5! = 120 permutations** for the optimal visit order. Don't build anything fancier (no need for TSP heuristics/ML) — exhaustive search is correct and sufficient per the project plan's own guidance.
2. **Dubins path geometry:** the actual curve generation between two robot configurations (start pose → each waypoint pose). Six path types: **LSL, LSR, RSL, RSR, RLR, LRL** (straight + circular arc combinations). This is the real engineering effort — getting realistic arc costs that match how the physical car actually turns matters more than the visit-ordering search.
   - Reference geometry (rsr/rlr derivations, tangent-point math) is in the project's `algorithms_briefing_24SS.pdf` — pull this from project knowledge if doing path math, it has worked examples with verified numeric answers (e.g. p1=(30,10), p2=(90,70), r=20 → l=84.85, etc.) useful for unit-testing the Dubins implementation.
3. **Obstacle avoidance:** inflate each obstacle's true 10×10cm footprint to a **40×40cm virtual obstacle** (NOT using raw grid-cell blocking) and treat the robot as a point at its centre for collision checking against that inflated boundary.

---

## 3. What's already decided (don't re-litigate these)

- **Language/stack:** Python. Recommended libraries: `numpy`, `dubins` (or hand-rolled Dubins math per the briefing's formulas), `networkx` (optional, likely unnecessary given brute-force is sufficient for 5 nodes), `pygame` for the simulator visualization.
- **First build target (explicitly chosen in prior session, over building grid/Dubins/Hamiltonian pieces in isolation first):** a **full pygame simulator skeleton with visualization** — grid rendering, robot representation with facing indicator, obstacle placement, basic move animation. Build the visual shell first, then wire in real path-planning logic incrementally.
- **Turning radius `r`:** configurable parameter, not hardcoded. Do not block on the physical car being measured before writing code.
- **Coordinate frame:** bottom-left-origin, matches both the algorithm briefing's pose notation and the obstacle `(h,k,F)` notation above. If `PROTOCOL.md` in project knowledge defines something different (it was flagged in earlier sessions as the team's **highest-risk open decision**, §3 of that doc) — check it and reconcile before this becomes an integration bug later. This was explicitly flagged as unresolved as of the last working session.

---

## 4. Integration contract — what my simulator eventually has to talk to

Per the team's protocol doc and architecture: PC (running my algorithm) ↔ Wi-Fi TCP socket ↔ RPi ↔ USB-serial UART ↔ STM32. Relevant message types from `PROTOCOL.md` (search project knowledge for full spec — this is summarized from memory of prior sessions, verify exact format before implementing):
- `ROBOT,<x>,<y>,<dir>` — robot position/heading report.
- `TARGET,<n>,<id>` — recognized target obstacle number + identified image ID.
- Motion primitives: forward/backward distance commands and turn commands (e.g. `FW010`, `BW010`, `FL`, `FR`, or similar — confirm exact strings in `PROTOCOL.md`).
- **Execution model is lock-step:** commands execute one primitive at a time, gated by encoder-truthful DONE signals from the STM32 side. The algorithm does NOT stream continuous control — it sends one motion primitive, waits for confirmation, adapts the next command based on actual achieved position. Plan the simulator's command-issuing logic around this discrete, confirmed-step model, not continuous control.
- This socket-level integration is NOT needed for B.1–B.3 (pure simulator, no live robot) but will matter for the live competition demo later (A.5 integration item) — build the algorithm core with this interface in mind so it's not a rewrite later.

---

## 5. Known unresolved items (flag if they block you, don't silently assume)

1. **Grid resolution** (10cm vs 5cm cells) — check `MDP_Functional_Specification.md` in project knowledge for the team's actual decision; default to 10cm/20×20 if not found.
2. **Coordinate frame definition** — `PROTOCOL.md` §3 was flagged as the team's highest-risk unratified decision as of the last session. Don't assume; verify.
3. **Actual measured robot dimensions** — 20×21cm is from the official checklist doc, treat as authoritative over any other figure encountered.
4. **Turning radius** — not yet empirically measured on the physical car. Keep configurable.
5. **Target-ID-to-image table** (`image_signs.pdf`) — was flagged in an earlier session as corrupted/unverified. Not needed for B.1–B.3 (pure path planning, no real image recognition involved), but will matter when integrating with P3's module later.

---

## 6. Immediate next action

Build the pygame simulator skeleton:
- 200×200cm arena rendered to a window (pick a px-per-cm scale).
- Grid overlay at the resolution decided in §3/§5.
- Start zone rendered (40×40cm, bottom-left).
- Robot rendered as a shape with a clear facing indicator, positioned at start zone initially.
- Manual/scripted obstacle placement (5 obstacles, each with a face annotation N/S/E/W) — hardcode a test layout first, don't build a full obstacle-editing UI yet (that's Android's job, C.5–C.7, not mine).
- Basic animated movement: given a sequence of forward/backward/turn primitives, animate the robot's pose changing on screen frame-by-frame.

Once that shell renders and animates correctly against a hardcoded test path, move to: (a) Dubins path generation between two arbitrary poses, validated against the briefing's worked numeric example, then (b) the brute-force Hamiltonian ordering over the 5 obstacle waypoints, then (c) wire (a)+(b) together so the simulator computes and animates a real shortest-time path end-to-end — that's B.1+B.2+B.3 complete.

---

## 7. Context NOT relevant to this module (don't go down these paths)

A separate, lengthy session today dealt with RPi networking setup (hostapd/dnsmasq/wpa_supplicant debugging on a secondary RPi for future PC↔RPi socket testing). That work is **complete and verified** (AP + DHCP confirmed working end-to-end) but is infrastructure, not algorithm code, and is out of scope for this handoff. Don't re-derive or touch RPi networking config unless explicitly asked — it's done. If you need to test real socket communication against that Pi later, ask for its current IP/SSID rather than assuming the values used during setup are still current.
