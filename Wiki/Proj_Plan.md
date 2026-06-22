# MDP Project Plan — SC2079 / CE3004 (Special Term, Part-Time)
 
**Team size:** 4 · **Stack:** Android Studio (Samsung Tab 7 Lite) + Raspberry Pi 4 + STM32F103RCT6 + Nano Robot Car + fixed camera
**Window:** Week 1 (Tue 23 Jun) → Week 5 (Tue 21 Jul evaluation, Thu 23 Jul video)
**Goal as stated:** hit all checklist/rubric markers. **Goal as corrected:** lock the Week-3 45% early, then buy down integration risk for the Week-5 competition.
 
---
 
## 1. The grade map (read this before assigning anything)
 
| Component | Type | Due | Weight |
|---|---|---|---|
| Project deliverable checklist | Group | **Week 3 — Tue 7 Jul** | **20%** |
| Quiz on individual subsection | **Individual** | **Week 3 — Tue 7 Jul (30 min)** | **20%** |
| Early-stage peer review | Individual | Week 3 | 5% |
| Image-recognition task (live competition) | Group | **Week 5 — Tue 21 Jul** | **25%** |
| Video report (≤5 min, mp4/mov/wmv) | Group | **Week 5 — Thu 23 Jul, 17:00** | **15%** |
| Final-stage peer review | Individual | Week 5 | 15% |
 
**Locked by end of Week 3: 45%. Locked in Week 5: 55%.**
Attendance is compulsory Weeks 1, 3, 5. Labs open (optional) Weeks 2 and 4. Practice slot: Sun 20 Jul, 18:30–21:30 — book it, this is your dress rehearsal for the competition.
 
**Implication:** treat Week 3 as the real deadline for the robot's *fundamentals*. Weeks 4–5 are not for building the basics; they're for integration, tuning, and the competition run.
 
---
 
## 2. Role assignment (4 people, mapped to checklist sections + quiz subsections)
 
The checklist already splits into four clean modules. Each owner is individually accountable for their quiz subsection, so the owner must *understand* their module, not just get it working.
 
| Person | Module (owns) | Checklist items | Quiz subsection |
|---|---|---|---|
| **P1 — Algorithm / Simulator** | Path planning + PC orchestrator (the "brain" on the laptop, talks to RPi over Wi-Fi) | B.1, B.2, B.3 | Algorithms |
| **P2 — Android** | Tablet remote controller + arena UI | C.1–C.10 | Android |
| **P3 — Image Recognition** | Camera capture + detection/recognition model | A.2 | Image recognition |
| **P4 — Hardware / RPi+STM** | Comms backbone + STM motion firmware + motor tuning | A.1, A.3, A.4, A.5 | RPi / STM / motors |
 
**Shared / integration tasks (no single owner — pair up):**
- A.5 (navigate around obstacle to find the image face) needs P1 + P3 + P4.
- The Week-5 competition needs all four.
- The communication protocol (string formats over BT and USB-serial) must be **co-designed by all four in Week 1** — see §4. This is the contract that lets modules be built independently.
**Why this split is honest about its weakness:** P4 carries the most checklist items (A.1, A.3, A.4, A.5) and the hardware is the critical path for *everyone else's* integration. If P4 is your weakest member, swap — put your strongest, most reliable person on hardware. Do not assign hardware to whoever "doesn't mind." The robot not moving straight blocks the whole team.
 
---
 
## 3. The hardware you actually have (from the component list)
 
- **Nano Robot Car:** 2× 12V Hall-encoder motors (rear wheels), **1× servo motor (front steering)** → this is car-style (Ackermann-ish) steering, *not* differential drive. That's why there's a **~25 cm turning radius** that grows with speed. Plan your algorithm around arcs, not point-turns.
- **STM32F103RCT6** microcontroller (motor + servo control, encoder feedback).
- **RPi 4, 2GB** + 32GB microSD (the on-robot computer / comms hub).
- **Fixed camera module**, front-centre. Best recognition distance ≈ **20 cm** from the obstacle.
- **Samsung Tab 7 Lite (SM-T220)** — your Android target device.
- **2550 mAh Li-ion battery** + charger. **Charge it every session** (see Nano charging guide: green LED = full, red = charging). A flat battery mid-demo is an avoidable zero.
**Arena facts (from algorithm briefing) — design to these numbers:**
- 200 cm × 200 cm, virtual boundaries. START zone 40×40 cm, bottom-left.
- 5 obstacles, each 10×10 cm footprint, axis-aligned. One face has the target image; the other three faces have "bull's-eye" markers.
- Robot footprint 20 × 21 cm, camera front-centre.
- Image at `(h, k, F)` where F ∈ {N,S,E,W}. To recognise an S-facing image at `(a,b,S)`, target the robot at roughly `(a−10, b−45)` facing the image. Centres need not align exactly.
- **15 images** in the pool. Competition shows **up to 5**. Full points = all 5 recognised within the **6-minute** timeout; ties broken by **time**.
---
 
## 4. Week 1 (now → 30 Jun): the foundation week. Do not skip any of this.
 
This is the highest-leverage week. Most teams waste it. The two things that matter most: (a) everyone can build *something* by end of week, (b) the comms contract is frozen.
 
**Day-1 / first session (whole team):**
1. Inventory and sign for all hardware. Charge the battery.
2. **Freeze the communication protocol together.** Define the exact string formats now, on paper, before anyone codes. From the checklist these are mandatory:
   - Android → robot: obstacle placement `(x, y, number)`; target-face annotation.
   - Robot → Android: `TARGET, <ObstacleNumber>, <TargetID>` (C.9) and `ROBOT, <x>, <y>, <direction>` (C.10).
   - PC/algorithm ↔ RPi (Wi-Fi) and RPi ↔ STM (USB-serial): movement commands (e.g. `FW010`, `BW010`, `FL`, `FR`, …) and acknowledgements.
   - Write this into a shared `PROTOCOL.md`. **No one deviates without team sign-off.** This single document is what lets four people work in parallel without colliding.
3. Decide repo structure: one Git repo per module (android / rpi / stm / algo), plus a shared `docs/` with PROTOCOL.md and the checklist tracker.
**Per-person Week-1 targets (so Week 3 is achievable):**
 
- **P4 (Hardware) — the critical path, start hardest first:**
  - Flash RPi OS, set **static IP + Wi-Fi hotspot**, install **Apache** (proves A.1 part 1 — browser shows default page).
  - Stand up **RPi ↔ STM USB-serial** (A.1 part 3): send a forward/backward command from RPi, STM drives the motor.
  - Stand up **RPi ↔ tablet Bluetooth (rfcomm)** (A.1 part 2): characters both ways.
  - Begin STM firmware: straight-line with encoder counts (A.3), and rotation/arc turns via servo (A.4).
- **P2 (Android):** project skeleton on the Tab 7 Lite; Bluetooth scan/connect GUI (C.2) + bi-directional text (C.1). Get C.1+C.2 demoable against P4's rfcomm channel by end of week.
- **P3 (Image Rec):** get the camera capturing on RPi; stand up a baseline detection pipeline (YOLO-style or your chosen model) on a laptop first using the 15-image set; produce a bounding box + label on a still image. Don't optimise yet — get the loop working.
- **P1 (Algorithm):** build the **simulator shell** (B.1): 200×200 grid, start zone, obstacles, robot pose + facing, forward/back/turn animation. This is pure software and can be 80% done in Week 1 with zero hardware dependency.
**End-of-Week-1 success test:** every member can demo *one thing* live, and PROTOCOL.md is frozen.
 
---
 
## 5. Week 2 (30 Jun → 7 Jul): clear the checklist. This is the sprint.
 
Target: **walk into Week 3 with almost every checklist item ready to demo to the supervisor.** Remember the marks are progressive and per-item, weighting undisclosed — so clear *quantity*.
 
| Item | Owner | Definition of "done for sign-off" |
|---|---|---|
| **A.1** | P4 | Wi-Fi(Apache page) + BT(tablet chars) + USB-serial(STM moves) all at once; a tablet button → RPi → STM → car moves. |
| **A.2** | P3 | RPi recognises any of 15 images at 20–50 cm, bounding box + image number, shown on PC. |
| **A.3** | P4 | Straight line, stops 80–130 cm, within ±6%, no drift. **Tune encoder counts empirically.** |
| **A.4** | P4 | Rotation 90–360° as called by supervisor. Note: faster = wider turn — calibrate per speed. |
| **A.5** | P4+P3+P1 | Drive to an obstacle (bull's-eye), go *around* it to find the image face. First real integration. |
| **B.1** | P1 | Arena simulator with moving robot on a grid. |
| **B.2** | P1 | Hamiltonian path visiting each image position once, in sim. |
| **B.3** | P1 | Shortest-*time* Hamiltonian path in sim (greedy heuristic first, exhaustive search if ≤5 nodes — it is, so brute-force all 5! orderings is fine and optimal). |
| **C.1–C.10** | P2 | All ten Android items — see §6. |
 
**Algorithm note for B.3:** with only 5 images, the "which order to visit" problem is tiny — exhaustive search over all permutations (≤120) is trivial and gives the optimal ordering. The hard part is the *path between two configurations* given the turning radius. Use the Dubins-path approach from the briefing (CSC / CCC arcs at minimum turning radius). Don't over-engineer the search; spend effort on realistic arc costs that match how the actual car turns.
 
**Booking demos:** the supervisor signs items *as you complete them* during lab. Don't save them all for the last hour of Week 3 — get items signed in Week 2's open lab (30 Jun) if you attend, and the moment they pass in Week 3.
 
---
 
## 6. Android (C.1–C.10) — the most checklist-dense module, and 20% of the video
 
P2 owns this, but the **video grade rewards UI polish heavily** (Android UI Design = 20% of the 15% video, plus it's the explicit "unique distinguishing feature" the rubric calls out). So Android is double-weighted. Build it to look good, not just function.
 
| Item | What it must do |
|---|---|
| C.1 | TX/RX text strings over BT serial. |
| C.2 | Connect button → device list → select → connect. |
| C.3 | Movement control via **buttons / gestures / tilt** (NOT typing strings — explicitly disallowed). |
| C.4 | Status TextView (e.g. "ready", "looking for target 2") — selective info only, not the raw stream. |
| C.5 | 2D arena canvas: numbered obstacle blocks (small white font), robot icon with clear N/S/E/W facing. |
| C.6 | Touch-place + touch-drag obstacles; drag off-map = delete; on finger-lift, send `(x,y,number)`. |
| C.7 | Touch to set which obstacle *face* has the image; appearance changes; send face + coord. |
| C.8 | Robust BT: survive a Disconnect, auto-reconnect on Connect — no app hang. |
| C.9 | On `TARGET,<obs>,<id>` → show Target ID (large white font) + thick coloured line on the image face. |
| C.10 | On `ROBOT,<x>,<y>,<dir>` → update robot position + facing on the map. |
 
**Verify with the AMD tool** before the robot is ready — it simulates the BT peer so P2 isn't blocked waiting on hardware.
 
---
 
## 7. Weeks 4–5: integration and the 25% competition
 
By now checklist marks are banked. Everything left is about the system working *together* under time pressure. This is where the real grade differential lives.
 
**Week 4 (14 Jul) — full-loop integration:**
- Close the loop: Android places obstacles → sends to algorithm (via RPi) → algorithm computes path → commands stream RPi → STM → car drives the route → camera recognises each image → `TARGET`/`ROBOT` strings flow back to update the tablet.
- Run **end-to-end timed trials** in the arena. You're optimising for: (a) recognising all 5, (b) doing it fast.
- Build a **failure recovery path** (briefing issue 2.3: "what if the image is not found"): if recognition fails at a face, the robot should reposition and retry, not stall. Teams that handle this score when others freeze.
- Log every run. Tune turning radius, speeds, stop distances against *measured* behaviour, not assumptions.
**Sun 20 Jul, 18:30–21:30 — practice slot:** full dress rehearsal in the real arena. Treat it as the competition. Find what breaks here, not on Tuesday.
 
**Tue 21 Jul — competition:** charged battery, spare SD card image, printed protocol cheat-sheet, known-good config committed and tagged in Git. Assign roles for the run itself (who places obstacles on the tablet, who starts, who watches time).
 
**Scoring reality:** full points = all 5 within 6 min; ties broken on time; partial recognition still scores. So **reliability beats cleverness** — a robot that *always* gets 4/5 in 5 min beats one that *sometimes* gets 5/5 and sometimes crashes.
 
---
 
## 8. Video report (15%, due Thu 23 Jul 17:00) — start filming in Week 4, not Week 5
 
Five equally-weighted criteria (20% each): **Android UI Design, Creativity, Presentation, Teamwork, Content.** ≤5 minutes; only the first 5 are graded.
 
**Must contain:**
- Each team member + their assigned responsibility (Teamwork + individual contribution must be *visible*).
- Implementations, with **special, explicit focus on the Android UI** — usability, touch design for a small screen, aesthetics, the unique features. This is called out twice in the rubric; it's the easiest place to differentiate.
- Special achievements / anything that gives you an edge over other teams.
**Practical:** capture raw footage *throughout* Weeks 4–5 (every successful run, every UI interaction). You cannot reshoot a working robot the night before. Assign one person (suggest P1 or P2) as editor; everyone appears on camera. Script the 5 minutes tightly — it's a brutal constraint and editing is most of the work.
 
---
 
## 9. Risk register (the things that actually sink MDP teams)
 
| Risk | Why it bites | Mitigation |
|---|---|---|
| **Integration left to the end** | Modules pass alone, fail together; no time to debug the seams | Freeze PROTOCOL.md Week 1; first integration (A.5) in Week 2 |
| **Hardware = critical path** | Car not driving straight blocks algo, image-rec, competition | Best member on P4; calibrate A.3/A.4 against measured data early |
| **Turning radius assumptions** | Algorithm assumes point-turns; real car arcs ~25 cm, wider when fast | Model arcs (Dubins) from day one; measure real radius per speed |
| **"Just basics" mindset** | Underweights the 25% competition (needs full integration) | Treat competition as the main event from Week 4 |
| **Individual quiz (20%)** | One person can't carry; each owner quizzed solo | Every owner learns their module deeply; cross-brief in Week 3 |
| **Dead battery / corrupt SD on demo day** | Avoidable zeros | Charge every session; keep a backup SD image; tag known-good Git commit |
| **Video done last-minute** | Can't reshoot a working robot; 5-min edit takes longer than you think | Film throughout Weeks 4–5; assign editor early |
| **BT/serial flakiness mid-run** | Lost connection = frozen robot | C.8 robust reconnect; serial ACK/retry in protocol |
 
---
 
## 10. This week's action list (do these in the next 7 days)
 
1. Whole team: inventory + sign hardware, **charge battery**.
2. Whole team: write and **freeze `PROTOCOL.md`** (all string formats).
3. Set up the four Git repos + shared `docs/` with a checklist tracker.
4. P4: RPi static IP + hotspot + Apache; RPi↔STM serial; RPi↔tablet rfcomm.
5. P2: Android skeleton + C.1 + C.2 working against P4's rfcomm.
6. P3: camera capture on RPi + baseline recognition on the 15-image set (bounding box + label).
7. P1: simulator shell (B.1) — grid, obstacles, robot pose, movement animation.
8. Book the Sun 20 Jul practice slot.
**The one sentence to remember:** your deadline isn't a month away — 45% of your grade is due in two weeks, and the biggest single component (25%) is won or lost on integration you must start *now*.