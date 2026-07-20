# Algorithm / Simulator — Presentation Script (~75 sec)

Allocated time: ~75 seconds (5-minute team video ÷ 4 members: Algorithm, Android, Image Recognition, Hardware).

---

**[0:00–0:08] Intro**

> "Hi, I'm [name] — I own the Algorithm module: the path-planning and the simulator that visualizes and drives the car."

*(Screen: simulator window already open, idle)*

**[0:08–0:15] What it does, one sentence**

> "Given the arena layout and five obstacles, it works out the shortest route to photograph every obstacle's target, then either shows that on screen or actually drives the car through it."

**[0:15–0:45] Core demo — local planning (30s)**

*(Screen: run `simulator.exe --random 3` or your usual demo command, obstacles visible)*

> "Here it's planning a route from scratch — it checks every possible visiting order, picks the shortest one, and works out the exact forward, backward, and turn commands needed to reach each obstacle's face at the right distance and angle. You can see it animate that route now — this is the same command list, in centimeters and degrees, that eventually goes to the real car."

**[0:45–1:05] Live mode — the network + real car link (20s)**

*(Screen: switch to `simulator.exe --live` window / a short clip of it connecting)*

> "For the actual run, the simulator connects over Wi-Fi to the Raspberry Pi on the car, receives the exact arena layout the tablet sent, plans that route live, and — once I add `--execute` — sends each move to the car one at a time, waiting for it to confirm before sending the next, with a live progress bar on screen."

**[1:05–1:15] Wrap / handoff (10s)**

> "So the simulator is really the bridge between 'here's the arena' and 'here's exactly how the car should move' — both for testing on my laptop and for the live run. Over to [next teammate] for [Android / Image Recognition / Hardware]."

---

## Before you record

- Have two things ready to alt-tab / cut between: (1) a local demo (`--random` or your usual obstacle set) already mid-run or ready to launch, and (2) either a real `--live` connection to the car, or — safer if hardware isn't guaranteed to cooperate on camera — a quick screen recording of a working `--live --execute` run you captured earlier as backup.
- Practice the cut points aloud once with a stopwatch — the 30s planning demo and 20s live-mode bit are the two spots most likely to run long if you improvise.
- If you're tight on time, the live-mode section (20s) is the most cuttable — the planning demo alone still satisfies "show the simulator working."
