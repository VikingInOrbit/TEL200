# TEL200 Walking Robot Project - A-Grade TODO

## 0. Project Rules

* [x] Keep walkingTEL200.py unchanged as the original assignment baseline.
* [ ] Implement all new behavior outside the baseline file, primarily in walkingTEL200_TestBed.py and main.py.
* [ ] Freeze a reproducible random seed policy for all random tests and figures.

---

## 1. Part 1: Motion Planning

* [ ] Implement world-to-base pose update so robot body motion is shown in world coordinates.
* [ ] Verify plotting limits always show full robot and trajectory.
* [ ] Create and save motion primitive: forward 10 cm in local joint space.
* [ ] Create and save motion primitive: rotate 1 degree clockwise and anticlockwise in local joint space.
* [ ] Verify at least 3 legs support the robot at all times, including turning.
* [ ] Run test A to B: A=(0,0,0) to B=(100,0,0).
* [ ] Run test A to C: A=(0,0,0) to C=(0,0,+10) and C=(0,0,-10).
* [ ] Run test A to D: A=(0,0,0) to D=(100,0,+10) and D=(100,0,-10).
* [ ] Run test A to E sequence: start A=(0,0,0) cm/deg, steps=4F,10CW,4F,10CCW,4F,10CW,4F, expected E≈(158.785,-13.892,-10.0) cm/deg.
* [ ] Save evidence plots and a pose-error table for all four tests.

---

## 2. Part 2: Path Planning (PRM)

* [ ] Load House map from toolbox and confirm map data integrity.
* [ ] Use assignment scale rule: 1 cell = 1 cm, target house size about 3.97 m x 5.98 m.
* [ ] Build PRM roadmap with npoints high enough for full-room connectivity.
* [ ] Reuse the same roadmap for all random start-goal tests.
* [ ] Generate at least 5 random start-goal pairs in free space.
* [ ] Solve and store paths for all test pairs.
* [ ] Create one fixed-scale figure showing full house, roadmap, and all generated paths.
* [ ] Simulate robot movement along planned paths using motion primitives.
* [ ] Document observed collision behavior and optional obstacle inflation note.

---

## 3. Part 3: Localization and Mapping (Required for A-Target)

* [ ] Load pose graph from killian data and generate scanMap.
* [ ] Explain what scanMap integer values represent.
* [ ] Explain Bresenham role in scanMap generation.
* [ ] Build binary KillianMap with threshold M=10.
* [ ] Apply rule: free cell = 0 only if scans > 10, else cell = 1.
* [ ] Visualize scanMap and KillianMap and compare results.
* [ ] Run PRM on KillianMap with full free-space coverage intent.
* [ ] Evaluate more efficient map-based alternatives for this setup.
* [ ] Produce planner comparison table with runtime and path-quality discussion.

---

## 4. Report (IMRaD, Max 10 Pages)

* [ ] Use approved NMBU template only (Word or LaTeX).
* [ ] Include all group members on cover page.
* [ ] Keep language within approved set (English, Norwegian, or Swedish).
* [ ] Structure report as Abstract, Introduction, Method, Results, Discussion.
* [ ] Keep report at maximum 10 pages including appendices.
* [ ] Include enough detail for MSc Robotics reproducibility.
* [ ] Include strong theory-practice links to course chapters 3-7.
* [ ] Include mandatory figures and tables from Parts 1-3.
* [ ] Reserve last 2-3 days before deadline primarily for report quality.

---

## 5. Video (Max 3 Minutes)

* [ ] Keep total duration at or below 3:00.
* [ ] Include commentary by audio and/or text overlay.
* [ ] Show Part 1 motion primitive demonstrations.
* [ ] Show Part 2 random goals, PRM paths, and robot movement.
* [ ] Show Part 3 mapping and planning highlights.
* [ ] Export playable final video and verify accessibility.

---

## 6. Final Submission

* [ ] Collect final report PDF, final video (or valid link), and all Python files.
* [ ] Pack everything into one ZIP named TEL200_Gr_X.zip.
* [ ] Ensure one group submission by group leader.
* [ ] Submit before 2026-04-30.
* [ ] Verify post-submission access for externally hosted video if used.

---

## 7. Final Acceptance Gate

* [ ] Every assignment requirement in TEL200_VT26_WalkingRobot.md is traceable to evidence.
* [ ] All required simulations run without blocking errors.
* [ ] Report is technically sound, concise, and theory-connected.
* [ ] Video clearly demonstrates and explains developed solution.

