# TEL200 Walking Robot Project Specification (A-Grade Target)

## 1. Purpose

This specification defines all functional, reporting, video, and submission requirements needed to achieve top-grade quality for the TEL200 Walking Robot project.

## 2. Baseline and Constraints

1. `walkingTEL200.py` is the original assignment baseline and must not be modified.
2. New implementation work is performed in derivative files such as `walkingTEL200_TestBed.py` and `main.py`.
3. Assignment requirement source is `TEL200_VT26_WalkingRobot.md`.
4. Deadline is 2026-04-30.
5. Final package name must be `TEL200_Gr_X.zip`.

## 3. Mandatory Deliverables

1. One report in PDF format, maximum 10 pages including appendices.
2. One video, maximum 3 minutes, with audio and/or text commentary.
3. All Python files used in the project.
4. One ZIP package with all deliverables, one submission per group.

## 4. Report Specification

1. One report per group, all member names on cover page.
2. Allowed languages: English, Norwegian, Swedish.
3. Required format source: NMBU Word template or NMBU LaTeX template.
4. Required structure: Abstract, Introduction, Method, Results, Discussion.
5. Reproducibility requirement: enough implementation detail for another MSc Robotics student.
6. Evaluation-critical requirement: strong theory-practice linkage to course literature and syllabus.

## 5. Functional Specification - Part 1 Motion Planning

1. Robot body shall move in world coordinates by applying world-to-base pose transformation.
2. Animation limits shall show complete body and trajectory.
3. Motion primitive P1 shall move robot forward 10 cm.
4. Motion primitive P2 shall rotate robot 1 degree in both directions.
5. Primitives shall be saved and replayed in local joint space for efficiency.
6. Robot shall maintain at least 3 supporting legs at all times, including turning.
7. Required test cases:
   1. A=(0,0,0) to B=(100,0,0).
   2. A=(0,0,0) to C=(0,0,+10) and C=(0,0,-10).
   3. A=(0,0,0) to D=(100,0,+10) and D=(100,0,-10).

## 6. Functional Specification - Part 2 Path Planning

1. House map shall be loaded from toolbox data.
2. Occupancy grid shall be interpreted as 1 cm per cell.
3. PRM shall be used as planner with npoints chosen for full-room connectivity.
4. At least 5 random start-goal pairs shall be planned on the same roadmap.
5. Output shall include fixed-scale full-house visualization with roadmap and paths.
6. Robot shall execute generated waypoint paths using saved motion primitives.
7. Collision side-effects from body size/local origin mismatch may occur and shall be discussed.
8. Obstacle inflation may be evaluated as optional improvement.

## 7. Functional Specification - Part 3 Localization and Mapping

1. Part 3 is required in this A-grade execution plan.
2. Pose graph shall be loaded from killian lidar dataset and converted to scanMap.
3. Report shall explain scanMap cell integer meaning.
4. Report shall explain Bresenham use in scanMap generation.
5. Binary KillianMap shall be generated with M=10 threshold.
6. Cell rule: value 0 only when scans > 10, else value 1.
7. PRM shall be run on KillianMap with free-space coverage goal.
8. Alternative map-based planning methods shall be analyzed and compared for efficiency.

## 8. Evidence Specification

1. Part 1 figures and pose/heading error table for all mandatory tests.
2. Part 2 figure with full map, roadmap, and at least 5 paths.
3. Part 3 figures for raw scanMap, binary KillianMap, and planning results.
4. Planner comparison table for Part 3 alternatives.
5. Video evidence aligned with report claims.

## 9. Requirement Traceability Matrix

1. DEL-01 Report present, max 10 pages, correct structure and format.
2. DEL-02 Video present, max 3 minutes, explanatory commentary included.
3. DEL-03 Python sources included.
4. DEL-04 ZIP name and deadline compliance.
5. P1-01 World-frame robot motion visible.
6. P1-02 Forward 10 cm primitive saved in joint space.
7. P1-03 1 degree turn primitive saved in joint space.
8. P1-04 Three-support-leg rule verified.
9. P1-05 Required tests A-B, A-C, A-D completed and documented.
10. P2-01 House map load and scale rule satisfied.
11. P2-02 PRM connectivity across all rooms.
12. P2-03 At least 5 random pairs using one roadmap.
13. P2-04 Fixed-scale complete-house path visualization.
14. P3-01 scanMap meaning and Bresenham explanation included.
15. P3-02 M=10 KillianMap correctly generated.
16. P3-03 PRM on KillianMap and alternative planner analysis completed.
17. REP-01 Theory-practice linkage to chapters 3-7 is explicit and substantial.

## 10. Quality Bar for A-Target

1. Technical solution is clear, stable, and reproducible.
2. Simulations are technically sound and well explained.
3. Report is concise, structured, and deeply connected to course theory.
4. Video communicates implementation and results clearly.
5. Part 3 is completed with analysis quality, not only attempted.

## 11. Timeline Gate Plan

1. Gate A: Part 1 complete and evidenced.
2. Gate B: Part 2 complete and evidenced.
3. Gate C: Part 3 complete and evidenced.
4. Gate D: Report and video complete and internally reviewed.
5. Gate E: Submission package validated and submitted on time.
