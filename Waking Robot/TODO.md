# TEL200 Walking Robot Project — Comprehensive TODO List

## 0. Project Setup & Planning

* [ ] Form group and assign roles
* [ ] Create shared workspace (Git repo / folder structure)
* [ ] Set up Python environment with required libraries
* [ ] Download and inspect provided files:

  * walkingTEL200.py
  * house.mat
  * killian.g2o.zip
* [ ] Read relevant course material:

  * Chapter 3–7 (focus on 5, 6, 7)
* [ ] Create project timeline (Gantt chart):

  * Task distribution
  * Meeting schedule
  * Final week reserved for report

---

## 1. Part 1: Motion Planning

### 1.1 Understand Existing Code

* [ ] Run walkingTEL200.py
* [ ] Understand:

  * Leg movement cycles (qcycle)
  * Base coordinate system (robot-centered)
  * Why robot is stationary in world frame

### 1.2 Implement World Motion

* [ ] Implement transformation from world to base frame
* [ ] Update robot pose over time
* [ ] Ensure animation shows robot moving in world coordinates
* [ ] Adjust plotting limits if necessary

### 1.3 Define Motion Primitives

* [ ] Create primitive 1: Move forward 10 cm
* [ ] Create primitive 2: Rotate 1 degree (CW + CCW)
* [ ] Ensure:

  * At least 3 legs always on ground
  * Motion stored in joint-space (qcycle format)
  * Efficient reuse of motion sequences

### 1.4 Test Motion Primitives

* [ ] Test 1: Move from A (0,0,0) → B (100,0,0)
* [ ] Test 2: Rotate from A (0,0,0) → C (0,0,±10°)
* [ ] Test 3: Combine:

  * A (0,0,0) → D (100,0,±10°)

### 1.5 Document Results

* [ ] Save plots/figures
* [ ] Record observations (stability, accuracy, issues)

---

## 2. Part 2: Path Planning (PRM)

### 2.1 Map Setup

* [ ] Load house map:

  * house = rtb_load_matfile("data/house.mat")
* [ ] Adjust scale:

  * 1 cell = 1 cm
* [ ] Verify map dimensions (~3.97m x 5.98m)

### 2.2 Implement PRM

* [ ] Choose number of nodes (npoints)
* [ ] Ensure:

  * Good connectivity
  * Coverage of all rooms
* [ ] Build roadmap graph
* [ ] Implement collision checking

### 2.3 Path Generation

* [ ] Generate at least 5 random start/goal pairs
* [ ] Use SAME roadmap for all tests
* [ ] Compute paths between points

### 2.4 Visualization

* [ ] Plot:

  * Map
  * PRM graph
  * Paths
* [ ] Keep fixed scaling across all plots
* [ ] Ensure full house visible

### 2.5 Robot Execution

* [ ] Convert PRM path → waypoints
* [ ] Convert waypoints → motion primitives
* [ ] Simulate robot following paths

### 2.6 Video Content

* [ ] Show:

  * Random goal generation
  * PRM path
  * Robot movement

### 2.7 Optional Improvement

* [ ] Consider obstacle inflation
* [ ] Evaluate collision issues

---

## 3. Part 3: Localization & Mapping (Optional for Top Grade)

### 3.1 Load Dataset

* [ ] Load pose graph:

  * pg = PoseGraph("data/killian.g2o.zip", lidar=True)
* [ ] Generate scan map:

  * scanMap = pg.scanmap()

### 3.2 Understand Mapping

* [ ] Study:

  * PoseGraph structure
  * scanmap function
* [ ] Understand:

  * Bresenham algorithm
  * Meaning of cell values
* [ ] Write explanation for report

### 3.3 Create Binary Occupancy Grid

* [ ] Define threshold M = 10
* [ ] For each cell:

  * If scans > M → free (0)
  * Else → occupied/unknown (1)
* [ ] Generate KillianMap

### 3.4 Visualization

* [ ] Plot KillianMap
* [ ] Compare with original scanMap

### 3.5 Path Planning on KillianMap

* [ ] Apply PRM on new map
* [ ] Ensure full free-space coverage

### 3.6 Analysis

* [ ] Evaluate PRM performance
* [ ] Suggest better alternatives:

  * Grid-based planners (e.g., A*)
  * Wavefront / Dijkstra
* [ ] Justify reasoning

---

## 4. Report (Max 10 Pages)

### 4.1 Structure (IMRaD)

* [ ] Abstract
* [ ] Introduction
* [ ] Method
* [ ] Results
* [ ] Discussion

### 4.2 Content Requirements

* [ ] Explain:

  * Motion primitives
  * PRM implementation
  * Mapping (if included)
* [ ] Include:

  * Figures and plots
  * Key equations/concepts
* [ ] Show connection to:

  * Course theory (Ch. 3–7)
* [ ] Ensure reproducibility

### 4.3 Formatting

* [ ] Use NMBU template (Word or LaTeX)
* [ ] Max 10 pages (including appendices)
* [ ] Add all group member names

---

## 5. Video (Max 3 Minutes)

* [ ] Show robot:

  * Moving with primitives
  * Following PRM paths
* [ ] Include:

  * Explanation (audio or text)
  * Multiple test cases
* [ ] Keep clear and concise

---

## 6. Final Submission

* [ ] Collect:

  * Report (PDF)
  * Video (file or link)
  * All Python files
* [ ] Zip everything:

  * TEL200_Gr_X.zip
* [ ] Submit before deadline:

  * 2026-04-30

---

## 7. Final Checks

* [ ] Code runs without errors
* [ ] All required tests completed
* [ ] Report is clear and well-structured
* [ ] Video is understandable
* [ ] Strong theory–practice connection demonstrated
