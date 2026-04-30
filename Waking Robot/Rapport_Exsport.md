\section{Introduction} %Denne også først

\subsection{Poses}

A pose describes the position and orientation of a robot in space. In this case, a pose is represented as $(x, y, \theta)$, where $x$ and $y$ define the position and $\theta$ defines the heading angle, where the vector along the positive x-axis corresponds to $\theta = 0$.

\subsection{LiDAR}

A LiDAR (Light Detection and Ranging) sensor is a time-of-flight system that emits laser pulses and measures how long it takes for the reflected light to return to the sensor. By scanning the environment while rotating and tilting the emitter–receiver system, the sensor can measure distances to multiple points in space. \cite{Anisi2026WalkingRobot}

By combining the rotation angle, tilt angle, and time-of-flight measurements, we can compute a set of 3D coordinates where the laser beam reflects off solid objects. This collection of points forms a point cloud, which represents the surrounding environment.

We simulated this type of sensor in Part 3. %TODO liker ikke denne setningen

\subsection{Kinematics}

Kinematics describes the motion of a robot without considering the forces that cause it. It focuses on positions, velocities, and accelerations of the robot’s joints and body.\cite{Anisi2026Ch7}

In robotics, forward kinematics is used to compute the position of the end effector from joint angles, while inverse kinematics is used to determine the joint angles required to reach a desired position.\cite{Anisi2026Ch7}

\subsection{Gait Cycle}

The gait cycle describes how the robot’s joints move over time during locomotion. It defines the pattern of movement that produces a step or stride.\cite{Anisi2026WalkingRobot}

A robot can take many short steps or fewer long steps, but the overall stride length can remain the same depending on the gait. Different gaits result in different motion characteristics, stability, and energy efficiency.

\subsection{Occupancy Map}

An occupancy map is a representation of the environment that indicates where obstacles and free space exist. It is typically generated from LiDAR data. \cite{Anisi2026WalkingRobot}

The environment is divided into a grid where each cell stores a probability of being occupied. In this sense, it can be viewed as a 2D representation derived from a 3D point cloud.

\subsection{Ray Tracing and Evidence Accumulation}

Ray tracing is a fundamental technique in scan-based mapping where laser rays are traced through a grid from the sensor to the detected obstacles. As each ray passes through grid cells, it accumulates evidence in two ways: cells along the ray path record free-space evidence (indicating they do not contain obstacles), while the cell at the ray's end-point (where the laser reflects) records occupancy evidence (indicating an obstacle is present). By combining evidence from many scans, a map can be built where each cell contains an integer count representing how many times that cell was observed as either free or occupied. This count map is then thresholded to produce a binary occupancy map. \cite{Anisi2026WalkingRobot}

\subsection{Bresenham's Line Algorithm}

Bresenham's line algorithm is an efficient method for tracing a straight line through a discrete grid. It determines which grid cells a line passes through using only integer arithmetic, making it computationally efficient. In the context of scan mapping, Bresenham's algorithm is used to determine all grid cells that lie along the path of a laser ray, allowing rapid accumulation of free-space evidence along the entire ray path rather than just at the impact point. \cite{Anisi2026WalkingRobot}

\subsection{PRM}

%Deliberate method used for path planning, since the map of the house is known, (Dette skal i diskusjon)

The probabilistic roadmap method, henceforth referred to as PRM, randomly distributes nodes on the map and connects them together giving a grid independent of any start or goal locations. This roadmap stays consistent throughout the project by using a set seed. The advantage to using a PRM is that once the node-map is created it can be used to go from any start-location to any goal. This is achieved by connecting both the start and goal to its nearest node and finding the shortest path between them using either $A^*$ or Dijkstra. \cite{Anisi2026Ch5NavI}

\subsection{Dijkstra}

Dijkstra’s algorithm is a graph search algorithm used to find the shortest path between nodes. It works by exploring all possible paths in order of increasing cost from the start node. it explores uniformly outward until the goal is reached. \cite{Anisi2026Ch5NavII}

\subsection{$A^*$}

$A^*$ is a pathfinding algorithm used to find the shortest path between two points. It improves on Dijkstra’s algorithm by using a heuristic to estimate the cost from the current node to the goal. \cite{Anisi2026Ch5NavII}

The algorithm evaluates nodes using the cost function:
\[
f(n) = g(n) + h(n)
\]
where $g(n)$ is the cost from the start node to the current node, and $h(n)$ is the estimated cost from the current node to the goal.

$A^*$ always expands the node with the lowest $f(n)$ value, making it efficient while still guaranteeing an optimal path if the heuristic is admissible.

\subsection{Unicycle model}

In this project the robot will follow a unicycle car model, granting a path smoothness of $C$. This allows the robot to make sharp turns, making the path planning process considerably easier. \cite{Anisi2026Ch4}

\clearpage
\subsection{References}
\begin{thebibliography}{9}
\bibitem{Anisi2026WalkingRobot} Anisi, D.A., Nordlie, H. and Aslaksen, L.H. (2026) "TEL200 Walking Robot Project", lecture notes, Norwegian University of Life Sciences (NMBU). Available at: Waking Robot/TEL200_VT26_WalkingRobot.pdf (Accessed: 30 April 2026).

\bibitem{Anisi2026Ch7} Anisi, D.A. (2026) "TEL200 Chapter 7: Robot manipulator kinematics", lecture notes, Norwegian University of Life Sciences (NMBU). Available at: Waking Robot/TEL200_Ch7.pdf (Accessed: 30 April 2026).

\bibitem{Anisi2026Ch5NavI} Anisi, D.A. (2026) "TEL200 Chapter 5: Navigation I, Probabilistic Roadmap Method", lecture notes, Norwegian University of Life Sciences (NMBU). Available at: Waking Robot/TEL200 Ch 5 - Navigation I.pdf (Accessed: 30 April 2026).

\bibitem{Anisi2026Ch5NavII} Anisi, D.A. (2026) "TEL200 Chapter 5: Navigation II, Dijkstra's algorithm and A\* search", lecture notes, Norwegian University of Life Sciences (NMBU). Available at: Waking Robot/TEL200 Ch 5 - Navigation II.pdf (Accessed: 30 April 2026).

\bibitem{Anisi2026Ch4} Anisi, D.A. (2026) "TEL200 Chapter 4: Mobile Robots, Unicycle model", lecture notes, Norwegian University of Life Sciences (NMBU). Available at: Waking Robot/TEL200 Ch 4 - Mobile_Robots.pdf (Accessed: 30 April 2026).
\end{thebibliography}
