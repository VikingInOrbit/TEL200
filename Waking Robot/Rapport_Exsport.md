\documentclass[nn]{NMBU}

\usepackage{multirow}
\usepackage{float}
\usepackage{algorithm}
\usepackage[noend]{algpseudocode}

\usepackage{listings}
\usepackage{xcolor} % Optional for colors
\usepackage{gensymb}
\lstset{
language=Python, % Set your language
basicstyle=\ttfamily\small,
keywordstyle=\color{blue},
commentstyle=\color{green},
numbers=left % Adds line numbers
}

\credits{10}
\studyprogramme{Applied Robotics}
\course{TEL200 Introduksjon til Robotikk} % Include the name of the course
\courseyear{2026} % Include the year of the course

\setauthor{Brage Bromset Bestvold, Gabriel Røer}
\settitle{Walking Robot Group 4}

\begin{abstract}

This project turns a walking gait into a navigation system. The work has three steps: first, a local walking primitive is built from leg kinematics; second, those primitives are used to follow PRM paths on the House map; third, a scan-based occupancy map is built from Killian Court data and used for planning. The same code base also measures pose error, path quality, and planner performance.

\end{abstract}

\begin{document}

\section{Introduction} %Denne også først

\subsection{Poses}

A pose describes the position and orientation of a robot in space. In this case, a pose is represented as $(x, y, \theta)$, where $x$ and $y$ define the position and $\theta$ defines the heading angle, where the vector along the positive x-axis corresponds to $\theta = 0$.

\subsection{LiDAR}

A LiDAR (Light Detection and Ranging) sensor is a time-of-flight system that emits laser pulses and measures how long it takes for the reflected light to return to the sensor. By scanning the environment while rotating and tilting the emitter–receiver system, the sensor can measure distances to multiple points in space.

By combining the rotation angle, tilt angle, and time-of-flight measurements, we can compute a set of 3D coordinates where the laser beam reflects off solid objects. This collection of points forms a point cloud, which represents the surrounding environment.

We simulated this type of sensor in Part 3. %TODO liker ikke denne setningen

\subsection{Kinematics}

Kinematics describes the motion of a robot without considering the forces that cause it. It focuses on positions, velocities, and accelerations of the robot’s joints and body.

In robotics, forward kinematics is used to compute the position of the end effector from joint angles, while inverse kinematics is used to determine the joint angles required to reach a desired position.

\subsection{Gait Cycle}

The gait cycle describes how the robot’s joints move over time during locomotion. It defines the pattern of movement that produces a step or stride.

A robot can take many short steps or fewer long steps, but the overall stride length can remain the same depending on the gait. Different gaits result in different motion characteristics, stability, and energy efficiency.

\subsection{Occupancy Map}

An occupancy map is a representation of the environment that indicates where obstacles and free space exist. It is typically generated from LiDAR data.

The environment is divided into a grid where each cell stores a probability of being occupied. In this sense, it can be viewed as a 2D representation derived from a 3D point cloud.

\subsection{PRM}

%Deliberate method used for path planning, since the map of the house is known, (Dette skal i diskusjon)

The probabilistic roadmap method, henceforth referred to as PRM, randomly distributes nodes on the map and connects them together giving a grid independent of any start or goal locations. This roadmap stays consistent throughout the project by using a set seed. The advantage to using a PRM is that once the node-map is created it can be used to go from any start-location to any goal. This is achieved by connecting both the start and goal to its nearest node and finding the shortest path between them using either $A^*$ or Dijkstra.

\subsection{Dijkstra}

Dijkstra’s algorithm is a graph search algorithm used to find the shortest path between nodes. It works by exploring all possible paths in order of increasing cost from the start node. it explores uniformly outward until the goal is reached.

\subsection{$A^*$}

$A^*$ is a pathfinding algorithm used to find the shortest path between two points. It improves on Dijkstra’s algorithm by using a heuristic to estimate the cost from the current node to the goal.

The algorithm evaluates nodes using the cost function:
\[
f(n) = g(n) + h(n)
\]
where $g(n)$ is the cost from the start node to the current node, and $h(n)$ is the estimated cost from the current node to the goal.

$A^*$ always expands the node with the lowest $f(n)$ value, making it efficient while still guaranteeing an optimal path if the heuristic is admissible.

\subsection{Unicycle model}

In this project the robot will follow a unicycle car model, granting a path smoothness of $C$. This allows the robot to make sharp turns, making the path planning process considerably easier.

\subsection{Vi trenger en problemstilling}
%Vi trenger en problemstilling

How does the performance of PRMs compare to grid-based search algorithms such as $A^*$ and Dijkstra in planning feasible paths on a binary occupancy grid derived from real lidar data (KillianMap), in terms of path

\section{Method}

The implementation is written in Python with the RVC3 robotics toolbox and is organized as one navigation pipeline. The pipeline connects three levels: motion primitives, global path planning, and mapping. This keeps the robot state, the global pose, and the map representation separate, but still connected. The work was managed in VS Code, and Git was used to keep the group synchronized through a shared repository and a running TODO list.

\subsection{System Overview}

The walking robot is modeled as four identical legs, each with three joints. The project starts with one leg model, builds a stable gait cycle, and then reuses that cycle as a motion library. Local robot motion is then lifted into world coordinates, and the global planner chooses where the robot should go next. In this way, the code moves from leg motion to body motion to map motion.

\subsection{Motion Planning and Motion Primitives}

The function \texttt{build_leg_model} creates the kinematic chain for one leg. \texttt{build_gait_cycle} then generates a closed Cartesian foot path, samples it with \texttt{mstraj}, and solves inverse kinematics at each sample. The previous joint solution is used as the seed for the next one, so the joint trajectory stays continuous.

\begin{align}
x*{k+1} &= x_k + \cos(\phi_k)\Delta s, \qquad \\
y*{k+1} &= y*k + \sin(\phi_k)\Delta s, \qquad \\
\phi*{k+1} &= \phi_k + \Delta\phi .
\end{align}

The continuous gait is turned into discrete primitives and saved for reuse. The project uses four primitive motions:

\begin{itemize}
\item forward translation of $10\,\mathrm{cm}$,
\item backward translation of $10\,\mathrm{cm}$,
\item clockwise rotation of $1^\circ$,
\item counterclockwise rotation of $1^\circ$.
\end{itemize}

These primitives are stored as joint-space trajectories in \texttt{.npz} files, so they can be replayed without recomputing inverse kinematics.

\subsection{Body Motion and Coordinate Transformation}

The robot does not move directly in world coordinates. Instead, each primitive is applied in the local robot frame, and the pose is updated after every step. The update follows a planar rigid-body motion model,

This is the bridge between the gait and navigation. A forward primitive becomes a forward move on the map, and a turn primitive changes the heading before the next segment starts. \texttt{apply_local_body_step} implements this update, while \texttt{execute_primitive} and \texttt{execute_sequence} record the resulting trajectories and pose errors.

\subsection{Path Planning Using Probabilistic Roadmap (PRM)}

Global path planning on the House map is done with a PRM. \texttt{PRMPlanner_use} loads the map and builds a roadmap with 300 sampled nodes. The roadmap is kept fixed for all queries, which makes the experiments reproducible and keeps the comparisons fair.

For each query, the start and goal positions are connected to the nearest roadmap nodes, and a shortest path is computed on the graph. The result is a geometric path in world coordinates. Because the robot moves with primitives, this path must be converted before execution.

\begin{figure}[H]
\centering
\includegraphics[width=\linewidth]{Figures/part2_0_path_plan_map.png}
\caption{PRM path converted into robot motion on the House map.}
\label{fig:method-prm-conversion}
\end{figure}

The function \texttt{pathToSeq} computes the turn needed to face the next segment and the distance needed to follow it. \texttt{followPath} then turns that sequence into alternating rotation and forward primitives. In Part 2.2, the code keeps the robot pose across segments instead of resetting it, so the full sequence behaves like one continuous navigation task.

\subsection{Localization and Mapping}

Part 3 uses LiDAR data from the MIT Killian Court dataset. A pose graph with robot poses and scans is loaded first, and each scan is traced through the grid to build a raw evidence map. Cells along each ray accumulate free-space evidence, while the hit cell receives evidence for occupancy. The result is an integer count map rather than a binary grid.

The scan evidence is built with ray tracing based on Bresenham's line method. This gives a simple grid representation of what the robot has observed, and it is suitable for thresholding into a binary occupancy map.

\begin{figure}[H]
\centering
\includegraphics[width=1\linewidth]{Figures/part3_scanmap_raw.png}
\caption{Raw scan evidence map built from the Killian Court pose graph.}
\label{fig:method-raw-scanmap}
\end{figure}

\paragraph{Binary occupancy conversion}
The count map is converted into the KillianMap with a threshold $M$. A cell is marked free when its scan count exceeds $M$, and all other cells are treated as occupied or unknown:

\[
K(i,j) =
\begin{cases}
1, & C(i,j) > M, \\
0, & \text{otherwise}.
\end{cases}
\]

In this project, $M=10$. The binary map is then used for planner comparison.

\begin{figure}[H]
\centering
\includegraphics[width=1\linewidth]{Figures/part3_killianmap_m10.png}
\caption{Binary KillianMap obtained by thresholding the scan evidence map with $M=10$.}
\label{fig:method-killianmap}
\end{figure}

\subsection{Path Planning on Occupancy Maps}

The binary KillianMap is queried with random free-space start and goal pairs. The code compares PRM, $A^*$, and Dijkstra on the same map and the same queries. $A^*$ uses the Euclidean heuristic in grid space, while Dijkstra uses no heuristic. For each query, the code stores whether the planner succeeds, the path length, and the runtime. This makes the comparison between planners direct and measurable.

\subsection{Summary of Methodology}

The method is simple in structure: first build a stable gait, then turn it into reusable motion primitives, then use those primitives to follow global paths, and finally evaluate the same ideas on an occupancy grid built from LiDAR data. The key point is that the same robot is described at three levels, \emph{leg motion}, \emph{body motion}, and \emph{map motion}. That separation is what makes the walking robot usable as a navigation system.

\section{Results}

\subsection{Part 1}

The Part 1 results show that the primitive motions work both on their own and when they are combined into a longer sequence. The robot can move forward, rotate in both directions, and chain those motions into more complex paths.

\begin{figure}[H]
\centering
\includegraphics[width=\linewidth]{Figures/TEL200ResultsPart1.png}
\caption{The 5 tests used in part 1, in descending order: move forward 100cm, move 10\degree counteclockwise, move 10\degree clockwise, move forward 100cm and turn 10\degree counterclocwise and move forward 100cm and turn 10\degree clockwise.}
\label{fig:ResultsPart1}
\end{figure}

The generated plots shown in figure \ref{fig:ResultsPart1} use square plots with pose arrows, so the start, target, and final heading are easier to ascertain.

\subsection{Part 2}

The robot was able to move between any two given rooms in the house. In figure \ref{fig:StudyDriveway}, the path the robot used to move from the study to the driveway is shown.

\begin{figure}[H]
\centering
\includegraphics[width=\linewidth]{Figures/part2_0_path_plan_map.png}
\caption{Path followed by robot when it moved from the study to the driveway}
\label{fig:StudyDriveway}
\end{figure}

The complementary video in appendix \ref{Appendix: Video} includes a showcase of the robot walking between 5 such randomly generated start-goal pairs.

Figure \ref{fig:Part2Error} shows the average of the total accumulated error due to restraints in movement, when the robot walked between 5 randomly generated start-goal pairs.

\begin{figure}[H]
\centering
\includegraphics[width=\linewidth]{Figures/Part2Error.png}
\caption{Average total error accumulated during Part 2 for 5 start-goal pairs.}
\label{fig:Part2Error}
\end{figure}

Figure \ref{fig:Part2Error} contain data for error in the robots angle, the robots offset along the y-axis and the robots offset along the x-axis.

\subsection{Part 3}

The Part 3 results show the scan-based map construction and the planner comparison on the resulting KillianMap. The raw evidence map (figure \ref{fig:part3-raw-scan}) is first built from the LiDAR pose graph, then thresholded into a binary occupancy map(figure \ref{fig:part3-killian}), and finally used by PRM (figure \ref{fig:part3-prm}), $A^*$ (figure \ref{fig:part3-astar}) , and Dijkstra (figure \ref{fig:part3-dijkstra}). The updated plots are square and keep the legend away from the path lines, which makes the map comparison easier to read.

\begin{figure}[H]
\centering
\includegraphics[width=\linewidth]{Figures/part3_scanmap_raw.png}
\caption{Part 3: raw scan evidence map.}
\label{fig:part3-raw-scan}
\end{figure}

The raw scan map, visualized in figure \ref{fig:part3-raw-scan}, shows how the lidar rays build up evidence before thresholding. The brighter and darker traces follow the observed walls and free space, so this image is the intermediate map that the rest of Part 3 is based on.

\begin{figure}[H]
\centering
\includegraphics[width=\linewidth]{Figures/part3_killianmap_m10.png}
\caption{Part 3: binary KillianMap with threshold $M=10$.}
\label{fig:part3-killian}
\end{figure}

The binary KillianMap in figure \ref{fig:part3-killian} shows the same data as figure \ref{fig:part3-raw-scan} after thresholding with $M=10$. This is the version used for planning, so it is the important map for the $A^*$, Dijkstra, and PRM comparison.

\begin{figure}[H]
\centering
\includegraphics[width=\linewidth]{Figures/part3_prm_paths.png}
\caption{Part 3: PRM paths on the binary map.}
\label{fig:part3-prm}
\end{figure}

\begin{figure}[H]
\centering
\includegraphics[width=\linewidth]{Figures/part3_astar_paths.png}
\caption{Part 3: $A^*$ paths on the binary map.}
\label{fig:part3-astar}
\end{figure}

\begin{figure}[H]
\centering
\includegraphics[width=\linewidth]{Figures/part3_dijkstra_paths.png}
\caption{Part 3: Dijkstra paths on the binary map.}
\label{fig:part3-dijkstra}
\end{figure}

Figures \ref{fig:part3-prm}, \ref{fig:part3-astar} and \ref{fig:part3-dijkstra} show the paths generated by PRM, $A^*$ and Dijkstra respectively. The paths generated were very similar with small variations visible in some places, PRM does stand out however, as PRM only completed 1 of the 5 queries, whereas $A^*$ and Dijkstra both completed 2 queries. The number of completed queries and the mean runtime for each is shown in table \ref{tab:ComparisonPart3}.

\begin{table}[H]
\centering
\begin{tabular}{c c c c}
& PRM & $A^*$ & Dijkstra & \\
\hline
success count: & 1 & 2 & 2 \\
mean runtime [s]: & 0.949 & 0.189 & 0.213 \\

    \end{tabular}
    \caption{Number of successful queries and mean runtime for the different path finding algorithms tested in part 3}
    \label{tab:ComparisonPart3}

\end{table}

PRM completed one query, $A^*$ and Dijkstra both completed 2. When it comes to the mean runtime, there is a considerable difference between PRM and the two others. There is also a difference between $A^*$ and Dijkstra, but it is considerably smaller. (Table \ref{tab:ComparisonPart3})

\section{Discussion}

This project ties the three parts of robotics together. Part 1 builds the local motion primitives, Part 2 uses them to follow PRM paths on the House map, and Part 3 applies the same idea to scan data from Killian Court. That means the robot is first described at the leg level, then at the body level, and finally at the map level.

%Part 1
In descending order, figure \ref{fig:ResultsPart1} shows the 5 tests used in Part 1. In the first test the robot moves straight forward 100 cm. This test checks the basic forward primitive and the pose update after ten repeated motions.
The next two figure shows the turning tests. first counterclockwise, then clockwise. The robot stays at the start position because the only change is in heading, not in translation. The fourth and fifth figures combines forward motion with respectivly, counterclockwise and clockwise turns. This is important because it shows that the robot can first translate and then change direction without losing the pose chain.

%Part 2
In part 2 the same roadmap is reused for 5 consecutive start and goal pairs. Since the graph stays fixed while the query changes, the tests shows that the roadmap covers the house sufficiently, such that the robot is able to reach all the rooms. The robot completing several routes also indicates that the local primitives and the global planner work together in a consistent way. The robots width is not taken into account when path planning, so in some routes the robots path can bee seen overlapping with corners. This can be observed in multiple locations on figure \ref{fig:StudyDriveway}.

Another limitation in the motion parts is the fixed resolution. The robot moves in fixed $10\,\mathrm{cm}$ steps and $1\degree$ turns, so each segment adds a small approximation error. As shown in figure \ref{fig:Part2Error} the error accumulation in part 2 was small enough to ignore, with the total error in angle for the robot being 0.7\degree in average for the 5 paths the robot followed. Likewise the error along both the x-axis and y-axis were both admissible, with the y-axis being the biggest, having a total error of approximately 0.04 m, or 4 cm. This margin of error is acceptable considering the robot is potentially traveling upwards to 4 m along the y-axis, depending on what the start and goal locations are. Over longer paths however, those small errors accumulate in pose and heading making the shorter paths stay closer to the target, while longer paths are slightly less exact.

%Part 3

The PRM figure shows the paths found by graph-based planning on the binary map. It is useful because it shows how the roadmap can move around the free space, but it also depends on the sampled graph structure.

The $A^*$ figure shows the grid-based paths with a heuristic. This usually gives direct routes across the free cells, so it is a good comparison against the roadmap planner.

The Dijkstra figure shows the same queries without a heuristic. It usually explores more broadly than $A^*$, so this plot helps show the effect of using the heuristic in the grid search.

PRM is useful because the roadmap can be reused, but its result depends on how well the sampled graph covers the free space. $A^*$ is the most direct grid-based method here because the Euclidean heuristic guides the search toward the goal. Dijkstra is the baseline because it finds the shortest path without a heuristic, but it usually expands more nodes and takes longer. Other methods such as visibility-graph planning, wavefront expansion, and potential fields could also be used, but $A^*$ and Dijkstra are the clearest comparison for a binary occupancy map.

The scan map shows how the occupancy grid is built from measurements instead of being given directly. The Bresenham ray tracing adds free-space evidence along each ray and obstacle evidence at the hit cell. Thresholding the count map with $M=10$ converts it into a binary KillianMap. This step is sensitive to noise, grid size, and rounding, so cells near borders and thin structures are the most likely to change classification. That is expected, and it is why the map should be treated as an estimate rather than an exact copy of the environment.

Snakk om Table \ref{tab:ComparisonPart3}

The unicycle model is a useful abstraction for this project because it matches the motion pattern used in the implementation: move forward, turn in place, then move forward again. This makes the primitive-based system easy to connect to navigation. At the same time, it is still a simplification of the real four-legged robot because it does not model slip, balance, or the physical constraints of a legged body. The model is therefore good for control, but not a full physical description.

The walking gait can be reused as a motion library, the library can be combined into global paths, and the same idea can be extended from the House map to scan-based planning on Killian Court data. The main error sources are the expected ones: limited primitive resolution, accumulated pose error, occupancy-grid uncertainty, and simplifying assumptions in the robot model. Even with those limits, the pipeline works as intended.

\section{TODO list}
Liste for Gabriel
Forklare og referere til likninger i metode-del
Sammenlikne de forskjellige metodene i del 3,
Legge inn kilder til alt av teori i introduksjon

Liste for Brage
Skrive om resultater
Generetl rydde opp i rot og omskrive setninger

\appendix{}
\section{Complementary video} \label{Appendix: Video}
A complementary video is in the zip file

\end{document}
