\section{Method}

The implementation is written in Python with the RVC3 robotics toolbox and is organized as one navigation pipeline. The pipeline connects three levels: motion primitives, global path planning, and mapping. This keeps the robot state, the global pose, and the map representation separate, but still connected. The work was managed in VS Code, and Git was used to keep the group synchronized through a shared repository and a running TODO list.

\subsection{System Overview}

The walking robot is modeled as four identical legs, each with three joints. The project starts with one leg model, builds a stable gait cycle, and then reuses that cycle as a motion library. Local robot motion is then lifted into world coordinates, and the global planner chooses where the robot should go next. In this way, the code moves from leg motion to body motion to map motion.

\subsection{Motion Planning and Motion Primitives}

The function \mathtt{build_leg_model} creates the kinematic chain for one leg. \texttt{build_gait_cycle} then generates a closed Cartesian foot path, samples it with \texttt{mstraj}, and solves inverse kinematics at each sample. The previous joint solution is used as the seed for the next one, so the joint trajectory stays continuous.

\[
x*{k+1} = x_k + \cos(\phi_k)\Delta s, \qquad
y*{k+1} = y*k + \sin(\phi_k)\Delta s, \qquad
\phi*{k+1} = \phi_k + \Delta\phi .
\]
The continuous gait is turned into discrete primitives and saved for reuse. The project uses four primitive motions:

\begin{itemize}
\item forward translation of $10\,\mathrm{cm}$,
\item backward translation of $10\,\mathrm{cm}$,
\item clockwise rotation of $1^\circ$,
\item counterclockwise rotation of $1^\circ$.
\end{itemize}

These primitives are stored as joint-space trajectories in \texttt{.npz} files, so they can be replayed without recomputing inverse kinematics.

\begin{figure}[ht]
\centering
\includegraphics[width=0.82\linewidth]{output/part1_test4_A_to_E_sequence.png}
\caption{Example of a motion-primitives sequence used in Part 1.}
\label{fig:method-motion-primitives}
\end{figure}

\subsection{Body Motion and Coordinate Transformation}

The robot does not move directly in world coordinates. Instead, each primitive is applied in the local robot frame, and the pose is updated after every step. The update follows a planar rigid-body motion model,

\[
x*{k+1} = x_k + \cos(\phi_k)\Delta s, \qquad
y*{k+1} = y*k + \sin(\phi_k)\Delta s, \qquad
\phi*{k+1} = \phi_k + \Delta\phi .
\]

This is the bridge between the gait and navigation. A forward primitive becomes a forward move on the map, and a turn primitive changes the heading before the next segment starts. \texttt{apply_local_body_step} implements this update, while \texttt{execute_primitive} and \texttt{execute_sequence} record the resulting trajectories and pose errors.

\subsection{Path Planning Using Probabilistic Roadmap (PRM)}

Global path planning on the House map is done with a PRM. \texttt{PRMPlanner_use} loads the map and builds a roadmap with 300 sampled nodes. The roadmap is kept fixed for all queries, which makes the experiments reproducible and keeps the comparisons fair.

For each query, the start and goal positions are connected to the nearest roadmap nodes, and a shortest path is computed on the graph. The result is a geometric path in world coordinates. Because the robot moves with primitives, this path must be converted before execution.

\begin{figure}[ht]
\centering
\includegraphics[width=0.86\linewidth]{output/part2_0_path_plan_map.png}
\caption{PRM path converted into robot motion on the House map.}
\label{fig:method-prm-conversion}
\end{figure}

\paragraph{Path-to-motion conversion}
The function \texttt{pathToSeq} computes the turn needed to face the next segment and the distance needed to follow it. \texttt{followPath} then turns that sequence into alternating rotation and forward primitives. In Part 2.2, the code keeps the robot pose across segments instead of resetting it, so the full sequence behaves like one continuous navigation task.

\subsection{Localization and Mapping}

Part 3 uses LiDAR data from the MIT Killian Court dataset. A pose graph with robot poses and scans is loaded first, and each scan is traced through the grid to build a raw evidence map. Cells along each ray accumulate free-space evidence, while the hit cell receives evidence for occupancy. The result is an integer count map rather than a binary grid.

\paragraph{Scan integration}
The scan evidence is built with ray tracing based on Bresenham's line method. This gives a simple grid representation of what the robot has observed, and it is suitable for thresholding into a binary occupancy map.

\begin{figure}[ht]
\centering
\includegraphics[width=0.86\linewidth]{output/part3_scanmap_raw.png}
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

\begin{figure}[ht]
\centering
\includegraphics[width=0.86\linewidth]{output/part3_killianmap_m10.png}
\caption{Binary KillianMap obtained by thresholding the scan evidence map with $M=10$.}
\label{fig:method-killianmap}
\end{figure}

\subsection{Path Planning on Occupancy Maps}

The binary KillianMap is queried with random free-space start and goal pairs. The code compares PRM, A*, and Dijkstra on the same map and the same queries. A* uses the Euclidean heuristic in grid space, while Dijkstra uses no heuristic. For each query, the code stores whether the planner succeeds, the path length, and the runtime. This makes the comparison between planners direct and measurable.

\subsection{Summary of Methodology}

The method is simple in structure: first build a stable gait, then turn it into reusable motion primitives, then use those primitives to follow global paths, and finally evaluate the same ideas on an occupancy grid built from LiDAR data. The key point is that the same robot is described at three levels, \emph{leg motion}, \emph{body motion}, and \emph{map motion}. That separation is what makes the walking robot usable as a navigation system.

\section{Results}

\subsection{Part 1}

The Part 1 results show that the primitive motions work both on their own and when they are combined into a longer sequence. The robot can move forward, rotate in both directions, and then chain those motions into more complex paths. The regenerated figures use square plots with pose arrows, so the start, target, and final heading are easier to read.

The first figure shows the simplest test. The robot starts at A and moves straight to B, so this plot checks the basic forward primitive and the pose update after one repeated motion.
\begin{figure}[ht]
\centering
\includegraphics[width=0.9\linewidth]{output/part1_test1_A_to_B.png}
\caption{Part 1: forward motion of 10 cm.}
\label{fig:part1-forward}
\end{figure}

The second figure shows the counterclockwise turn test. The robot stays close to the start position because the main change is in heading, not in translation.
\begin{figure}[ht]
\centering
\includegraphics[width=0.9\linewidth]{output/part1_test2_A_to_C_ccw.png}
\caption{Part 1: counterclockwise rotation.}
\label{fig:part1-ccw}
\end{figure}

The third figure shows the same rotation test in the opposite direction. This confirms that the clockwise primitive gives the expected turn while still keeping the robot near the origin.
\begin{figure}[ht]
\centering
\includegraphics[width=0.9\linewidth]{output/part1_test2_A_to_C_cw.png}
\caption{Part 1: clockwise rotation.}
\label{fig:part1-cw}
\end{figure}

The fourth figure combines forward motion with a counterclockwise turn. This is important because it shows that the robot can first translate and then change direction without losing the pose chain.
\begin{figure}[ht]
\centering
\includegraphics[width=0.9\linewidth]{output/part1_test3_A_to_D_ccw.png}
\caption{Part 1: forward motion followed by counterclockwise rotation.}
\label{fig:part1-forward-ccw}
\end{figure}

The fifth figure repeats the same combined motion, but with a clockwise turn. It checks that the opposite turn direction is also consistent when the primitive sequence is reused.
\begin{figure}[ht]
\centering
\includegraphics[width=0.9\linewidth]{output/part1_test3_A_to_D_cw.png}
\caption{Part 1: forward motion followed by clockwise rotation.}
\label{fig:part1-forward-cw}
\end{figure}

The sixth figure is a longer sequence that ends at E. This plot shows that several primitive segments can be chained together and still produce a readable path.
\begin{figure}[ht]
\centering
\includegraphics[width=0.9\linewidth]{output/part1_test4_A_to_E_sequence.png}
\caption{Part 1: longer motion sequence ending at E.}
\label{fig:part1-seq-e}
\end{figure}

The last Part 1 figure extends the same idea to point F. It is the strongest Part 1 check because it uses a longer chain of motions and makes small pose errors easier to notice.
\begin{figure}[ht]
\centering
\includegraphics[width=0.9\linewidth]{output/part1_test4_A_to_F_sequence.png}
\caption{Part 1: longer motion sequence ending at F.}
\label{fig:part1-seq-f}
\end{figure}

\subsection{Part 3}

The Part 3 results show the scan-based map construction and the planner comparison on the resulting KillianMap. The raw evidence map is first built from the LiDAR pose graph, then thresholded into a binary occupancy map, and finally used by PRM, A\*, and Dijkstra. The updated plots are square and keep the legend away from the path lines, which makes the map comparison easier to read.

The raw scan map shows how the lidar rays build up evidence before thresholding. The brighter and darker traces follow the observed walls and free space, so this image is the intermediate map that the rest of Part 3 is based on.
\begin{figure}[ht]
\centering
\includegraphics[width=0.9\linewidth]{output/part3_scanmap_raw.png}
\caption{Part 3: raw scan evidence map.}
\label{fig:part3-raw-scan}
\end{figure}

The binary KillianMap shows the same data after thresholding with $M=10$. This is the version used for planning, so it is the important map for the A\*, Dijkstra, and PRM comparison.
\begin{figure}[ht]
\centering
\includegraphics[width=0.9\linewidth]{output/part3_killianmap_m10.png}
\caption{Part 3: binary KillianMap with threshold $M=10$.}
\label{fig:part3-killian}
\end{figure}

The PRM figure shows the paths found by graph-based planning on the binary map. It is useful because it shows how the roadmap can move around the free space, but it also depends on the sampled graph structure.
\begin{figure}[ht]
\centering
\includegraphics[width=0.9\linewidth]{output/part3_prm_paths.png}
\caption{Part 3: PRM paths on the binary map.}
\label{fig:part3-prm}
\end{figure}

The A\* figure shows the grid-based paths with a heuristic. This usually gives direct routes across the free cells, so it is a good comparison against the roadmap planner.
\begin{figure}[ht]
\centering
\includegraphics[width=0.9\linewidth]{output/part3_astar_paths.png}
\caption{Part 3: A\* paths on the binary map.}
\label{fig:part3-astar}
\end{figure}

The Dijkstra figure shows the same queries without a heuristic. It usually explores more broadly than A\*, so this plot helps show the effect of using the heuristic in the grid search.
\begin{figure}[ht]
\centering
\includegraphics[width=0.9\linewidth]{output/part3_dijkstra_paths.png}
\caption{Part 3: Dijkstra paths on the binary map.}
\label{fig:part3-dijkstra}
\end{figure}
