\documentclass[nn]{NMBU}

\usepackage{multirow}
\usepackage{float}
\usepackage{algorithm}
\usepackage[noend]{algpseudocode}
\usepackage{graphicx}
\usepackage{paracol}
\usepackage{listings}

\credits{10}
\studyprogramme{Anvendt Robotikk}
\course{TEL200 Introduksjon til Robotikk}
\courseyear{2026}

\setauthor{Brage B. Bestvold, Gabriel Røer, Stian Mo}
\settitle{ABB RobotStudio and YuMi Project}

\begin{abstract}
This report documents the work conducted using the ABB YuMi robot in RobotStudio. The project is divided into three parts: RobotStudio Basics, YuMi Application, and the YuMi Challenge.

The YuMi robot is a collaborative dual-arm robot designed for flexible and safe automation. The main objective of this project was to gain practical experience with RobotStudio and develop robot programs using RAPID, with emphasis on object manipulation and control through digital input signals.
\end{abstract}

\begin{document}

\section{Introduction}

For this TEL200 assignment, we worked with ABB’s YuMi robot in RobotStudio to bring theory into a hands-on work. Our goal was to program the YuMi robot, using RAPID to control motion, detect user input, and handle emergency situations.

The key objectives in this project are:

\begin{itemize}
    \item Show a complete setup and test in RobotStudio with RAPID eg: target positions and trajectory.
    \item Using different move commands, "MoveJ and MoveL", to get both predictable movement.
    \item Use digital I/O "cube, cylinder, prism, home, emergency stop", and use a virtual smart component to visualize emergency states.
\end{itemize}


And ain goal was to take wat we  had lernd about robotics so far and actually apply it in practice. Later in this paper we will go through the method, results and discussion of the project.

\section{Method}


\subsection{setup}

The project setup consists of:
\begin{itemize}
    \item ABB YuMi IRB400 robot, holding two Virtual SmartGripper tools 
    \item Emergency stop button
    \item 4 colored buttons: green, blue, red and yellow
    \item A plate with indents for three geometrical shapes
    \item 3 geometrical objects: a cube, a cylinder and a triangular prism.
\end{itemize}

Only the left arm of the robot is used during this project.


The project was mainly programmed using RAPID code, combining built-in functions such as \texttt{g\_JogOut} with custom-defined functions (e.g., \texttt{unstack}) for object manipulation. RAPID is ABB’s programming language used to control robot motion and logic. The program structure includes motion instructions, control flow, and signal handling. The creation of the digital signals and the smart component were done in the RobotStudio interface.

\subsection{Rapid Code}

The rapid code can be split into 4 parts. Defining the targets, defining procedures for moving between those targets, defining the trap routine and the main loop.

\subsubsection{Targets}

All the targets for the robot was defined using the robtarget function following the example given in \ref{fig:robtarget}

\begin{figure}[H]
    \centering
    \includegraphics[width=0.5\linewidth]{usedAttachments/robtarget.png}
    \caption{Example of defining a target in RAPID}
    \label{fig:robtarget}
\end{figure}



The target is defined as a four-part array. The first part, [x, y, z], specifies the Cartesian coordinates of the robot’s tool center point in space. The second part, [q1, q2, q3, q4], represents the orientation of the tool using quaternion values. The third part, [cf1, cf2, cf3, cf4], contains the configuration data, which encodes the robot’s joint configuration to avoid ambiguity in positioning. The fourth part, [ext1, ext2, ext3, ext4, ext5, ext6], holds external axis values or additional parameters, such as reorientation or axis offsets.

\subsubsection{Movement}

In order to move the robot-arm between these targets, two different types of movement is used:

\begin{itemize}
    \item \textbf{MoveJ}: Joint-based movement. The robot moves each joint independently to reach the target. This is faster and more robust, and was mainly used for larger movements. and guarantees good handeling of singulerities. it was used in the home command to insure the robot is always able to get home.
    \item \textbf{MoveL}: Linear movement. The robot moves the tool in a straight line in Cartesian space. This provides higher precision and was used when approaching objects.
\end{itemize}

Choosing between these is important, as MoveL can lead to singularities, while MoveJ is generally safer but less precise.

\subsubsection{Trap routine}
In order to ensure the robot will be able to stop at all times, a trap routine was connected to di$\_$EmergencySituation. When a trap routine is activated it will immediately stop running the routine it currently is and will run the trap routine instead. Which in this case entails sending a digital output to the smart component, and waiting for the emergency button to be reset.

\subsubsection{Main control loop}
In the main control loop the system continuously checks the digital input signals, and activates the movement routine associated whenever an input changes.


\subsection{Digital signals}

Digital signals are essential for interfacing the robot with external systems and user commands. The signals defined for this project and its type is shown in \ref{tab:IOTable}. 

\begin{table}[H]
    \centering
    \begin{tabular}{c|c}
        Signal name & Signal type\\
         \hline di$\_$cube & DI\\
        di$\_$cylinder & DI\\
        di$\_$prism & DI\\
        di$\_$EmergencySituation & DI\\
        di$\_$home & DI\\
        EmergencyButtonPress & DO\\
         
    \end{tabular}
    \caption{Digital signals defined, DI: Digital input, DO: Digital output}
    \label{tab:IOTable}
\end{table}

Following is an explanation of the digital signals in \ref{tab:IOTable} and their usage:

\begin{itemize}
    \item \texttt{di\_cube}, \texttt{di\_cylinder}, \texttt{di\_prism}: These signals are connected to the green, blue and red buttons and are used to initiate the procedures for movement of the cube, the cylinder and the prism, respectively.
    \item \texttt{di\_EmergencySituation}: This signal is connected to the emergency stop button and acts as an emergency stop. When triggered, a trap routine activates and the robot halts all operations immediately, ensuring safety for both the operator and the equipment.
    \item \texttt{di\_home}: This signal is connected to the yellow button and returns the robot to its home position, resetting its pose and preparing it for the next operation or shutdown.
    \item \texttt{EmergencyButtonPress}: Responsible for moving the Emergency stop button up and down during simulation. It's value is changed in the trap routine EmergencyStopTrap to be the same as \texttt{di\_EmergencySituation}.
\end{itemize}

The RAPID program continuously monitors the input signals, allowing for responsive and safe operation. By structuring the code to handle each input robustly, the system can adapt to various scenarios, such as object detection, emergency stops, and resetting.


 

\subsection{Smart component/mechanism}
In order to model the Emergency stop button moving up and down during simulation, a smart component was made. First a mechanism is created, linking the base and button of the emergency button, with two poses, one with the button in its pressed position and one in its unpressed position. This mechanism is then turned into a smart component. By using PoseMover to switch between the poses based on the digital Output signal, EmergencyButtonPress, the emergency button goes up and down in accordance with di$\_$EmergencySituation. 


\subsection{Theoretical Background}

Robot motion is based on kinematics, which describes how joint movements translate into end-effector position and orientation.

A robot pose consists of both position and orientation in 3D space. This is typically represented using coordinate systems and transformation matrices.

Key concepts used in this project are directly drawn from the TEL200 lectures:

	From Chapter 2: Representing position and orientation
\begin{quote}
"Know position and orientation (Pose)
Coordinate systems and representations
Not only in 2D and 3D but also in joint- or configuration space
Not only now but also as a function of time: Describe motion and time
Time varying pose, paths, trajectories
Accelerating bodies and coordinate systems
Dynamics of rigid, mechanical bodies
Creating smooth paths and trajectories
Kinematic modelling and control of robot (arm)"
\end{quote}

\begin{figure}[H]
    \centering
    \includegraphics[width=0.5\linewidth]{usedAttachments/TEL200_Ch2_p006_x503_y174_w457_h236_01.jpeg}
    \caption{Coordinate frame representation (TEL200 Ch2)}
    \label{fig:yumi_home}
\end{figure}



	\textbf{From Chapter 3: Time and Motion}
\begin{quote}
"Pose: the position and orientation $\xi$ of an object (Ch 2.)
Path: a varying pose $\xi(s)$, for some parameter s
Trajectory: a path with specified timing, $\xi(s(t))$, for t
Defines the time-evolution, or speed along the path from A to B
The notions of path and trajectory can also be generalized to motion in the configuration space or joint space of a robot."
\end{quote}

In our implementation, we used these concepts directly:
- The pick-and-place sequence is defined as a trajectory set by ordered targets in the workcell.
- We adjusted feed rates and motion types (MoveJ versus MoveL) to enforce time evolution and velocity profiles depending on task requirement.
- For straight-line interactions (approach, grasp, place) we selected slower translation speeds and finer motion segments to improve accuracy.

In addition, we included explicit settling handling in our code by inserting short waits and fine stops after critical pick/place moves. This reduces residual oscillations caused by tool load or gripper contact and ensures that the next motion starts from a stable pose.

"Kinematics: A branch of mechanics that studies the motion of a body, or system of bodies. Concerned with positions (and angles) and velocities (translational and angular). Not concerned with mass, forces or moments (that’s Dynamics, Ch. 9)"
\end{quote}
In practical robot programming, it is important to consider both the settling time and the speed of motion. Settling time refers to the period required for the robot to stabilize after a movement, ensuring that the end-effector is accurately positioned before interacting with objects. This is especially critical during tasks that require precision, such as picking or placing items.

The speed of motion is adjusted depending on whether the robot is performing a simple move or an interaction. For example, when moving between distant points (e.g., returning to home position), higher speeds can be used to maximize efficiency. However, during interactions with objects (e.g., grasping or stacking), the speed is reduced to prevent collisions and ensure safe handling. This dynamic adjustment is implemented in the RAPID code by setting different speed parameters for motion instructions, allowing the robot to balance speed and accuracy based on the task.

\begin{figure}[H]
    \centering
    \includegraphics[width=0.5\linewidth]{usedAttachments/TEL200_Ch3_p013_x617_y373_w182_h95_01.png}
    \label{fig:yumi_home}
    \caption{Orientation matrix and rotation axis (TEL200 Ch3)}
\end{figure}


	\textbf{From Chapter 8: Manipulator velocity and Jacobian}
\begin{quote}
"A robot’s end-effector moves in Cartesian space with a translational and rotational velocity – a spatial velocity; However, that velocity is a consequence of the velocities of the individual robot joints; This section introduces the relationship between the velocity of the joints and the spatial velocity of the end-effector: the Jacobian matrix."
\end{quote}
\begin{figure}[H]
    \centering
    \fbox{\parbox{0.7\linewidth}{
        \centering
        \textbf{Placeholder Image} \\[5pt]
    }}
    \label{fig:yumi_home}
    \caption{Manipulator Jacobian and velocity (TEL200 Ch8)}
\end{figure}

\begin{itemize}
    \item \textbf{Forward kinematics}: Calculates end-effector position from joint angles
    \item \textbf{Inverse kinematics}: Determines joint angles required to reach a target
    \item \textbf{Singularities}: Configurations where the robot loses degrees of freedom
    \item \textbf{Manipulator Jacobian}: Relates joint velocities to end-effector velocity
\end{itemize}

	\textbf{Configuration Space and Path Planning}
\begin{quote}
"Not only in 2D \& 3D but also in joint- or configuration space" (TEL200 Ch2)
\end{quote}
Configuration space (C-space) is the set of all possible robot configurations, typically defined by joint angles. Path planning in C-space allows robots to avoid obstacles and reach targets efficiently.


\begin{figure}[H]
    \centering
    \fbox{\parbox{0.7\linewidth}{
        \centering
        \textbf{Placeholder Image} \\[5pt]
    }}
    \label{fig:yumi_home}
    \caption{Configuration space illustration (TEL200 Ch2)}
\end{figure}

	\textbf{Transformation Matrices and Jacobian}
Robot pose and motion are mathematically described using transformation matrices:
\begin{quote}
"The derivative of pose can be determined by expressing pose as a homogeneous transformation matrix and taking the derivative with respect to time..." (TEL200 Ch3)
\end{quote}
Transformation matrices are fundamental tools in robotics for describing the pose (position and orientation) of the robot's end-effector. A homogeneous transformation matrix combines rotation and translation, allowing for easy computation of the robot's pose in different coordinate frames. By chaining these matrices, the robot's kinematic chain can be modeled, enabling calculation of the end-effector's position from joint angles (forward kinematics) and vice versa (inverse kinematics).

The Jacobian matrix is derived from the transformation matrices and represents the relationship between joint velocities and end-effector velocities. It is used for tasks such as resolved-rate motion control, where the desired velocity of the end-effector is translated into joint velocities. The Jacobian also plays a crucial role in force control, as it relates forces applied at the end-effector to torques at the joints. Understanding and computing the Jacobian is essential for advanced robot control, including avoiding singularities and optimizing manipulability.
\begin{figure}[H]
    \centering
    \fbox{\parbox{0.7\linewidth}{
        \centering
        \textbf{Placeholder Image} \\[5pt]
    }}
    \label{fig:yumi_home}
    \caption{Transformation matrix formula (TEL200 Ch3)}
\end{figure}
The manipulator Jacobian relates joint velocities to end-effector velocity:
\begin{quote}
"A Jacobian is the matrix equivalent of the derivative – the derivative of a vector-valued function of a vector with respect to a vector." (TEL200 Ch8)
\end{quote}
\begin{figure}[H]
    \centering
    \fbox{\parbox{0.7\linewidth}{
        \centering
        \textbf{Placeholder Image} \\[5pt]
    }}
    \label{fig:yumi_home}
    \caption{Jacobian matrix formula (TEL200 Ch8)}
\end{figure}

	\textbf{Singularities and Manipulability}
Singularities are robot configurations where the Jacobian loses rank, causing loss of control or infinite velocities:
\begin{quote}
"Singularities: Configurations where the robot loses degrees of freedom" (TEL200 Ch7)
"Jacobian Condition and Manipulability" (TEL200 Ch8)
\end{quote}
Manipulability is often measured by the condition number or determinant of the Jacobian, indicating how well the robot can move in all directions.

	\textbf{Practical Applications}
\begin{itemize}
    \item \textbf{Resolved-rate motion control}: Using the Jacobian to compute joint velocities for desired end-effector velocity.
    \item \textbf{Force-velocity relationships}: The Jacobian also relates forces at the end-effector to joint torques.
\end{itemize}
\begin{quote}
"The Jacobian matrix can be used to compute: Manipulability, Resolved-rate motion control, Force-velocity Relationships, Numerical Inverse Kinematics..." (TEL200 Ch8)
\end{quote}

Simulation in RobotStudio acts as a digital tvin for the real robot. This allows safe testing, debugging, and optimization before deploying code to physical hardware.



\section{Results}

The system was successfully implemented and tested in RobotStudio simulation.

\begin{itemize}
    \item The robot manipulated objects based on digital input signals
    \item The stacking and unstacking logic worked as intended (challenge)
    \item Emergency stop and home functions behaved correctly
\end{itemize}


\begin{figure}[H]
    \centering
    \includegraphics[width=0.5\linewidth]{usedAttachments/Resultater1.png}
    \label{fig:Results1}
    \caption{Robot moving the prism}
\end{figure}

As seen in figure \ref{fig:Results1}, the robot is able to move the geometries across the board.


\begin{figure}[H]
    \centering
    \includegraphics[width=0.5\linewidth]{usedAttachments/Resultater2.png}
    \label{fig:Results2}
    \caption{Robot stacking the geometries}
\end{figure}

As seen in figure \ref{fig:Results2}, the robot is also able to stack the geometries,

 Challenges occurred however, when deploying the program on the physical YuMi robot. While the simulation performed as expected, the real robot did not properly execute the program.

Due to time constraints, all demonstrations and results are based on simulation. The system architecture was designed to allow easy integration with the real robot once communication issues are resolved.

\section{Discussion}

Object manipulation is a fundamental aspect of robotics, enabling automation in manufacturing, logistics, and industrial processes.

In this project, a stacking mechanism was implemented to organize objects and retrieve them when needed. This demonstrates how robots can be used for structured storage and handling tasks. Similar principles can be applied to real-world applications such as conveyor systems and palletizing.

The use of dynamic setpoints and reusable code improves flexibility and scalability. By defining positions and generating paths programmatically, the system becomes easier to modify and extend.

One of the main challenges was transitioning from simulation to real hardware. This highlights the importance of understanding differences between virtual and physical systems, including I/O configuration and controller synchronization.

The project also highlighted the importance of robust error handling and adaptability in robot programming. By designing the system to respond to digital input signals, the robot can quickly adapt to new tasks or respond to unexpected events, such as emergency stops. This flexibility is crucial in industrial environments where requirements can change rapidly.

Another key insight was the value of simulation for development and testing. RobotStudio allowed for safe experimentation and optimization of the RAPID code, reducing the risk of errors when deploying to physical hardware. However, the transition to real-world deployment revealed challenges such as hardware limitations, communication issues, and the need for precise calibration. Addressing these challenges requires a thorough understanding of both the software and hardware aspects of robotics.

Overall, the project demonstrated how theoretical concepts from the TEL200 course, such as kinematics, path planning, and signal handling, are applied in practice. The experience gained will be valuable for future work in robotics, particularly in designing systems that are both flexible and reliable.



\section{Conclusion}

The project provided valuable experience in robot programming using ABB RobotStudio and RAPID. Both basic and advanced robotics concepts were applied in practice.

The results demonstrate how simulation can be used effectively for development, while also highlighting challenges when transferring solutions to physical systems. The YuMi Challenge further emphasized creativity and advanced application design.
Through this project, we gained practical skills in programming, simulation, and troubleshooting industrial robots. The use of RAPID and RobotStudio enabled us to implement complex behaviors, such as object manipulation and emergency handling, while reinforcing theoretical knowledge from the TEL200 course.

Simulation proved to be an invaluable tool for safe development and testing, but the experience also underscored the importance of careful integration and validation when moving to real hardware. The YuMi Challenge fostered creativity and teamwork, encouraging us to design flexible and scalable solutions.

%Skal denne være her? Jeg tror ikke vi skal oppsumere i konkluajonen
In summary, the project bridged the gap between theory and practice, preparing us for future work in robotics and automation. The lessons learned about adaptability, error handling, and system integration will be essential for tackling real-world challenges in the field. It also provided highly valued insight into the implementation aspects of robotics from an MSc Environmental Physics viewpoint.

\section*{Appendix}
%unødvendig? skal ikke rapporten være selvstendig uten Video og Robotstudio, en annen elev som skal gjenskape prosjektet vil jo ikke ha tilgang til disse filene
Video and RobotStudio Pack\&Go file are submitted separately.



\clearpage

\subsection*{RAPID Code Appendix}

%Jeg syns vi bør legge med all koden vår
Below is a section for copy/paste of key RAPID code used in the project. This includes motion instructions, signal handling, and custom functions.

\begin{lstlisting}
! Example RAPID code snippet
PROC main()
    MoveJ home, v100, fine, tool0;
    IF di_cube THEN
        MoveL cube_pick, v50, fine, tool0;
        ! Grasp cube
    ENDIF
    IF di_EmergencySituation THEN
        Stop;
    ENDIF
ENDPROC

\end{lstlisting}


Additional code can be added here as needed for reference.

\bibliography{bib}


\end{document}