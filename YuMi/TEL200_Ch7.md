# TEL200_Ch7

Source PDF: TEL200_Ch7.pdf

## Page 1

### Images

#### Image 1 (Page 1)
![Image 1 - Page 1](Attachments_Ch7/TEL200_Ch7_p001_x480_y203_w202_h135_01.png)

---

## Page 2

TEL200 – Introduction to Robotics
David A. Anisi
Chapter 7: Robot manipulator kinematics

---

## Page 3

TEL200 Introduction to Robotics
Change of schedule from next week:
• Project/lab:
Tuesdays 
14.15-18.00 
TF4-102
Thursdays
12.15-14.00
TF4-102
Fridays
8.15-10.00
TF4-102
Check time-edit for updated schedule!
2

---

## Page 4

Agenda
7.1 Forward Kinematics
7.2 Inverse Kinematics
7.3 Trajectories

### Images

#### Image 1 (Page 4)
![Image 1 - Page 4](Attachments_Ch7/TEL200_Ch7_p004_x792_y363_w168_h168_01.jpeg)

#### Image 2 (Page 4)
![Image 2 - Page 4](Attachments_Ch7/TEL200_Ch7_p004_x792_y85_w181_h181_02.jpeg)

#### Image 3 (Page 4)
![Image 3 - Page 4](Attachments_Ch7/TEL200_Ch7_p004_x466_y100_w300_h300_03.jpeg)

#### Image 4 (Page 4)
![Image 4 - Page 4](Attachments_Ch7/TEL200_Ch7_p004_x696_y201_w167_h167_04.jpeg)

#### Image 5 (Page 4)
![Image 5 - Page 4](Attachments_Ch7/TEL200_Ch7_p004_x624_y373_w168_h140_05.jpeg)

#### Image 6 (Page 4)
![Image 6 - Page 4](Attachments_Ch7/TEL200_Ch7_p004_x471_y337_w134_h201_06.jpeg)

---

## Page 5

Kinematics
• From the Greek word for motion
• A branch of mechanics that studies the 
motion of a body, or system of bodies
• Concerned with positions (and angles) 
and velocities (translational and angular) 
• Not concerned with mass, forces or 
moments (that’s Dynamics, Ch. 9)

### Images

#### Image 1 (Page 5)
![Image 1 - Page 5](Attachments_Ch7/TEL200_Ch7_p005_x559_y124_w311_h291_01.jpeg)

---

## Page 6

Kinematics
$$
The relationship and mapping between configuration (e.g., joint coordinates q = [q1, … , qN])
$$
$$
and end-effector pose (e.g., Cartesian: 𝝃= [x, y, z, Φ1, Φ2, Φ3])
$$
Forward kinematics
Backward kinematics
q  
 𝝃

---

## Page 7

Some robot manipulator terminology
• Base is the link of the manipulator which is 
usually connected to the ground/mobile unit
• Joint is a part of the robot body which 
allows controlled or free relative motion of 
two links. Joint qi connects Linki and Linki+1
• Link is the rigid part of the robot body 
(upper arm, forearm, etc.)
• End effecter is the link of the manipulator 
which is used to hold the tools (gripper, 
spray/welding gun, etc.)
Base
Shoulder
Upper arm
Elbow block
Forearm
End effector
q0
q3
q2
q1
q4
q5
Wrist

### Images

#### Image 1 (Page 7)
![Image 1 - Page 7](Attachments_Ch7/TEL200_Ch7_p007_x490_y151_w340_h340_01.jpeg)

---

## Page 8

How do we go between joint-values and TCP?
Tool Center Point 
(TCP)

### Images

#### Image 1 (Page 8)
![Image 1 - Page 8](Attachments_Ch7/TEL200_Ch7_p008_x124_y138_w712_h401_01.jpeg)

---

## Page 9

How do we go between joint-values and pose? vice verca?
Tool Center Point 
(TCP)

### Images

#### Image 1 (Page 9)
![Image 1 - Page 9](Attachments_Ch7/TEL200_Ch7_p009_x8_y136_w599_h398_01.jpeg)

#### Image 2 (Page 9)
![Image 2 - Page 9](Attachments_Ch7/TEL200_Ch7_p009_x608_y343_w353_h195_02.jpeg)

---

## Page 10

How do we go between linear- and axis-wise jogging?

### Images

#### Image 1 (Page 10)
![Image 1 - Page 10](Attachments_Ch7/TEL200_Ch7_p010_x503_y201_w403_h222_01.jpeg)

#### Image 2 (Page 10)
![Image 2 - Page 10](Attachments_Ch7/TEL200_Ch7_p010_x56_y199_w402_h225_02.jpeg)

---

## Page 11

How do we go between robtarget and jointtarget?

### Images

#### Image 1 (Page 11)
![Image 1 - Page 11](Attachments_Ch7/TEL200_Ch7_p011_x272_y142_w417_h401_01.jpeg)

---

## Page 12

How do we go between robtarget and jointtarget?

### Images

#### Image 1 (Page 12)
![Image 1 - Page 12](Attachments_Ch7/TEL200_Ch7_p012_x46_y327_w772_h82_01.png)

#### Image 2 (Page 12)
![Image 2 - Page 12](Attachments_Ch7/TEL200_Ch7_p012_x102_y403_w615_h88_02.jpeg)

#### Image 3 (Page 12)
![Image 3 - Page 12](Attachments_Ch7/TEL200_Ch7_p012_x95_y199_w609_h110_03.jpeg)

#### Image 4 (Page 12)
![Image 4 - Page 12](Attachments_Ch7/TEL200_Ch7_p012_x49_y148_w766_h55_04.png)

---

## Page 13

Arm Types
• Arm-type robots or robot manipulators are a very common type of robot.
2
IRB 910SC
IRB 360
Parallel/Delta robot
Payload: up to 8 kg
Reach: 0,8 – 1,6 m
SCARA robot
Payload: 6 kg
Reach: 0.45, 0.55, 0.65 m

### Images

#### Image 1 (Page 13)
![Image 1 - Page 13](Attachments_Ch7/TEL200_Ch7_p013_x121_y219_w300_h300_01.jpeg)

#### Image 2 (Page 13)
![Image 2 - Page 13](Attachments_Ch7/TEL200_Ch7_p013_x641_y219_w219_h219_02.jpeg)

#### Image 3 (Page 13)
![Image 3 - Page 13](Attachments_Ch7/TEL200_Ch7_p013_x193_y470_w84_h40_03.png)

#### Image 4 (Page 13)
![Image 4 - Page 13](Attachments_Ch7/TEL200_Ch7_p013_x253_y470_w117_h40_04.png)

#### Image 5 (Page 13)
![Image 5 - Page 13](Attachments_Ch7/TEL200_Ch7_p013_x645_y465_w131_h40_05.png)

---

## Page 14

Arm Types
• Arm-type robots or robot manipulators are a very common type of robot.
2
IRB 910SC
IRB 360
Parallel/Delta robot
SCARA robot
•
Food handling
•
Picking and packing tasks
•
Material handling
•
Small parts assembly
•
Inspection

### Images

#### Image 1 (Page 14)
![Image 1 - Page 14](Attachments_Ch7/TEL200_Ch7_p014_x121_y219_w300_h300_01.jpeg)

#### Image 2 (Page 14)
![Image 2 - Page 14](Attachments_Ch7/TEL200_Ch7_p014_x641_y219_w219_h219_02.jpeg)

#### Image 3 (Page 14)
![Image 3 - Page 14](Attachments_Ch7/TEL200_Ch7_p014_x193_y470_w84_h40_03.png)

#### Image 4 (Page 14)
![Image 4 - Page 14](Attachments_Ch7/TEL200_Ch7_p014_x253_y470_w117_h40_04.png)

#### Image 5 (Page 14)
![Image 5 - Page 14](Attachments_Ch7/TEL200_Ch7_p014_x645_y465_w131_h40_05.png)

---

## Page 15

Arm Types
• Arm-type robots or robot manipulators are a very common type of robot.
2
IRB 6620LX
IRB 660
Gantry robot
Articulated robot
Payload: 150 kg
Reach: 1.8 – 33.0 m (L) 
             2.5 – 4.0 m (H)
Payload: 250 kg
Reach: 3.15 m

### Images

#### Image 1 (Page 15)
![Image 1 - Page 15](Attachments_Ch7/TEL200_Ch7_p015_x121_y200_w300_h300_01.jpeg)

#### Image 2 (Page 15)
![Image 2 - Page 15](Attachments_Ch7/TEL200_Ch7_p015_x539_y191_w283_h283_02.jpeg)

#### Image 3 (Page 15)
![Image 3 - Page 15](Attachments_Ch7/TEL200_Ch7_p015_x164_y467_w78_h40_03.png)

#### Image 4 (Page 15)
![Image 4 - Page 15](Attachments_Ch7/TEL200_Ch7_p015_x217_y467_w29_h40_04.png)

#### Image 5 (Page 15)
![Image 5 - Page 15](Attachments_Ch7/TEL200_Ch7_p015_x223_y467_w65_h40_05.png)

#### Image 6 (Page 15)
![Image 6 - Page 15](Attachments_Ch7/TEL200_Ch7_p015_x603_y466_w109_h40_06.png)

#### Image 7 (Page 15)
![Image 7 - Page 15](Attachments_Ch7/TEL200_Ch7_p015_x223_y467_w65_h40_07.png)

---

## Page 16

Arm Types
• Arm-type robots or robot manipulators are a very common type of robot.
2
IRB 6620LX
IRB 660
•
Material handling
•
Palletizing
•
Machine tending
•
Heavy arc welding
•
Grinding
•
Painting

### Images

#### Image 1 (Page 16)
![Image 1 - Page 16](Attachments_Ch7/TEL200_Ch7_p016_x121_y200_w300_h300_01.jpeg)

#### Image 2 (Page 16)
![Image 2 - Page 16](Attachments_Ch7/TEL200_Ch7_p016_x539_y191_w283_h283_02.jpeg)

---

## Page 17

Arm Types
• Arm-type robots or robot manipulators are a very common type of robot.
2
YuMi – IRB 14050
YuMi – IRB 14000
Dual-arm robot
Single-arm robot
Inherently safe design: rounded edges, lightweight, power/force limitation, force/torque (F/T) sensors;

### Images

#### Image 1 (Page 17)
![Image 1 - Page 17](Attachments_Ch7/TEL200_Ch7_p017_x89_y152_w439_h293_01.jpeg)

#### Image 2 (Page 17)
![Image 2 - Page 17](Attachments_Ch7/TEL200_Ch7_p017_x543_y106_w226_h339_02.jpeg)

#### Image 3 (Page 17)
![Image 3 - Page 17](Attachments_Ch7/TEL200_Ch7_p017_x232_y425_w61_h40_03.png)

#### Image 4 (Page 17)
![Image 4 - Page 17](Attachments_Ch7/TEL200_Ch7_p017_x269_y425_w30_h40_04.png)

#### Image 5 (Page 17)
![Image 5 - Page 17](Attachments_Ch7/TEL200_Ch7_p017_x275_y425_w101_h40_05.png)

#### Image 6 (Page 17)
![Image 6 - Page 17](Attachments_Ch7/TEL200_Ch7_p017_x535_y421_w74_h40_06.png)

#### Image 7 (Page 17)
![Image 7 - Page 17](Attachments_Ch7/TEL200_Ch7_p017_x269_y425_w30_h40_07.png)

#### Image 8 (Page 17)
![Image 8 - Page 17](Attachments_Ch7/TEL200_Ch7_p017_x275_y425_w101_h40_08.png)

---

## Page 18

Arm Types
• Arm-type robots or robot manipulators are a very common type of robot.
2
YuMi – IRB 14050
YuMi – IRB 14000
Human-robot collaboration:
•
Assembly 
•
Pharmaceutical
•
Healthcare

### Images

#### Image 1 (Page 18)
![Image 1 - Page 18](Attachments_Ch7/TEL200_Ch7_p018_x89_y152_w439_h293_01.jpeg)

#### Image 2 (Page 18)
![Image 2 - Page 18](Attachments_Ch7/TEL200_Ch7_p018_x543_y106_w226_h339_02.jpeg)

---

## Page 19

Manipulators with branching chains
Atlas
HUBO2
Spot

### Images

#### Image 1 (Page 19)
![Image 1 - Page 19](Attachments_Ch7/TEL200_Ch7_p019_x616_y170_w322_h290_01.jpeg)

#### Image 2 (Page 19)
![Image 2 - Page 19](Attachments_Ch7/TEL200_Ch7_p019_x57_y146_w221_h332_02.jpeg)

#### Image 3 (Page 19)
![Image 3 - Page 19](Attachments_Ch7/TEL200_Ch7_p019_x309_y149_w276_h332_03.jpeg)

---

## Page 20

Industrial robots
• Classification: serial mechanical structure;
a.
Spherical (1%);
b.
Cylindrical (12%);
c.
Cartesian (20%) ;
d.
Articulated (59%);
e.
SCARA (8%).
2
- (b)
- (a)
- (c)
- (e)
- (d)
Source IFR (2019): Percentage of installed
robot manipulators worldwide.

### Images

#### Image 1 (Page 20)
![Image 1 - Page 20](Attachments_Ch7/TEL200_Ch7_p020_x503_y140_w452_h306_01.jpeg)

#### Image 2 (Page 20)
![Image 2 - Page 20](Attachments_Ch7/TEL200_Ch7_p020_x752_y22_w71_h50_02.jpeg)

---

## Page 21

Introduction
• A robot arm, more formally a serial-link manipulator, comprises a chain of
rigid links and joints;
2

---

## Page 22

Introduction
• A robot arm, more formally a serial-link manipulator, comprises a chain of
rigid links and joints;
• Each joint has one Degree of Freedom (DoF), either translational (a sliding or
prismatic joint) or rotational (a revolute joint).
2
Prismatic
Revolute
Schematic representation of robot joints [2].

### Images

#### Image 1 (Page 22)
![Image 1 - Page 22](Attachments_Ch7/TEL200_Ch7_p022_x444_y305_w131_h116_01.png)

#### Image 2 (Page 22)
![Image 2 - Page 22](Attachments_Ch7/TEL200_Ch7_p022_x219_y304_w128_h113_02.jpeg)

---

## Page 23

Introduction
• Motion of a given joint changes the relative pose of the links that it connects;
• One end of the chain, the base, is generally fixed and the other end is free to
move in space and holds the tool or end-effector that does the useful work.
2
{𝐵}
{𝐸}
Schematic representation of a robot arm [2].

### Images

#### Image 1 (Page 23)
![Image 1 - Page 23](Attachments_Ch7/TEL200_Ch7_p023_x191_y253_w374_h259_01.jpeg)

---

## Page 24

Introduction
• Motion of a given joint changes the relative pose of the links that it connects;
• One end of the chain, the base, is generally fixed and the other end is free to
move in space and holds the tool or end-effector that does the useful work.
2
{𝐵}
{𝐸}
Position:       𝑝𝑒𝑏= [ 𝑥𝑒𝑏 𝑦𝑒𝑏 𝑧𝑒𝑏 ]
Orientation:  𝑅𝑒𝑏= 𝑛𝑒𝑏𝑠𝑒𝑏𝑎𝑒𝑏
Schematic representation of a robot arm [2].

### Images

#### Image 1 (Page 24)
![Image 1 - Page 24](Attachments_Ch7/TEL200_Ch7_p024_x191_y253_w374_h259_01.jpeg)

---

## Page 25

Forward Kinematics (Ch. 7.1)
• Forward kinematics is the mapping from joint coordinates 𝒒, or robot
configuration, to end-effector pose 𝝃𝑬given by:
where 𝒦⋅is - in general - a nonlinear function
2
𝝃𝑬=  𝒦(𝒒)
(1)
Forward kinematics
Backward kinematics
q  
 𝝃

---

## Page 26

2-Dimensional (Planar) Robotic Arms
• Consider this simple robot arm, which has a single rotational joint.
2
Planar arm with one rotational joint.

### Images

#### Image 1 (Page 26)
![Image 1 - Page 26](Attachments_Ch7/TEL200_Ch7_p026_x270_y162_w260_h167_01.jpeg)

---

## Page 27

2-Dimensional (Planar) Robotic Arms
• Consider this simple robot arm, which has a single rotational joint.
• We can describe the pose of its end-effector – frame {𝐸} – by a sequence of
relative poses:
…a rotation about the joint axis 𝑞1 and then a translation by 𝑎1 along the
rotated 𝑥-axis;
2
Planar arm with one rotational joint.

### Images

#### Image 1 (Page 27)
![Image 1 - Page 27](Attachments_Ch7/TEL200_Ch7_p027_x270_y162_w260_h167_01.jpeg)

---

## Page 28

2-Dimensional (Planar) Robotic Arms
• Consider this simple robot arm, which has a single rotational joint.
• We can describe the pose of its end-effector – frame {𝐸} – by a sequence of
relative poses:
2
(2)
Planar arm with one rotational joint.

### Images

#### Image 1 (Page 28)
![Image 1 - Page 28](Attachments_Ch7/TEL200_Ch7_p028_x328_y427_w214_h47_01.png)

#### Image 2 (Page 28)
![Image 2 - Page 28](Attachments_Ch7/TEL200_Ch7_p028_x270_y162_w260_h167_02.jpeg)

---

## Page 29

2-Dimensional (Planar) Robotic Arms
• Then, the forward kinematics map is given by:
2
𝜉𝑞=
cos 𝑞1
−sin 𝑞1
𝑎1 cos 𝑞1
sin 𝑞1
 cos 𝑞1 
𝑎1 sin 𝑞1
0
0
1
(3)

---

## Page 30

2-Dimensional (Planar) Robotic Arms
• Then, the forward kinematics map is given by:
• The forward kinematics, for a particular value of 𝑞1 = 30 deg and for the case
𝑎1 = 1 m, yields:
which is an SE(2) homogeneous transformation matrix representing the pose of
the end-effector – coordinate frame {𝐸}.
2
𝜉𝑞=
cos 𝑞1
−sin 𝑞1
𝑎1 cos 𝑞1
sin 𝑞1
 cos 𝑞1 
𝑎1 sin 𝑞1
0
0
1
(3)
𝜉30 =
0.8660
−0.5000
0.8600
0.5000
 0.8660
0.5000
0
0
1
(4)

---

## Page 31

2-Dimensional (Planar) Robotic Arms
• Consider now a robot arm with two joints
• The pose of the end-effector is given by:
2
Planar arm with two rotational joints.
(5)

### Images

#### Image 1 (Page 31)
![Image 1 - Page 31](Attachments_Ch7/TEL200_Ch7_p031_x146_y181_w368_h177_01.jpeg)

#### Image 2 (Page 31)
![Image 2 - Page 31](Attachments_Ch7/TEL200_Ch7_p031_x259_y424_w403_h76_02.png)

---

## Page 32

2-Dimensional (Planar) Robotic Arms
• Then, the forward kinematics map is given by:
2
𝜉𝑞=
cos 𝑞12
−sin 𝑞12
𝑎1 cos 𝑞1 + 𝑎2 cos 𝑞12
sin 𝑞12
 cos 𝑞12 
𝑎1 sin 𝑞1 + 𝑎2 sin 𝑞12
0
0
1
𝑞12 = 𝑞1 + 𝑞2
(6)

---

## Page 33

2-Dimensional (Planar) Robotic Arms
• Then, the forward kinematics map is given by:
• The forward kinematics, for a particular value of 𝑞1 = 30 deg and 𝑞2 = 40 deg,
and for the case 𝑎1 = 𝑎2 = 1 m, yields:
which is an SE(2) homogeneous transformation matrix representing the pose of
the end-effector – coordinate frame {𝐸}.
2
𝜉𝑞=
cos 𝑞12
−sin 𝑞12
𝑎1 cos 𝑞1 + 𝑎2 cos 𝑞12
sin 𝑞12
 cos 𝑞12 
𝑎1 sin 𝑞1 + 𝑎2 sin 𝑞12
0
0
1
(6)
𝜉30, 40 =
0.3420
−0.9397
1.208
0.9397
 0.3420
1.440
0
0
1
(7)
𝑞12 = 𝑞1 + 𝑞2

---

## Page 34

2-Dimensional (Planar) Robotic Arms
• So far, we have only considered revolute joints, but we could use a prismatic
joint instead
2
Planar arm with two joints: one 
rotational and one prismatic.

### Images

#### Image 1 (Page 34)
![Image 1 - Page 34](Attachments_Ch7/TEL200_Ch7_p034_x265_y200_w254_h164_01.jpeg)

---

## Page 35

2-Dimensional (Planar) Robotic Arms
• So far, we have only considered revolute joints, but we could use a prismatic
joint instead
• The pose of the end-effector is given by:
2
Planar arm with two joints: one 
rotational and one prismatic.
(8)

### Images

#### Image 1 (Page 35)
![Image 1 - Page 35](Attachments_Ch7/TEL200_Ch7_p035_x265_y200_w254_h164_01.jpeg)

#### Image 2 (Page 35)
![Image 2 - Page 35](Attachments_Ch7/TEL200_Ch7_p035_x314_y417_w315_h78_02.png)

---

## Page 36

2-Dimensional (Planar) Robotic Arms
• We can easily add a third (revolute) joint
and use the now familiar functionality to represent and work with this arm.
where:
2
𝜉𝑞=
cos 𝑞123
−sin 𝑞123
𝑎1 cos 𝑞1 + 𝑎2 cos 𝑞12 + 𝑎3 cos 𝑞123
sin 𝑞123
 cos 𝑞123 
𝑎1 sin 𝑞1 + 𝑎2 sin 𝑞12 + 𝑎3 sin 𝑞123
0
0
1
(10)
𝑞123 = 𝑞1 + 𝑞2 + 𝑞3
(9)
(11)

### Images

#### Image 1 (Page 36)
![Image 1 - Page 36](Attachments_Ch7/TEL200_Ch7_p036_x225_y188_w539_h67_01.png)

---

## Page 37

2-Dimensional (Planar) Robotic Arms
• We can easily add a third (revolute) joint
and use the now familiar functionality to represent and work with this arm.
• This robot has 3 degrees of freedom and is able to access all points in the task
space 𝒯⊂𝐒𝐄(2), that is, achieve any pose in the plane (limited by reach).
2
(9)
𝜉𝑞=
cos 𝑞123
−sin 𝑞123
𝑎1 cos 𝑞1 + 𝑎2 cos 𝑞12 + 𝑎3 cos 𝑞123
sin 𝑞123
 cos 𝑞123 
𝑎1 sin 𝑞1 + 𝑎2 sin 𝑞12 + 𝑎3 sin 𝑞123
0
0
1
(10)

### Images

#### Image 1 (Page 37)
![Image 1 - Page 37](Attachments_Ch7/TEL200_Ch7_p037_x225_y188_w539_h67_01.png)

---

## Page 38

3-Dimensional (Planar) Robotic Arms
• Truly useful robot arms have a task space 𝒯⊂𝐒𝐄(3) enabling arbitrary
position and orientation of the end-effector.
• This requires a robot arm with a configuration space dim 𝒞≥dim 𝒯which
can be achieved by a robot with 6 (six) or more joints.
2

---

## Page 39

3-Dimensional (Planar) Robotic Arms
• We can extend the technique from the previous
section for a robot like the Puma 560
2
Puma robot in the zero joint-angle configuration showing 
dimensions and joint axes.

### Images

#### Image 1 (Page 39)
![Image 1 - Page 39](Attachments_Ch7/TEL200_Ch7_p039_x639_y41_w174_h421_01.png)

---

## Page 40

3-Dimensional (Planar) Robotic Arms
• We can extend the technique from the previous
section for a robot like the Puma 560
• Starting with the world frame {0} we move up,
rotate about the waist axis (𝑞1), rotate about
the shoulder axis (𝑞2), move to the left, move
up and so on.
2
Puma robot in the zero joint-angle configuration showing 
dimensions and joint axes.

### Images

#### Image 1 (Page 40)
![Image 1 - Page 40](Attachments_Ch7/TEL200_Ch7_p040_x639_y41_w174_h421_01.png)

---

## Page 41

3-Dimensional (Planar) Robotic Arms
• We can extend the technique from the previous
section for a robot like the Puma 560
• Starting with the world frame {0} we move up,
rotate about the waist axis (𝑞1), rotate about
the shoulder axis (𝑞2), move to the left, move
up and so on.
2
Puma robot in the zero joint-angle configuration showing 
dimensions and joint axes.

### Images

#### Image 1 (Page 41)
![Image 1 - Page 41](Attachments_Ch7/TEL200_Ch7_p041_x639_y41_w174_h421_01.png)

#### Image 2 (Page 41)
![Image 2 - Page 41](Attachments_Ch7/TEL200_Ch7_p041_x62_y360_w444_h39_02.png)

#### Image 3 (Page 41)
![Image 3 - Page 41](Attachments_Ch7/TEL200_Ch7_p041_x105_y419_w402_h51_03.jpeg)

---

## Page 42

3-Dimensional (Planar) Robotic Arms
• Then, we write down the complete transform expression:
• The wrist term represents a ZYZ Euler angle sequence and provides arbitrary 
orientation but is subject to a singularity when the middle angle q5= 0.
2
(12)

### Images

#### Image 1 (Page 42)
![Image 1 - Page 42](Attachments_Ch7/TEL200_Ch7_p042_x244_y175_w472_h87_01.png)

---

## Page 43

• The pose of the end effector is
where j𝜉j + 1 𝑞j is the pose of link frame {j+1} w.r.t. link {j}
• In functional form
FK as a chain of Robot Links
[formula text unreadable from PDF encoding; see source PDF/image]
![Unreadable formula - Page 43](Attachments_Ch7/TEL200_Ch7_p043_unreadable_formula_01.png)

### Images

#### Image 1 (Page 43)
![Image 1 - Page 43](Attachments_Ch7/TEL200_Ch7_p043_x204_y319_w553_h221_01.jpeg)

#### Image 2 (Page 43)
![Image 2 - Page 43](Attachments_Ch7/TEL200_Ch7_p043_x123_y174_w425_h47_02.jpeg)

#### Image 3 (Page 43)
![Image 3 - Page 43](Attachments_Ch7/TEL200_Ch7_p043_x140_y293_w122_h34_03.jpeg)

---

## Page 44

3-Dimensional (Planar) Robotic Arms
• While this notation is somehow intuitive, it does become cumbersome as the
number of robot joints increases.
• Several approaches have been developed to more concisely describe the
motion of a serial-link robotic arm:
– Denavit-Hartenberg (DH) notation (Ch. 7.1.5)
– Product of exponentials (not included in TEL200)
• While many other conventions have been developed, the DH convention 
remains the standard approach
2

---

## Page 45

Denavit-Hartenberg Parameters
• One systematic way of describing the geometry of a serial chain of links and
joints is Denavit-Hartenberg notation.
2
Tab. 7.2 - Denavit-Hartenberg parameters: their symbol, physical meaning, and formal definition.

### Images

#### Image 1 (Page 45)
![Image 1 - Page 45](Attachments_Ch7/TEL200_Ch7_p045_x132_y197_w655_h263_01.jpeg)

---

## Page 46

Denavit-Hartenberg Parameters
• One systematic way of describing the geometry of a serial chain of links and
joints is Denavit-Hartenberg notation.
2
Fig. 7.18: Definition of standard Denavit-Hartenberg link parameters.
•
The colors red and blue denote all things associated
with links 𝑗−1 and 𝑗respectively;
•
The numbers in circles
represent the order in which
the elementary transforms are applied;
•
The unit vector 𝑥𝑗is parallel to 𝑧𝑗−1 × 𝑧𝑗and if those
two axes are parallel then 𝑑𝑗can be chosen arbitrarily.
1

### Images

#### Image 1 (Page 46)
![Image 1 - Page 46](Attachments_Ch7/TEL200_Ch7_p046_x12_y219_w394_h283_01.jpeg)

---

## Page 47

Denavit-Hartenberg Parameters
• The transformation from link coordinate frame {𝑗} to frame {𝑗+ 1} is defined in
terms of elementary rotations and translations as
which can be expanded in homogeneous matrix form in SE(3) as
2
[formula text unreadable from PDF encoding; see source PDF/image]
![Unreadable formula - Page 47](Attachments_Ch7/TEL200_Ch7_p047_unreadable_formula_01.png)

### Images

#### Image 1 (Page 47)
![Image 1 - Page 47](Attachments_Ch7/TEL200_Ch7_p047_x89_y227_w542_h56_01.jpeg)

#### Image 2 (Page 47)
![Image 2 - Page 47](Attachments_Ch7/TEL200_Ch7_p047_x43_y370_w648_h140_02.jpeg)

---

## Page 48

Denavit-Hartenberg Parameters
• The transformation from link coordinate frame {𝑗} to frame {𝑗+ 1} is defined in
terms of elementary rotations and translations as
which can be expanded in homogeneous matrix form in SE(3) as
2
[formula text unreadable from PDF encoding; see source PDF/image]
![Unreadable formula - Page 48](Attachments_Ch7/TEL200_Ch7_p048_unreadable_formula_01.png)
Parameters 𝛼𝑗 and 𝑎𝑗 are
always constant

### Images

#### Image 1 (Page 48)
![Image 1 - Page 48](Attachments_Ch7/TEL200_Ch7_p048_x89_y227_w542_h56_01.jpeg)

#### Image 2 (Page 48)
![Image 2 - Page 48](Attachments_Ch7/TEL200_Ch7_p048_x43_y370_w648_h140_02.jpeg)

---

## Page 49

Denavit-Hartenberg Parameters
• The transformation from link coordinate frame {𝑗} to frame {𝑗+ 1} is defined in
terms of elementary rotations and translations as
which can be expanded in homogeneous matrix form in SE(3) as
2
[formula text unreadable from PDF encoding; see source PDF/image]
![Unreadable formula - Page 49](Attachments_Ch7/TEL200_Ch7_p049_unreadable_formula_01.png)
For a revolute joint, 𝜃𝑗 is the joint 
variable and 𝑑𝑗 is constant

### Images

#### Image 1 (Page 49)
![Image 1 - Page 49](Attachments_Ch7/TEL200_Ch7_p049_x89_y227_w542_h56_01.jpeg)

#### Image 2 (Page 49)
![Image 2 - Page 49](Attachments_Ch7/TEL200_Ch7_p049_x43_y370_w648_h140_02.jpeg)

---

## Page 50

Denavit-Hartenberg Parameters
• The transformation from link coordinate frame {𝑗} to frame {𝑗+ 1} is defined in
terms of elementary rotations and translations as
which can be expanded in homogeneous matrix form in SE(3) as
2
[formula text unreadable from PDF encoding; see source PDF/image]
![Unreadable formula - Page 50](Attachments_Ch7/TEL200_Ch7_p050_unreadable_formula_01.png)
For a prismatic joint, 𝑑𝑗 is the joint 
variable, 𝜃𝑗 is constant and 𝛼𝑗= 0

### Images

#### Image 1 (Page 50)
![Image 1 - Page 50](Attachments_Ch7/TEL200_Ch7_p050_x89_y227_w542_h56_01.jpeg)

#### Image 2 (Page 50)
![Image 2 - Page 50](Attachments_Ch7/TEL200_Ch7_p050_x43_y370_w648_h140_02.jpeg)

---

## Page 51

Denavit-Hartenberg Parameters
• The transformation from link coordinate frame {𝑗} to frame {𝑗+ 1} is defined in
terms of elementary rotations and translations as
which can be expanded in homogeneous matrix form in SE(3) as
2
[formula text unreadable from PDF encoding; see source PDF/image]
![Unreadable formula - Page 51](Attachments_Ch7/TEL200_Ch7_p051_unreadable_formula_01.png)

### Images

#### Image 1 (Page 51)
![Image 1 - Page 51](Attachments_Ch7/TEL200_Ch7_p051_x89_y227_w542_h56_01.jpeg)

#### Image 2 (Page 51)
![Image 2 - Page 51](Attachments_Ch7/TEL200_Ch7_p051_x43_y370_w648_h140_02.jpeg)

#### Image 3 (Page 51)
![Image 3 - Page 51](Attachments_Ch7/TEL200_Ch7_p051_x718_y451_w209_h76_03.jpeg)

---

## Page 52

• Using DH convention, a link is specified by only four parameters while it in general 
would entail six parameters; three each for translation and rotation.
• This is due to the two imposed constraints: 
1.
Position: 
axis xj intersects zj−1
2.
Orientation: 
axis xj is perpendicular to zj−1.
Consequently:
1.
The link coordinate frame might be outside the actual, physical robot links!
2.
The robot must have a particular “zero-angle configuration” that might even be 
mechanically unachievable!
Denavit-Hartenberg Parameters

---

## Page 53

• Useful to add tools and base by extending FK expression of (7.1) with 2 transforms:
FK as a chain of Robot Links
[formula text unreadable from PDF encoding; see source PDF/image]
![Unreadable formula - Page 53](Attachments_Ch7/TEL200_Ch7_p053_unreadable_formula_01.png)

### Images

#### Image 1 (Page 53)
![Image 1 - Page 53](Attachments_Ch7/TEL200_Ch7_p053_x77_y270_w807_h184_01.jpeg)

#### Image 2 (Page 53)
![Image 2 - Page 53](Attachments_Ch7/TEL200_Ch7_p053_x128_y202_w490_h74_02.jpeg)

---

## Page 54

• Useful to add tools and base by extending FK expression of (7.1) with 2 transforms:
FK as a chain of Robot Links
[formula text unreadable from PDF encoding; see source PDF/image]
![Unreadable formula - Page 54](Attachments_Ch7/TEL200_Ch7_p054_unreadable_formula_01.png)
The base transform 𝜉𝐵puts the base of the robot arm at an arbitrary pose
within the world coordinate frame.

### Images

#### Image 1 (Page 54)
![Image 1 - Page 54](Attachments_Ch7/TEL200_Ch7_p054_x77_y270_w807_h184_01.jpeg)

#### Image 2 (Page 54)
![Image 2 - Page 54](Attachments_Ch7/TEL200_Ch7_p054_x128_y202_w490_h74_02.jpeg)

---

## Page 55

• Useful to add tools and base by extending FK expression of (7.1) with 2 transforms:
FK as a chain of Robot Links
[formula text unreadable from PDF encoding; see source PDF/image]
![Unreadable formula - Page 55](Attachments_Ch7/TEL200_Ch7_p055_unreadable_formula_01.png)
The frame {𝑁} is often defined as the center of the spherical wrist mechanism,
and the tool transform 𝜉𝑇describes the pose of the tool tip with respect to that.

### Images

#### Image 1 (Page 55)
![Image 1 - Page 55](Attachments_Ch7/TEL200_Ch7_p055_x77_y270_w807_h184_01.jpeg)

#### Image 2 (Page 55)
![Image 2 - Page 55](Attachments_Ch7/TEL200_Ch7_p055_x128_y202_w490_h74_02.jpeg)

---

## Page 56

Robotics Toolbox - Denavit-Hartenberg (DH) 
• Determining the DH parameters for a particular robot is challenging, but the 
Robotics Toolbox includes many robot models
2

### Images

#### Image 1 (Page 56)
![Image 1 - Page 56](Attachments_Ch7/TEL200_Ch7_p056_x174_y213_w533_h317_01.jpeg)

---

## Page 57

Robotics Toolbox - Denavit-Hartenberg (DH) 
$$
>>> irb140 = models.DH.IRB140();
$$
ready
zero
dextrous/normal

### Images

#### Image 1 (Page 57)
![Image 1 - Page 57](Attachments_Ch7/TEL200_Ch7_p057_x582_y178_w376_h376_01.jpeg)

#### Image 2 (Page 57)
![Image 2 - Page 57](Attachments_Ch7/TEL200_Ch7_p057_x47_y174_w573_h350_02.jpeg)

---

## Page 58

Denavit-Hartenberg convention – Ethan Tira-Thompson
• Video clip:
2

### Images

#### Image 1 (Page 58)
![Image 1 - Page 58](Attachments_Ch7/TEL200_Ch7_p058_x196_y145_w640_h360_01.jpeg)

---

## Page 59

Denavit-Hartenberg convention – Peter Corke
NB. RVC Ed 3 (Python) uses Zero-Based Indexing, i.e. indexed starting from zero (video uses 1-based, form Ed 2)

### Images

#### Image 1 (Page 59)
![Image 1 - Page 59](Attachments_Ch7/TEL200_Ch7_p059_x179_y142_w602_h340_01.jpeg)

---

## Page 60

Inverse Kinematics (Ch. 7.2)
• We have shown how to determine the pose of the end-effector given the joint
coordinates and optional tool and base transforms.
• Another problem with real practical interest is the inverse problem:
2
Given the desired pose of the end-effector  𝝃𝑬what 
are the required joint coordinates 𝒒?
Forward kinematics
Backward kinematics
q
𝝃E

---

## Page 61

Inverse Kinematics
• This is called the inverse kinematics problem which is written in functional
form as
and, in general, this function is not unique, that is, several joint coordinate
vectors 𝒒may result in the same end-effector pose 𝜉E;
2
𝜉
𝑞
𝒦(⋅)
[formula text unreadable from PDF encoding; see source PDF/image]
![Unreadable formula - Page 61](Attachments_Ch7/TEL200_Ch7_p061_unreadable_formula_01.png)

### Images

#### Image 1 (Page 61)
![Image 1 - Page 61](Attachments_Ch7/TEL200_Ch7_p061_x88_y336_w24_h171_01.png)

#### Image 2 (Page 61)
![Image 2 - Page 61](Attachments_Ch7/TEL200_Ch7_p061_x74_y472_w298_h24_02.png)

#### Image 3 (Page 61)
![Image 3 - Page 61](Attachments_Ch7/TEL200_Ch7_p061_x124_y369_w212_h106_03.png)

#### Image 4 (Page 61)
![Image 4 - Page 61](Attachments_Ch7/TEL200_Ch7_p061_x355_y192_w170_h46_04.jpeg)

---

## Page 62

Inverse Kinematics
• This is called the inverse kinematics problem which is written in functional
form as
and, in general, this function is not unique, that is, several joint coordinate
vectors 𝒒may result in the same end-effector pose 𝜉E;
2
[formula text unreadable from PDF encoding; see source PDF/image]
![Unreadable formula - Page 62](Attachments_Ch7/TEL200_Ch7_p062_unreadable_formula_01.png)
𝜉1 = 𝒦(𝑞1)
𝜉2 = 𝒦(𝑞2)
𝜉3 = 𝒦(𝑞3)
𝑞
𝜉1
𝜉2
𝑞1
𝑞2
𝑞3
𝜉3
𝜉
𝒦(⋅)

### Images

#### Image 1 (Page 62)
![Image 1 - Page 62](Attachments_Ch7/TEL200_Ch7_p062_x355_y192_w170_h46_01.jpeg)

#### Image 2 (Page 62)
![Image 2 - Page 62](Attachments_Ch7/TEL200_Ch7_p062_x74_y472_w298_h24_02.png)

#### Image 3 (Page 62)
![Image 3 - Page 62](Attachments_Ch7/TEL200_Ch7_p062_x124_y370_w212_h106_03.png)

#### Image 4 (Page 62)
![Image 4 - Page 62](Attachments_Ch7/TEL200_Ch7_p062_x91_y450_w171_h18_04.png)

#### Image 5 (Page 62)
![Image 5 - Page 62](Attachments_Ch7/TEL200_Ch7_p062_x255_y454_w8_h32_05.png)

#### Image 6 (Page 62)
![Image 6 - Page 62](Attachments_Ch7/TEL200_Ch7_p062_x90_y427_w242_h18_06.png)

#### Image 7 (Page 62)
![Image 7 - Page 62](Attachments_Ch7/TEL200_Ch7_p062_x324_y432_w8_h55_07.png)

#### Image 8 (Page 62)
![Image 8 - Page 62](Attachments_Ch7/TEL200_Ch7_p062_x91_y384_w96_h18_08.png)

#### Image 9 (Page 62)
![Image 9 - Page 62](Attachments_Ch7/TEL200_Ch7_p062_x180_y389_w8_h98_09.png)

#### Image 10 (Page 62)
![Image 10 - Page 62](Attachments_Ch7/TEL200_Ch7_p062_x88_y336_w24_h171_10.png)

---

## Page 63

Inverse Kinematics
• This is called the inverse kinematics problem which is written in functional
form as
and, in general, this function is not unique, that is, several joint coordinate
vectors 𝒒may result in the same end-effector pose 𝜉E;
2
[formula text unreadable from PDF encoding; see source PDF/image]
![Unreadable formula - Page 63](Attachments_Ch7/TEL200_Ch7_p063_unreadable_formula_01.png)
𝑞1 = 𝒦−1(𝜉1)
𝑞2 = 𝒦−1(𝜉2)
𝑞3 = 𝒦−1(𝜉3)
𝑞
𝜉1
𝜉2
𝑞1
𝑞2
𝑞3
𝜉3
𝜉
𝒦(⋅)

### Images

#### Image 1 (Page 63)
![Image 1 - Page 63](Attachments_Ch7/TEL200_Ch7_p063_x355_y192_w170_h46_01.jpeg)

#### Image 2 (Page 63)
![Image 2 - Page 63](Attachments_Ch7/TEL200_Ch7_p063_x74_y472_w298_h24_02.png)

#### Image 3 (Page 63)
![Image 3 - Page 63](Attachments_Ch7/TEL200_Ch7_p063_x124_y370_w212_h106_03.png)

#### Image 4 (Page 63)
![Image 4 - Page 63](Attachments_Ch7/TEL200_Ch7_p063_x96_y455_w166_h8_04.png)

#### Image 5 (Page 63)
![Image 5 - Page 63](Attachments_Ch7/TEL200_Ch7_p063_x249_y454_w18_h39_05.png)

#### Image 6 (Page 63)
![Image 6 - Page 63](Attachments_Ch7/TEL200_Ch7_p063_x95_y431_w237_h8_06.png)

#### Image 7 (Page 63)
![Image 7 - Page 63](Attachments_Ch7/TEL200_Ch7_p063_x96_y389_w91_h8_07.png)

#### Image 8 (Page 63)
![Image 8 - Page 63](Attachments_Ch7/TEL200_Ch7_p063_x319_y431_w18_h61_08.png)

#### Image 9 (Page 63)
![Image 9 - Page 63](Attachments_Ch7/TEL200_Ch7_p063_x174_y388_w18_h104_09.png)

#### Image 10 (Page 63)
![Image 10 - Page 63](Attachments_Ch7/TEL200_Ch7_p063_x88_y336_w24_h171_10.png)

---

## Page 64

Inverse Kinematics
• This is called the inverse kinematics problem which is written in functional
form as
and, in general, this function is not unique, that is, several joint coordinate
vectors 𝒒may result in the same end-effector pose 𝜉E;
2
[formula text unreadable from PDF encoding; see source PDF/image]
![Unreadable formula - Page 64](Attachments_Ch7/TEL200_Ch7_p064_unreadable_formula_01.png)
Multiple solutions !
𝑞
𝜉1
𝜉2
𝑞1
𝑞2
𝑞3
𝜉3
𝜉
𝒦(⋅)
𝑞1 = 𝒦−1(𝜉1)
𝑞2 = 𝒦−1(𝜉2)
𝑞3 = 𝒦−1(𝜉3)

### Images

#### Image 1 (Page 64)
![Image 1 - Page 64](Attachments_Ch7/TEL200_Ch7_p064_x355_y192_w170_h46_01.jpeg)

#### Image 2 (Page 64)
![Image 2 - Page 64](Attachments_Ch7/TEL200_Ch7_p064_x124_y370_w212_h106_02.png)

#### Image 3 (Page 64)
![Image 3 - Page 64](Attachments_Ch7/TEL200_Ch7_p064_x74_y472_w298_h24_03.png)

#### Image 4 (Page 64)
![Image 4 - Page 64](Attachments_Ch7/TEL200_Ch7_p064_x175_y389_w18_h104_04.png)

#### Image 5 (Page 64)
![Image 5 - Page 64](Attachments_Ch7/TEL200_Ch7_p064_x96_y455_w166_h8_05.png)

#### Image 6 (Page 64)
![Image 6 - Page 64](Attachments_Ch7/TEL200_Ch7_p064_x249_y454_w18_h38_06.png)

#### Image 7 (Page 64)
![Image 7 - Page 64](Attachments_Ch7/TEL200_Ch7_p064_x95_y432_w237_h8_07.png)

#### Image 8 (Page 64)
![Image 8 - Page 64](Attachments_Ch7/TEL200_Ch7_p064_x319_y432_w18_h61_08.png)

#### Image 9 (Page 64)
![Image 9 - Page 64](Attachments_Ch7/TEL200_Ch7_p064_x96_y389_w91_h8_09.png)

#### Image 10 (Page 64)
![Image 10 - Page 64](Attachments_Ch7/TEL200_Ch7_p064_x180_y389_w120_h8_10.png)

#### Image 11 (Page 64)
![Image 11 - Page 64](Attachments_Ch7/TEL200_Ch7_p064_x319_y432_w18_h61_11.png)

#### Image 12 (Page 64)
![Image 12 - Page 64](Attachments_Ch7/TEL200_Ch7_p064_x319_y432_w18_h61_12.png)

#### Image 13 (Page 64)
![Image 13 - Page 64](Attachments_Ch7/TEL200_Ch7_p064_x192_y454_w18_h38_13.png)

#### Image 14 (Page 64)
![Image 14 - Page 64](Attachments_Ch7/TEL200_Ch7_p064_x249_y454_w18_h38_14.png)

#### Image 15 (Page 64)
![Image 15 - Page 64](Attachments_Ch7/TEL200_Ch7_p064_x175_y389_w18_h104_15.png)

#### Image 16 (Page 64)
![Image 16 - Page 64](Attachments_Ch7/TEL200_Ch7_p064_x309_y388_w18_h104_16.png)

#### Image 17 (Page 64)
![Image 17 - Page 64](Attachments_Ch7/TEL200_Ch7_p064_x292_y389_w30_h8_17.png)

#### Image 18 (Page 64)
![Image 18 - Page 64](Attachments_Ch7/TEL200_Ch7_p064_x88_y336_w24_h171_18.png)

---

## Page 65

Inverse Kinematics
• This is called the inverse kinematics problem which is written in functional
form as
and, in general, this function is not unique, that is, several joint coordinate
vectors 𝒒may result in the same end-effector pose 𝜉E;
2
[formula text unreadable from PDF encoding; see source PDF/image]
![Unreadable formula - Page 65](Attachments_Ch7/TEL200_Ch7_p065_unreadable_formula_01.png)
𝜉
𝑞
𝒦(⋅)
𝜉4
𝜉5
No solutions !
𝑞4 = 𝒦−1(𝜉4)
𝑞5 = 𝒦−1(𝜉5)

### Images

#### Image 1 (Page 65)
![Image 1 - Page 65](Attachments_Ch7/TEL200_Ch7_p065_x355_y192_w170_h46_01.jpeg)

#### Image 2 (Page 65)
![Image 2 - Page 65](Attachments_Ch7/TEL200_Ch7_p065_x88_y336_w24_h171_02.png)

#### Image 3 (Page 65)
![Image 3 - Page 65](Attachments_Ch7/TEL200_Ch7_p065_x74_y472_w298_h24_03.png)

#### Image 4 (Page 65)
![Image 4 - Page 65](Attachments_Ch7/TEL200_Ch7_p065_x124_y369_w212_h106_04.png)

#### Image 5 (Page 65)
![Image 5 - Page 65](Attachments_Ch7/TEL200_Ch7_p065_x96_y364_w91_h8_05.png)

#### Image 6 (Page 65)
![Image 6 - Page 65](Attachments_Ch7/TEL200_Ch7_p065_x96_y471_w166_h8_06.png)

---

## Page 66

Inverse Kinematics
• Two main approaches can be used to determine the inverse kinematics.
1.
Closed form or analytical solution;
2.
An iterative numerical solution.
2

---

## Page 67

2-Dimensional (Planar) Robotic Arms
• We will illustrate inverse kinematics for a simple 2-joint robot two ways:
1.
Algebraic closed-form
2.
Numerical solution
2
Given the desired pose of the end-effector  𝝃𝑬what are the required joint coordinates 𝒒0 and 𝒒1?
Planar arm with two rotational joints.

### Images

#### Image 1 (Page 67)
![Image 1 - Page 67](Attachments_Ch7/TEL200_Ch7_p067_x384_y190_w322_h190_01.jpeg)

---

## Page 68

Closed-form Solution
• Compute the forward kinematics symbolically using SymPy:
>>> import sympy
$$
>>> a1, a2 = sympy.symbols("a1 a2")
$$
$$
>>> e = ET2.R() * ET2.tx(a1) * ET2.R() * ET2.tx(a2);
$$
• Define symbolic variables for the joint angles
$$
>>> q0, q1 = sympy.symbols("q0 q1")
$$
• Compute forward kinematics as an SE(2) matrix
$$
>>> TE = e.fkine([q0, q1])
$$
cos(q0 + q1) -sin(q0 + q1) a1*cos(q0) + a2*cos(q0 + q1)
sin(q0 + q1) cos(q0 + q1) a1*sin(q0) + a2*sin(q0 + q1)
0 0 1
2

### Images

#### Image 1 (Page 68)
![Image 1 - Page 68](Attachments_Ch7/TEL200_Ch7_p068_x37_y173_w128_h45_01.png)

#### Image 2 (Page 68)
![Image 2 - Page 68](Attachments_Ch7/TEL200_Ch7_p068_x138_y173_w84_h45_02.png)

#### Image 3 (Page 68)
![Image 3 - Page 68](Attachments_Ch7/TEL200_Ch7_p068_x37_y209_w145_h45_03.png)

#### Image 4 (Page 68)
![Image 4 - Page 68](Attachments_Ch7/TEL200_Ch7_p068_x155_y209_w162_h45_04.png)

#### Image 5 (Page 68)
![Image 5 - Page 68](Attachments_Ch7/TEL200_Ch7_p068_x291_y209_w104_h45_05.png)

#### Image 6 (Page 68)
![Image 6 - Page 68](Attachments_Ch7/TEL200_Ch7_p068_x37_y245_w489_h45_06.png)

#### Image 7 (Page 68)
![Image 7 - Page 68](Attachments_Ch7/TEL200_Ch7_p068_x37_y209_w145_h45_07.png)

#### Image 8 (Page 68)
![Image 8 - Page 68](Attachments_Ch7/TEL200_Ch7_p068_x155_y319_w89_h45_08.png)

#### Image 9 (Page 68)
![Image 9 - Page 68](Attachments_Ch7/TEL200_Ch7_p068_x217_y319_w100_h45_09.png)

#### Image 10 (Page 68)
![Image 10 - Page 68](Attachments_Ch7/TEL200_Ch7_p068_x291_y319_w68_h45_10.png)

#### Image 11 (Page 68)
![Image 11 - Page 68](Attachments_Ch7/TEL200_Ch7_p068_x332_y319_w63_h45_11.png)

#### Image 12 (Page 68)
![Image 12 - Page 68](Attachments_Ch7/TEL200_Ch7_p068_x37_y394_w115_h45_12.png)

#### Image 13 (Page 68)
![Image 13 - Page 68](Attachments_Ch7/TEL200_Ch7_p068_x125_y394_w86_h45_13.png)

#### Image 14 (Page 68)
![Image 14 - Page 68](Attachments_Ch7/TEL200_Ch7_p068_x184_y394_w106_h45_14.png)

#### Image 15 (Page 68)
![Image 15 - Page 68](Attachments_Ch7/TEL200_Ch7_p068_x37_y430_w144_h45_15.png)

#### Image 16 (Page 68)
![Image 16 - Page 68](Attachments_Ch7/TEL200_Ch7_p068_x154_y430_w33_h45_16.png)

#### Image 17 (Page 68)
![Image 17 - Page 68](Attachments_Ch7/TEL200_Ch7_p068_x160_y430_w399_h45_17.png)

#### Image 18 (Page 68)
![Image 18 - Page 68](Attachments_Ch7/TEL200_Ch7_p068_x37_y466_w505_h45_18.png)

#### Image 19 (Page 68)
![Image 19 - Page 68](Attachments_Ch7/TEL200_Ch7_p068_x37_y502_w71_h39_19.png)

---

## Page 69

Closed-form Solution
• Consider just the end-effector position
>>> x_fk, y_fk = TE.t; 
• Define desired end-effector position using two more symbolic variables
$$
>>> x, y = sympy.symbols("x y")
$$
• Two equations and two unknowns: 
• SymPy need some help to solve such trigonometric equations
• Square both equations and add them: 
• Rewrite as:
2

### Images

#### Image 1 (Page 69)
![Image 1 - Page 69](Attachments_Ch7/TEL200_Ch7_p069_x37_y175_w67_h45_01.png)

#### Image 2 (Page 69)
![Image 2 - Page 69](Attachments_Ch7/TEL200_Ch7_p069_x77_y175_w63_h45_02.png)

#### Image 3 (Page 69)
![Image 3 - Page 69](Attachments_Ch7/TEL200_Ch7_p069_x114_y175_w37_h45_03.png)

#### Image 4 (Page 69)
![Image 4 - Page 69](Attachments_Ch7/TEL200_Ch7_p069_x77_y175_w63_h45_04.png)

#### Image 5 (Page 69)
![Image 5 - Page 69](Attachments_Ch7/TEL200_Ch7_p069_x167_y175_w91_h45_05.png)

#### Image 6 (Page 69)
![Image 6 - Page 69](Attachments_Ch7/TEL200_Ch7_p069_x37_y255_w120_h45_06.png)

#### Image 7 (Page 69)
![Image 7 - Page 69](Attachments_Ch7/TEL200_Ch7_p069_x131_y255_w162_h45_07.png)

#### Image 8 (Page 69)
![Image 8 - Page 69](Attachments_Ch7/TEL200_Ch7_p069_x266_y255_w79_h45_08.png)

#### Image 9 (Page 69)
![Image 9 - Page 69](Attachments_Ch7/TEL200_Ch7_p069_x520_y298_w208_h28_09.jpeg)

#### Image 10 (Page 69)
![Image 10 - Page 69](Attachments_Ch7/TEL200_Ch7_p069_x520_y380_w208_h35_10.jpeg)

#### Image 11 (Page 69)
![Image 11 - Page 69](Attachments_Ch7/TEL200_Ch7_p069_x525_y423_w253_h35_11.jpeg)

---

## Page 70

Closed-form Solution – Solving for q1
$$
>>> eq1 = (x_fk**2 + y_fk**2 - x**2 - y**2).trigsimp()
$$
a1**2 + 2*a1*a2*cos(q1) + a2**2 - x**2 - y**2
$$
>>> q1_sol = sympy.solve(eq1, q1)
$$
[-acos(-(a1**2 + a2**2 - x**2 - y**2)/(2*a1*a2)) + 2*pi,
acos((-a1**2 - a2**2 + x**2 + y**2)/(2*a1*a2)) 
• Eq1 has only one unknown, q1 
• q1_sol is a list. 
• Correspont to pos or neg q1
• IK problem does not have 
unique solution

### Images

#### Image 1 (Page 70)
![Image 1 - Page 70](Attachments_Ch7/TEL200_Ch7_p070_x43_y137_w67_h45_01.png)

#### Image 2 (Page 70)
![Image 2 - Page 70](Attachments_Ch7/TEL200_Ch7_p070_x84_y137_w89_h45_02.png)

#### Image 3 (Page 70)
![Image 3 - Page 70](Attachments_Ch7/TEL200_Ch7_p070_x146_y137_w63_h45_03.png)

#### Image 4 (Page 70)
![Image 4 - Page 70](Attachments_Ch7/TEL200_Ch7_p070_x183_y137_w75_h45_04.png)

#### Image 5 (Page 70)
![Image 5 - Page 70](Attachments_Ch7/TEL200_Ch7_p070_x146_y137_w63_h45_05.png)

#### Image 6 (Page 70)
![Image 6 - Page 70](Attachments_Ch7/TEL200_Ch7_p070_x269_y137_w58_h45_06.png)

#### Image 7 (Page 70)
![Image 7 - Page 70](Attachments_Ch7/TEL200_Ch7_p070_x301_y137_w33_h45_07.png)

#### Image 8 (Page 70)
![Image 8 - Page 70](Attachments_Ch7/TEL200_Ch7_p070_x313_y137_w69_h45_08.png)

#### Image 9 (Page 70)
![Image 9 - Page 70](Attachments_Ch7/TEL200_Ch7_p070_x183_y137_w75_h45_09.png)

#### Image 10 (Page 70)
![Image 10 - Page 70](Attachments_Ch7/TEL200_Ch7_p070_x416_y137_w96_h45_10.png)

#### Image 11 (Page 70)
![Image 11 - Page 70](Attachments_Ch7/TEL200_Ch7_p070_x486_y137_w45_h45_11.png)

#### Image 12 (Page 70)
![Image 12 - Page 70](Attachments_Ch7/TEL200_Ch7_p070_x43_y177_w321_h45_12.png)

#### Image 13 (Page 70)
![Image 13 - Page 70](Attachments_Ch7/TEL200_Ch7_p070_x146_y137_w63_h45_13.png)

#### Image 14 (Page 70)
![Image 14 - Page 70](Attachments_Ch7/TEL200_Ch7_p070_x43_y217_w149_h45_14.png)

#### Image 15 (Page 70)
![Image 15 - Page 70](Attachments_Ch7/TEL200_Ch7_p070_x165_y217_w135_h45_15.png)

#### Image 16 (Page 70)
![Image 16 - Page 70](Attachments_Ch7/TEL200_Ch7_p070_x274_y217_w106_h45_16.png)

#### Image 17 (Page 70)
![Image 17 - Page 70](Attachments_Ch7/TEL200_Ch7_p070_x43_y257_w32_h45_17.png)

#### Image 18 (Page 70)
![Image 18 - Page 70](Attachments_Ch7/TEL200_Ch7_p070_x313_y137_w69_h45_18.png)

#### Image 19 (Page 70)
![Image 19 - Page 70](Attachments_Ch7/TEL200_Ch7_p070_x301_y137_w33_h45_19.png)

#### Image 20 (Page 70)
![Image 20 - Page 70](Attachments_Ch7/TEL200_Ch7_p070_x111_y257_w159_h45_20.png)

#### Image 21 (Page 70)
![Image 21 - Page 70](Attachments_Ch7/TEL200_Ch7_p070_x310_y257_w229_h45_21.png)

#### Image 22 (Page 70)
![Image 22 - Page 70](Attachments_Ch7/TEL200_Ch7_p070_x86_y297_w40_h45_22.png)

#### Image 23 (Page 70)
![Image 23 - Page 70](Attachments_Ch7/TEL200_Ch7_p070_x106_y297_w81_h45_23.png)

#### Image 24 (Page 70)
![Image 24 - Page 70](Attachments_Ch7/TEL200_Ch7_p070_x172_y297_w303_h45_24.png)

---

## Page 71

Closed-form Solution – Solving for q0
• For q0, we first expand the two equations:
$$
>>> eq0 = tuple(map(sympy.expand_trig, [x_fk - x, y_fk - y]))
$$
(a1*cos(q0) + a2*(-sin(q0)*sin(q1) + cos(q0)*cos(q1)) - x,
a1*sin(q0) + a2*(sin(q0)*cos(q1) + sin(q1)*cos(q0)) - y)
• solve them for sin(q0) and cos(q0)
$$
>>> q0_sol = sympy.solve(eq0, [sympy.sin(q0), sympy.cos(q0)]);
$$
• The ratio of these is tan(q0)
>>> sympy.atan2(q0_sol[sympy.sin(q0)], q0_sol[sympy.cos(q0)]).simplify()
atan2((a1*y - a2*x*sin(q1) + a2*y*cos(q1))/(a1**2 + 2*a1*a2*cos(q1)+ a2**2),
(a1*x + a2*x*cos(q1) + a2*y*sin(q1))/(a1**2 + 2*a1*a2*cos(q1)+ a2**2))

### Images

#### Image 1 (Page 71)
![Image 1 - Page 71](Attachments_Ch7/TEL200_Ch7_p071_x45_y182_w99_h36_01.png)

#### Image 2 (Page 71)
![Image 2 - Page 71](Attachments_Ch7/TEL200_Ch7_p071_x122_y182_w56_h36_02.png)

#### Image 3 (Page 71)
![Image 3 - Page 71](Attachments_Ch7/TEL200_Ch7_p071_x157_y182_w27_h36_03.png)

#### Image 4 (Page 71)
![Image 4 - Page 71](Attachments_Ch7/TEL200_Ch7_p071_x162_y182_w52_h36_04.png)

#### Image 5 (Page 71)
![Image 5 - Page 71](Attachments_Ch7/TEL200_Ch7_p071_x198_y182_w132_h36_05.png)

#### Image 6 (Page 71)
![Image 6 - Page 71](Attachments_Ch7/TEL200_Ch7_p071_x309_y182_w43_h36_06.png)

#### Image 7 (Page 71)
![Image 7 - Page 71](Attachments_Ch7/TEL200_Ch7_p071_x331_y182_w31_h36_07.png)

#### Image 8 (Page 71)
![Image 8 - Page 71](Attachments_Ch7/TEL200_Ch7_p071_x341_y182_w26_h36_08.png)

#### Image 9 (Page 71)
![Image 9 - Page 71](Attachments_Ch7/TEL200_Ch7_p071_x345_y182_w38_h36_09.png)

#### Image 10 (Page 71)
![Image 10 - Page 71](Attachments_Ch7/TEL200_Ch7_p071_x362_y182_w34_h36_10.png)

#### Image 11 (Page 71)
![Image 11 - Page 71](Attachments_Ch7/TEL200_Ch7_p071_x157_y182_w27_h36_11.png)

#### Image 12 (Page 71)
![Image 12 - Page 71](Attachments_Ch7/TEL200_Ch7_p071_x389_y182_w38_h36_12.png)

#### Image 13 (Page 71)
![Image 13 - Page 71](Attachments_Ch7/TEL200_Ch7_p071_x345_y182_w38_h36_13.png)

#### Image 14 (Page 71)
![Image 14 - Page 71](Attachments_Ch7/TEL200_Ch7_p071_x450_y182_w44_h36_14.png)

#### Image 15 (Page 71)
![Image 15 - Page 71](Attachments_Ch7/TEL200_Ch7_p071_x45_y222_w152_h36_15.png)

#### Image 16 (Page 71)
![Image 16 - Page 71](Attachments_Ch7/TEL200_Ch7_p071_x180_y222_w267_h36_16.png)

#### Image 17 (Page 71)
![Image 17 - Page 71](Attachments_Ch7/TEL200_Ch7_p071_x362_y182_w34_h36_17.png)

#### Image 18 (Page 71)
![Image 18 - Page 71](Attachments_Ch7/TEL200_Ch7_p071_x45_y262_w387_h36_18.png)

#### Image 19 (Page 71)
![Image 19 - Page 71](Attachments_Ch7/TEL200_Ch7_p071_x420_y262_w29_h36_19.png)

#### Image 20 (Page 71)
![Image 20 - Page 71](Attachments_Ch7/TEL200_Ch7_p071_x157_y182_w27_h36_20.png)

#### Image 21 (Page 71)
![Image 21 - Page 71](Attachments_Ch7/TEL200_Ch7_p071_x45_y342_w119_h36_21.png)

#### Image 22 (Page 71)
![Image 22 - Page 71](Attachments_Ch7/TEL200_Ch7_p071_x143_y342_w108_h36_22.png)

#### Image 23 (Page 71)
![Image 23 - Page 71](Attachments_Ch7/TEL200_Ch7_p071_x230_y342_w67_h36_23.png)

#### Image 24 (Page 71)
![Image 24 - Page 71](Attachments_Ch7/TEL200_Ch7_p071_x275_y342_w91_h36_24.png)

#### Image 25 (Page 71)
![Image 25 - Page 71](Attachments_Ch7/TEL200_Ch7_p071_x450_y182_w44_h36_25.png)

#### Image 26 (Page 71)
![Image 26 - Page 71](Attachments_Ch7/TEL200_Ch7_p071_x368_y342_w36_h36_26.png)

#### Image 27 (Page 71)
![Image 27 - Page 71](Attachments_Ch7/TEL200_Ch7_p071_x230_y342_w67_h36_27.png)

#### Image 28 (Page 71)
![Image 28 - Page 71](Attachments_Ch7/TEL200_Ch7_p071_x427_y342_w51_h36_28.png)

#### Image 29 (Page 71)
![Image 29 - Page 71](Attachments_Ch7/TEL200_Ch7_p071_x480_y342_w41_h36_29.png)

#### Image 30 (Page 71)
![Image 30 - Page 71](Attachments_Ch7/TEL200_Ch7_p071_x45_y422_w196_h36_30.png)

#### Image 31 (Page 71)
![Image 31 - Page 71](Attachments_Ch7/TEL200_Ch7_p071_x224_y422_w71_h36_31.png)

#### Image 32 (Page 71)
![Image 32 - Page 71](Attachments_Ch7/TEL200_Ch7_p071_x273_y422_w42_h36_32.png)

#### Image 33 (Page 71)
![Image 33 - Page 71](Attachments_Ch7/TEL200_Ch7_p071_x368_y342_w36_h36_33.png)

#### Image 34 (Page 71)
![Image 34 - Page 71](Attachments_Ch7/TEL200_Ch7_p071_x336_y422_w73_h36_34.png)

#### Image 35 (Page 71)
![Image 35 - Page 71](Attachments_Ch7/TEL200_Ch7_p071_x387_y422_w96_h36_35.png)

#### Image 36 (Page 71)
![Image 36 - Page 71](Attachments_Ch7/TEL200_Ch7_p071_x462_y422_w64_h36_36.png)

#### Image 37 (Page 71)
![Image 37 - Page 71](Attachments_Ch7/TEL200_Ch7_p071_x504_y422_w75_h36_37.png)

#### Image 38 (Page 71)
![Image 38 - Page 71](Attachments_Ch7/TEL200_Ch7_p071_x558_y422_w32_h36_38.png)

#### Image 39 (Page 71)
![Image 39 - Page 71](Attachments_Ch7/TEL200_Ch7_p071_x45_y462_w61_h36_39.png)

#### Image 40 (Page 71)
![Image 40 - Page 71](Attachments_Ch7/TEL200_Ch7_p071_x85_y462_w69_h36_40.png)

#### Image 41 (Page 71)
![Image 41 - Page 71](Attachments_Ch7/TEL200_Ch7_p071_x389_y182_w38_h36_41.png)

#### Image 42 (Page 71)
![Image 42 - Page 71](Attachments_Ch7/TEL200_Ch7_p071_x368_y342_w36_h36_42.png)

#### Image 43 (Page 71)
![Image 43 - Page 71](Attachments_Ch7/TEL200_Ch7_p071_x174_y462_w370_h36_43.png)

#### Image 44 (Page 71)
![Image 44 - Page 71](Attachments_Ch7/TEL200_Ch7_p071_x480_y342_w41_h36_44.png)

#### Image 45 (Page 71)
![Image 45 - Page 71](Attachments_Ch7/TEL200_Ch7_p071_x542_y462_w70_h36_45.png)

#### Image 46 (Page 71)
![Image 46 - Page 71](Attachments_Ch7/TEL200_Ch7_p071_x45_y502_w477_h36_46.png)

#### Image 47 (Page 71)
![Image 47 - Page 71](Attachments_Ch7/TEL200_Ch7_p071_x224_y422_w71_h36_47.png)

---

## Page 72

Closed-form Solution
• SymPy is comparatively weak at trigonometric expressions
• More powerful symbolic tools exist
• The complexity of algebraic solution increases dramatically with the number
of joints and more sophisticated symbolic solution approaches need to be
used.
• In Matlab, the ikine_sym method generates symbolic inverse kinematics
solutions for a limited class of robot manipulators.
2

---

## Page 73

Numerical Solution
• We can think of the inverse kinematics problem as one of adjusting the joint
coordinates 𝑞until the forward kinematics 𝒦matches the desired pose p;
• More formally this is an optimization problem – to minimize the error
between the forward kinematic solution and the desired pose 𝜉∗;
• For our simple 2-link robot arm example the error function comprises only the
error in the end-effector position, not its orientation:
2
(23)
(24)

### Images

#### Image 1 (Page 73)
![Image 1 - Page 73](Attachments_Ch7/TEL200_Ch7_p073_x378_y395_w242_h62_01.png)

#### Image 2 (Page 73)
![Image 2 - Page 73](Attachments_Ch7/TEL200_Ch7_p073_x388_y281_w222_h49_02.png)

---

## Page 74

Numerical Solution
• We can solve this problem using the SciPy multi-variable minimization function
$$
>>> pstar = np.array([0.6, 0.7]); # desired position
$$
$$
>>> E = lambda q: np.linalg.norm(e.fkine(q).t - pstar); #define error
$$
$$
>>> sol = optimize.minimize(E, [0, 0]);
$$
>>> sol.x
array([ -0.2295, 2.183])
where the first argument is the error function, that incorporates the desired
end-effector position;
…and the second argument is the initial guess the joint coordinates.
2

### Images

#### Image 1 (Page 74)
![Image 1 - Page 74](Attachments_Ch7/TEL200_Ch7_p074_x37_y174_w61_h44_01.png)

#### Image 2 (Page 74)
![Image 2 - Page 74](Attachments_Ch7/TEL200_Ch7_p074_x71_y174_w68_h44_02.png)

#### Image 3 (Page 74)
![Image 3 - Page 74](Attachments_Ch7/TEL200_Ch7_p074_x117_y174_w41_h44_03.png)

#### Image 4 (Page 74)
![Image 4 - Page 74](Attachments_Ch7/TEL200_Ch7_p074_x132_y174_w95_h44_04.png)

#### Image 5 (Page 74)
![Image 5 - Page 74](Attachments_Ch7/TEL200_Ch7_p074_x201_y174_w267_h44_05.png)

#### Image 6 (Page 74)
![Image 6 - Page 74](Attachments_Ch7/TEL200_Ch7_p074_x71_y214_w141_h44_06.png)

#### Image 7 (Page 74)
![Image 7 - Page 74](Attachments_Ch7/TEL200_Ch7_p074_x186_y214_w145_h44_07.png)

#### Image 8 (Page 74)
![Image 8 - Page 74](Attachments_Ch7/TEL200_Ch7_p074_x305_y214_w32_h44_08.png)

#### Image 9 (Page 74)
![Image 9 - Page 74](Attachments_Ch7/TEL200_Ch7_p074_x311_y214_w80_h44_09.png)

#### Image 10 (Page 74)
![Image 10 - Page 74](Attachments_Ch7/TEL200_Ch7_p074_x365_y214_w64_h44_10.png)

#### Image 11 (Page 74)
![Image 11 - Page 74](Attachments_Ch7/TEL200_Ch7_p074_x305_y214_w32_h44_11.png)

#### Image 12 (Page 74)
![Image 12 - Page 74](Attachments_Ch7/TEL200_Ch7_p074_x409_y214_w31_h44_12.png)

#### Image 13 (Page 74)
![Image 13 - Page 74](Attachments_Ch7/TEL200_Ch7_p074_x455_y214_w107_h44_13.png)

#### Image 14 (Page 74)
![Image 14 - Page 74](Attachments_Ch7/TEL200_Ch7_p074_x536_y214_w67_h44_14.png)

#### Image 15 (Page 74)
![Image 15 - Page 74](Attachments_Ch7/TEL200_Ch7_p074_x37_y254_w333_h44_15.png)

#### Image 16 (Page 74)
![Image 16 - Page 74](Attachments_Ch7/TEL200_Ch7_p074_x71_y294_w63_h44_16.png)

#### Image 17 (Page 74)
![Image 17 - Page 74](Attachments_Ch7/TEL200_Ch7_p074_x37_y334_w86_h44_17.png)

#### Image 18 (Page 74)
![Image 18 - Page 74](Attachments_Ch7/TEL200_Ch7_p074_x103_y334_w150_h44_18.png)

---

## Page 75

Numerical Solution
• The computed joint angles indeed give the desired end-effector position as:
>>> e.fkine(sol.x).printline()
t = 0.6, 0.7; 112°
• As already discussed, there are two solutions for 𝑞but the solution that is
found depends on the initial choice of 𝑞 in optimize.minimize;
2
𝜉
𝑞

### Images

#### Image 1 (Page 75)
![Image 1 - Page 75](Attachments_Ch7/TEL200_Ch7_p075_x37_y175_w56_h44_01.png)

#### Image 2 (Page 75)
![Image 2 - Page 75](Attachments_Ch7/TEL200_Ch7_p075_x71_y175_w36_h44_02.png)

#### Image 3 (Page 75)
![Image 3 - Page 75](Attachments_Ch7/TEL200_Ch7_p075_x81_y175_w31_h44_03.png)

#### Image 4 (Page 75)
![Image 4 - Page 75](Attachments_Ch7/TEL200_Ch7_p075_x86_y175_w66_h44_04.png)

#### Image 5 (Page 75)
![Image 5 - Page 75](Attachments_Ch7/TEL200_Ch7_p075_x126_y175_w32_h44_05.png)

#### Image 6 (Page 75)
![Image 6 - Page 75](Attachments_Ch7/TEL200_Ch7_p075_x132_y175_w49_h44_06.png)

#### Image 7 (Page 75)
![Image 7 - Page 75](Attachments_Ch7/TEL200_Ch7_p075_x159_y175_w35_h44_07.png)

#### Image 8 (Page 75)
![Image 8 - Page 75](Attachments_Ch7/TEL200_Ch7_p075_x126_y175_w32_h44_08.png)

#### Image 9 (Page 75)
![Image 9 - Page 75](Attachments_Ch7/TEL200_Ch7_p075_x179_y175_w94_h44_09.png)

#### Image 10 (Page 75)
![Image 10 - Page 75](Attachments_Ch7/TEL200_Ch7_p075_x247_y175_w39_h44_10.png)

#### Image 11 (Page 75)
![Image 11 - Page 75](Attachments_Ch7/TEL200_Ch7_p075_x126_y175_w32_h44_11.png)

#### Image 12 (Page 75)
![Image 12 - Page 75](Attachments_Ch7/TEL200_Ch7_p075_x48_y215_w36_h44_12.png)

#### Image 13 (Page 75)
![Image 13 - Page 75](Attachments_Ch7/TEL200_Ch7_p075_x48_y215_w36_h44_13.png)

#### Image 14 (Page 75)
![Image 14 - Page 75](Attachments_Ch7/TEL200_Ch7_p075_x48_y215_w36_h44_14.png)

#### Image 15 (Page 75)
![Image 15 - Page 75](Attachments_Ch7/TEL200_Ch7_p075_x81_y175_w31_h44_15.png)

#### Image 16 (Page 75)
![Image 16 - Page 75](Attachments_Ch7/TEL200_Ch7_p075_x48_y215_w36_h44_16.png)

#### Image 17 (Page 75)
![Image 17 - Page 75](Attachments_Ch7/TEL200_Ch7_p075_x123_y215_w32_h44_17.png)

#### Image 18 (Page 75)
![Image 18 - Page 75](Attachments_Ch7/TEL200_Ch7_p075_x132_y215_w57_h44_18.png)

#### Image 19 (Page 75)
![Image 19 - Page 75](Attachments_Ch7/TEL200_Ch7_p075_x163_y215_w33_h44_19.png)

#### Image 20 (Page 75)
![Image 20 - Page 75](Attachments_Ch7/TEL200_Ch7_p075_x334_y353_w24_h171_20.png)

#### Image 21 (Page 75)
![Image 21 - Page 75](Attachments_Ch7/TEL200_Ch7_p075_x321_y488_w298_h24_21.png)

#### Image 22 (Page 75)
![Image 22 - Page 75](Attachments_Ch7/TEL200_Ch7_p075_x371_y386_w212_h106_22.png)

---

## Page 76

3-Dimensional Robotic Arms
• Closed-form solutions have been developed for 
most common types of 6-axis industrial robots and 
many are included in the Toolbox;
• A necessary condition for a closed-form solution 
of a 6-axis robot is a spherical wrist mechanism. 
• This is a gimbal-like mechanism: have a singularity. 
• It doesn’t cause any translation. So, position of 
end-effector is a function only of first three joints!
• An arbitrary end-effector orientation is achieved 
independently by means of the three wrist joints.
2

### Images

#### Image 1 (Page 76)
![Image 1 - Page 76](Attachments_Ch7/TEL200_Ch7_p076_x616_y190_w287_h226_01.jpeg)

---

## Page 77

• Can be solved in a fixed number of operations (therefore, computationally fast)
• Results in all possible solutions to the manipulator kinematics
• Most desirable for real-time control
• Often difficult or even impossible to find
Inverser kinematics: analytical/closed-form solutions

---

## Page 78

• Results in a numerical, iterative solution to system of equations (optimization problem 
solved e.g., using Newton/Raphson techniques)
• Unknown number of operations to solve
• Only returns a single solution 
• Accuracy can be dictated by user but can prolong solution time
• Because of these reasons, this is much less desirable than a closed-form solution.
• Can be applied to all robots.
Inverser kinematics: numerical solution

---

## Page 79

Inverse kinematics for 6-DOF robot 
https://youtu.be/nLI6F6yjcfY

### Images

#### Image 1 (Page 79)
![Image 1 - Page 79](Attachments_Ch7/TEL200_Ch7_p079_x179_y142_w602_h340_01.jpeg)

---

## Page 80

Forward and Inverse Kinematics
• Video clip:
2
https://youtu.be/SZP1KQ2qSEA

### Images

#### Image 1 (Page 80)
![Image 1 - Page 80](Attachments_Ch7/TEL200_Ch7_p080_x196_y145_w640_h360_01.jpeg)

---

## Page 81

Trajectories (Ch. 7.3)
• One of the most common requirements in robotics is to move the robot end-
effector smoothly from pose A to pose B;
• Here, we will discuss two approaches to generating such trajectories: straight
lines in configuration space and straight lines in task space.
• These trajectories are known respectively as:
❑Cartesian motion (MoveL in RAPID) 
❑Joint-space motion (MoveJ/MoveAbsJ in RAPID)
2

---

## Page 82

Joint-space Motion
• Consider the end-effector moving between two Cartesian poses.
$$
>>> TE1 = SE3.Trans(0.4, -0.2, 0) * SE3.Rx(3);
$$
$$
>>> TE2 = SE3.Trans(0.4, 0.2, 0) * SE3.Rx(1);
$$
which describe points in the 𝑥𝑦-plane with different end-effector orientations. 
2

### Images

#### Image 1 (Page 82)
![Image 1 - Page 82](Attachments_Ch7/TEL200_Ch7_p082_x34_y177_w272_h53_01.png)

#### Image 2 (Page 82)
![Image 2 - Page 82](Attachments_Ch7/TEL200_Ch7_p082_x275_y177_w39_h53_02.png)

#### Image 3 (Page 82)
![Image 3 - Page 82](Attachments_Ch7/TEL200_Ch7_p082_x282_y177_w213_h53_03.png)

#### Image 4 (Page 82)
![Image 4 - Page 82](Attachments_Ch7/TEL200_Ch7_p082_x34_y223_w453_h53_04.png)

---

## Page 83

Joint-space Motion
• The joint coordinate vectors associated with these poses are given by:
$$
>>> sol1 = puma.ikine_a(TE1, "ru");
$$
$$
>>> sol2 = puma.ikine_a(TE2, "ru");
$$
where we have specified the right-handed elbow-up (ru) configuration
• We require the motion to occur over a time period of 2 seconds in 20 ms time
steps:
$$
>>> t = np.arange(0, 2, 0.02);
$$
2

### Images

#### Image 1 (Page 83)
![Image 1 - Page 83](Attachments_Ch7/TEL200_Ch7_p083_x34_y178_w68_h53_01.png)

#### Image 2 (Page 83)
![Image 2 - Page 83](Attachments_Ch7/TEL200_Ch7_p083_x70_y178_w37_h53_02.png)

#### Image 3 (Page 83)
![Image 3 - Page 83](Attachments_Ch7/TEL200_Ch7_p083_x75_y178_w59_h53_03.png)

#### Image 4 (Page 83)
![Image 4 - Page 83](Attachments_Ch7/TEL200_Ch7_p083_x103_y178_w44_h53_04.png)

#### Image 5 (Page 83)
![Image 5 - Page 83](Attachments_Ch7/TEL200_Ch7_p083_x103_y178_w44_h53_05.png)

#### Image 6 (Page 83)
![Image 6 - Page 83](Attachments_Ch7/TEL200_Ch7_p083_x138_y178_w88_h53_06.png)

#### Image 7 (Page 83)
![Image 7 - Page 83](Attachments_Ch7/TEL200_Ch7_p083_x194_y178_w38_h53_07.png)

#### Image 8 (Page 83)
![Image 8 - Page 83](Attachments_Ch7/TEL200_Ch7_p083_x200_y178_w102_h53_08.png)

#### Image 9 (Page 83)
![Image 9 - Page 83](Attachments_Ch7/TEL200_Ch7_p083_x270_y178_w62_h53_09.png)

#### Image 10 (Page 83)
![Image 10 - Page 83](Attachments_Ch7/TEL200_Ch7_p083_x194_y178_w38_h53_10.png)

#### Image 11 (Page 83)
![Image 11 - Page 83](Attachments_Ch7/TEL200_Ch7_p083_x324_y178_w79_h53_11.png)

#### Image 12 (Page 83)
![Image 12 - Page 83](Attachments_Ch7/TEL200_Ch7_p083_x194_y178_w38_h53_12.png)

#### Image 13 (Page 83)
![Image 13 - Page 83](Attachments_Ch7/TEL200_Ch7_p083_x103_y178_w44_h53_13.png)

#### Image 14 (Page 83)
![Image 14 - Page 83](Attachments_Ch7/TEL200_Ch7_p083_x75_y387_w40_h53_14.png)

#### Image 15 (Page 83)
![Image 15 - Page 83](Attachments_Ch7/TEL200_Ch7_p083_x106_y387_w56_h53_15.png)

#### Image 16 (Page 83)
![Image 16 - Page 83](Attachments_Ch7/TEL200_Ch7_p083_x137_y387_w101_h53_16.png)

#### Image 17 (Page 83)
![Image 17 - Page 83](Attachments_Ch7/TEL200_Ch7_p083_x206_y387_w39_h53_17.png)

#### Image 18 (Page 83)
![Image 18 - Page 83](Attachments_Ch7/TEL200_Ch7_p083_x103_y178_w44_h53_18.png)

#### Image 19 (Page 83)
![Image 19 - Page 83](Attachments_Ch7/TEL200_Ch7_p083_x279_y387_w56_h53_19.png)

#### Image 20 (Page 83)
![Image 20 - Page 83](Attachments_Ch7/TEL200_Ch7_p083_x206_y387_w39_h53_20.png)

---

## Page 84

Joint-space Motion
• A joint-space trajectory is formed by smoothly interpolating between the joint
configurations 𝑞1  and 𝑞2.
• The scalar interpolation functions quintic or trapezoidal can be used in
conjunction with the multi-axis driver function mtraj as:
$$
>>> traj = mtraj(quintic, sol1.q, sol2.q, t);
$$
or
$$
>>> traj = mtraj(trapezoidal, sol1.q, sol2.q, t);
$$
which each result in a trajectory matrix 𝑞∈ℝ100×6 with one row per time step
and one column per joint.
2

### Images

#### Image 1 (Page 84)
![Image 1 - Page 84](Attachments_Ch7/TEL200_Ch7_p084_x34_y279_w73_h53_01.png)

#### Image 2 (Page 84)
![Image 2 - Page 84](Attachments_Ch7/TEL200_Ch7_p084_x75_y279_w66_h53_02.png)

#### Image 3 (Page 84)
![Image 3 - Page 84](Attachments_Ch7/TEL200_Ch7_p084_x115_y279_w49_h53_03.png)

#### Image 4 (Page 84)
![Image 4 - Page 84](Attachments_Ch7/TEL200_Ch7_p084_x133_y279_w85_h53_04.png)

#### Image 5 (Page 84)
![Image 5 - Page 84](Attachments_Ch7/TEL200_Ch7_p084_x186_y279_w39_h53_05.png)

#### Image 6 (Page 84)
![Image 6 - Page 84](Attachments_Ch7/TEL200_Ch7_p084_x193_y279_w97_h53_06.png)

#### Image 7 (Page 84)
![Image 7 - Page 84](Attachments_Ch7/TEL200_Ch7_p084_x259_y279_w203_h53_07.png)

#### Image 8 (Page 84)
![Image 8 - Page 84](Attachments_Ch7/TEL200_Ch7_p084_x193_y371_w141_h53_08.png)

#### Image 9 (Page 84)
![Image 9 - Page 84](Attachments_Ch7/TEL200_Ch7_p084_x259_y279_w203_h53_09.png)

---

## Page 85

Joint-space Motion
• From here on we will use the equivalent jtraj convenience function
$$
>>> traj = jtraj(sol1.q, sol2.q, t)
$$
Trajectory created by jtraj: 100 time steps x 6 axes
• For mtraj and jtraj the final argument can be a time vector, as here, or an
integer specifying the number of time steps.
• Function jtraj is equivalent to mtraj with quintic interpolation but optimized for
the multi-axis case and also allowing initial- and final velocity to be set using
additional arguments;
2

### Images

#### Image 1 (Page 85)
![Image 1 - Page 85](Attachments_Ch7/TEL200_Ch7_p085_x34_y169_w73_h53_01.png)

#### Image 2 (Page 85)
![Image 2 - Page 85](Attachments_Ch7/TEL200_Ch7_p085_x75_y169_w66_h53_02.png)

#### Image 3 (Page 85)
![Image 3 - Page 85](Attachments_Ch7/TEL200_Ch7_p085_x115_y169_w49_h53_03.png)

#### Image 4 (Page 85)
![Image 4 - Page 85](Attachments_Ch7/TEL200_Ch7_p085_x133_y169_w72_h53_04.png)

#### Image 5 (Page 85)
![Image 5 - Page 85](Attachments_Ch7/TEL200_Ch7_p085_x173_y169_w192_h53_05.png)

#### Image 6 (Page 85)
![Image 6 - Page 85](Attachments_Ch7/TEL200_Ch7_p085_x34_y209_w242_h53_06.png)

#### Image 7 (Page 85)
![Image 7 - Page 85](Attachments_Ch7/TEL200_Ch7_p085_x285_y209_w43_h53_07.png)

#### Image 8 (Page 85)
![Image 8 - Page 85](Attachments_Ch7/TEL200_Ch7_p085_x34_y169_w73_h53_08.png)

#### Image 9 (Page 85)
![Image 9 - Page 85](Attachments_Ch7/TEL200_Ch7_p085_x338_y209_w76_h53_09.png)

#### Image 10 (Page 85)
![Image 10 - Page 85](Attachments_Ch7/TEL200_Ch7_p085_x388_y209_w163_h53_10.png)

---

## Page 86

Joint-space Motion
• The trajectory is best viewed as an animation
>>> puma.plot(traj.q);
but we can also plot the joint angles versus time
>>> xplot(t, traj.q);
2
Fig. 7.20. Joint-space motion. 
a Joint coordinates versus time;
b Cartesian position versus time;
c Cartesian position locus in the 𝑥𝑦-plane
d roll-pitch-yaw angles versus time

### Images

#### Image 1 (Page 86)
![Image 1 - Page 86](Attachments_Ch7/TEL200_Ch7_p086_x34_y167_w73_h53_01.png)

#### Image 2 (Page 86)
![Image 2 - Page 86](Attachments_Ch7/TEL200_Ch7_p086_x75_y167_w132_h53_02.png)

#### Image 3 (Page 86)
![Image 3 - Page 86](Attachments_Ch7/TEL200_Ch7_p086_x176_y167_w39_h53_03.png)

#### Image 4 (Page 86)
![Image 4 - Page 86](Attachments_Ch7/TEL200_Ch7_p086_x183_y167_w84_h53_04.png)

#### Image 5 (Page 86)
![Image 5 - Page 86](Attachments_Ch7/TEL200_Ch7_p086_x236_y167_w45_h53_05.png)

#### Image 6 (Page 86)
![Image 6 - Page 86](Attachments_Ch7/TEL200_Ch7_p086_x75_y243_w80_h53_06.png)

#### Image 7 (Page 86)
![Image 7 - Page 86](Attachments_Ch7/TEL200_Ch7_p086_x124_y243_w58_h53_07.png)

#### Image 8 (Page 86)
![Image 8 - Page 86](Attachments_Ch7/TEL200_Ch7_p086_x486_y148_w456_h328_08.jpeg)

---

## Page 87

Cartesian Motion
• For many applications we require straight-line motion in Cartesian space
which is known as Cartesian motion.
• This is implemented using the Toolbox function ctraj
$$
>>> Ts = ctraj(TE1, TE2, t);
$$
where the arguments are the initial and final pose, T1 and T2, and the time 
steps and it returns the trajectory as an array of SE3 objects.
2

### Images

#### Image 1 (Page 87)
![Image 1 - Page 87](Attachments_Ch7/TEL200_Ch7_p087_x34_y242_w282_h53_01.png)

---

## Page 88

Cartesian Motion
• As for the previous joint-space example, now we will extract and plot the
translation
$$
>>> xplot(t, Ts.t, labels="x y z");
$$
and orientation components
$$
>>> xplot(t, Ts.rpy("xyz"), labels="roll pitch yaw");
$$
of this motion which is shown in Fig. 7.21 along with the path of the end-effector
in the 𝑥𝑦-plane.
2

### Images

#### Image 1 (Page 88)
![Image 1 - Page 88](Attachments_Ch7/TEL200_Ch7_p088_x34_y205_w68_h53_01.png)

#### Image 2 (Page 88)
![Image 2 - Page 88](Attachments_Ch7/TEL200_Ch7_p088_x75_y205_w80_h53_02.png)

#### Image 3 (Page 88)
![Image 3 - Page 88](Attachments_Ch7/TEL200_Ch7_p088_x124_y205_w53_h53_03.png)

#### Image 4 (Page 88)
![Image 4 - Page 88](Attachments_Ch7/TEL200_Ch7_p088_x151_y205_w51_h53_04.png)

#### Image 5 (Page 88)
![Image 5 - Page 88](Attachments_Ch7/TEL200_Ch7_p088_x170_y205_w37_h53_05.png)

#### Image 6 (Page 88)
![Image 6 - Page 88](Attachments_Ch7/TEL200_Ch7_p088_x175_y205_w46_h53_06.png)

#### Image 7 (Page 88)
![Image 7 - Page 88](Attachments_Ch7/TEL200_Ch7_p088_x195_y205_w120_h53_07.png)

#### Image 8 (Page 88)
![Image 8 - Page 88](Attachments_Ch7/TEL200_Ch7_p088_x289_y205_w42_h53_08.png)

#### Image 9 (Page 88)
![Image 9 - Page 88](Attachments_Ch7/TEL200_Ch7_p088_x305_y205_w41_h53_09.png)

#### Image 10 (Page 88)
![Image 10 - Page 88](Attachments_Ch7/TEL200_Ch7_p088_x315_y205_w48_h53_10.png)

#### Image 11 (Page 88)
![Image 11 - Page 88](Attachments_Ch7/TEL200_Ch7_p088_x331_y205_w38_h53_11.png)

#### Image 12 (Page 88)
![Image 12 - Page 88](Attachments_Ch7/TEL200_Ch7_p088_x170_y205_w37_h53_12.png)

#### Image 13 (Page 88)
![Image 13 - Page 88](Attachments_Ch7/TEL200_Ch7_p088_x146_y297_w37_h53_13.png)

#### Image 14 (Page 88)
![Image 14 - Page 88](Attachments_Ch7/TEL200_Ch7_p088_x331_y205_w38_h53_14.png)

#### Image 15 (Page 88)
![Image 15 - Page 88](Attachments_Ch7/TEL200_Ch7_p088_x176_y297_w63_h53_15.png)

#### Image 16 (Page 88)
![Image 16 - Page 88](Attachments_Ch7/TEL200_Ch7_p088_x315_y205_w48_h53_16.png)

#### Image 17 (Page 88)
![Image 17 - Page 88](Attachments_Ch7/TEL200_Ch7_p088_x224_y297_w62_h53_17.png)

#### Image 18 (Page 88)
![Image 18 - Page 88](Attachments_Ch7/TEL200_Ch7_p088_x255_y297_w54_h53_18.png)

#### Image 19 (Page 88)
![Image 19 - Page 88](Attachments_Ch7/TEL200_Ch7_p088_x283_y297_w88_h53_19.png)

#### Image 20 (Page 88)
![Image 20 - Page 88](Attachments_Ch7/TEL200_Ch7_p088_x340_y297_w85_h53_20.png)

#### Image 21 (Page 88)
![Image 21 - Page 88](Attachments_Ch7/TEL200_Ch7_p088_x398_y297_w80_h53_21.png)

#### Image 22 (Page 88)
![Image 22 - Page 88](Attachments_Ch7/TEL200_Ch7_p088_x452_y297_w72_h53_22.png)

---

## Page 89

Cartesian Motion
• The corresponding joint-space trajectory is 
obtained by applying the inverse kinematics
$$
>>> qc = puma.ikine_a(Ts);
$$
and is shown in Fig. 7.21a.
2
Fig. 7.21. Cartesian motion. 
a Joint coordinates versus time; 
b Cartesian position versus time; 
c Cartesian position locus in the xy-plane;
d roll-pitch-yaw angles versus time

### Images

#### Image 1 (Page 89)
![Image 1 - Page 89](Attachments_Ch7/TEL200_Ch7_p089_x480_y140_w451_h328_01.jpeg)

#### Image 2 (Page 89)
![Image 2 - Page 89](Attachments_Ch7/TEL200_Ch7_p089_x34_y197_w73_h53_02.png)

#### Image 3 (Page 89)
![Image 3 - Page 89](Attachments_Ch7/TEL200_Ch7_p089_x75_y197_w54_h53_03.png)

#### Image 4 (Page 89)
![Image 4 - Page 89](Attachments_Ch7/TEL200_Ch7_p089_x98_y197_w37_h53_04.png)

#### Image 5 (Page 89)
![Image 5 - Page 89](Attachments_Ch7/TEL200_Ch7_p089_x103_y197_w49_h53_05.png)

#### Image 6 (Page 89)
![Image 6 - Page 89](Attachments_Ch7/TEL200_Ch7_p089_x121_y197_w164_h53_06.png)

#### Image 7 (Page 89)
![Image 7 - Page 89](Attachments_Ch7/TEL200_Ch7_p089_x253_y197_w72_h53_07.png)

---

## Page 90

Motion through a Singularity
• Here, we deliberately choose a joint
trajectory that moves through a robot
wrist singularity.
• We change the Cartesian endpoints of
the previous example to
$$
>>> TE1 = SE3.Trans(0.5, -0.3, 1.12) * SE3.OA((0, 1, 0), (1, 0, 0));
$$
$$
>>> TE2 = SE3.Trans(0.5, 0.3, 1.12) * SE3.OA((0, 1, 0), (1, 0, 0));
$$
which results in motion in the 𝑦-direction
with the end-effector 𝑧-axis pointing in the
world 𝑥-direction.
2
Fig. 7.22: Block diagram model models/jointspace 
for joint-space motion

### Images

#### Image 1 (Page 90)
![Image 1 - Page 90](Attachments_Ch7/TEL200_Ch7_p090_x40_y293_w159_h31_01.png)

#### Image 2 (Page 90)
![Image 2 - Page 90](Attachments_Ch7/TEL200_Ch7_p090_x181_y293_w23_h31_02.png)

#### Image 3 (Page 90)
![Image 3 - Page 90](Attachments_Ch7/TEL200_Ch7_p090_x185_y293_w231_h31_03.png)

#### Image 4 (Page 90)
![Image 4 - Page 90](Attachments_Ch7/TEL200_Ch7_p090_x40_y333_w372_h31_04.png)

#### Image 5 (Page 90)
![Image 5 - Page 90](Attachments_Ch7/TEL200_Ch7_p090_x515_y142_w406_h186_05.jpeg)

---

## Page 91

Motion through a Singularity
• The Cartesian path is
$$
>>> Ts = ctraj(TE1, TE2, t);
$$
which we convert to joint coordinates
$$
>>> sol = puma.ikine_a(Ts, "lu");
$$
And plot againt time in Fig. 7.23a. 
$$
>>> xplot(t, sol.q, unwrap=True);
$$
2
Fig. 7.23: Joint agnles through a wrist singularity.
a Joint coordinates computed by the inverse kinematics 
function (ikine6s);

### Images

#### Image 1 (Page 91)
![Image 1 - Page 91](Attachments_Ch7/TEL200_Ch7_p091_x34_y171_w282_h53_01.png)

#### Image 2 (Page 91)
![Image 2 - Page 91](Attachments_Ch7/TEL200_Ch7_p091_x34_y251_w123_h53_02.png)

#### Image 3 (Page 91)
![Image 3 - Page 91](Attachments_Ch7/TEL200_Ch7_p091_x125_y251_w151_h53_03.png)

#### Image 4 (Page 91)
![Image 4 - Page 91](Attachments_Ch7/TEL200_Ch7_p091_x245_y251_w44_h53_04.png)

#### Image 5 (Page 91)
![Image 5 - Page 91](Attachments_Ch7/TEL200_Ch7_p091_x258_y251_w79_h53_05.png)

#### Image 6 (Page 91)
![Image 6 - Page 91](Attachments_Ch7/TEL200_Ch7_p091_x305_y251_w49_h53_06.png)

#### Image 7 (Page 91)
![Image 7 - Page 91](Attachments_Ch7/TEL200_Ch7_p091_x323_y251_w55_h53_07.png)

#### Image 8 (Page 91)
![Image 8 - Page 91](Attachments_Ch7/TEL200_Ch7_p091_x34_y331_w73_h53_08.png)

#### Image 9 (Page 91)
![Image 9 - Page 91](Attachments_Ch7/TEL200_Ch7_p091_x75_y331_w80_h53_09.png)

#### Image 10 (Page 91)
![Image 10 - Page 91](Attachments_Ch7/TEL200_Ch7_p091_x124_y331_w58_h53_10.png)

#### Image 11 (Page 91)
![Image 11 - Page 91](Attachments_Ch7/TEL200_Ch7_p091_x151_y331_w77_h53_11.png)

#### Image 12 (Page 91)
![Image 12 - Page 91](Attachments_Ch7/TEL200_Ch7_p091_x196_y331_w43_h53_12.png)

#### Image 13 (Page 91)
![Image 13 - Page 91](Attachments_Ch7/TEL200_Ch7_p091_x208_y331_w106_h53_13.png)

#### Image 14 (Page 91)
![Image 14 - Page 91](Attachments_Ch7/TEL200_Ch7_p091_x282_y331_w86_h53_14.png)

#### Image 15 (Page 91)
![Image 15 - Page 91](Attachments_Ch7/TEL200_Ch7_p091_x337_y331_w45_h53_15.png)

#### Image 16 (Page 91)
![Image 16 - Page 91](Attachments_Ch7/TEL200_Ch7_p091_x481_y100_w405_h301_16.jpeg)

---

## Page 92

Motion through a Singularity
• At time t1.3 s wrist joint angles q3 and 
q5 change very rapidly.
• q5 increase, while q3 decrease rapidly 
• Counterrotational motion: the gripper
does not rotate but; motors are 
working very hard.
2
Fig. 7.23: Joint agnles through a wrist singularity.
a Joint coordinates computed by the inverse kinematics 
function (ikine6s);

### Images

#### Image 1 (Page 92)
![Image 1 - Page 92](Attachments_Ch7/TEL200_Ch7_p092_x481_y100_w405_h301_01.jpeg)

---

## Page 93

Motion through a Singularity
• This is since q4 is close to zero which 
means that the q3 and q5 rotational 
axes of the wrist are almost aligned…
…that is another gimbal lock situation or
singularity.
2
Fig. 7.23: Joint agnles through a wrist singularity.
a Joint coordinates computed by the inverse kinematics 
function (ikine6s);

### Images

#### Image 1 (Page 93)
![Image 1 - Page 93](Attachments_Ch7/TEL200_Ch7_p093_x481_y100_w405_h301_01.jpeg)

---

## Page 94

Motion through a Singularity
• The joint axis alignment means that
the robot has lost 1 degree of freedom
and is now effectively a 5-axis robot;
• Kinematically we can only solve for the
sum (𝑞3 + 𝑞5) and there are an infinite
number of solutions for 𝑞3 and 𝑞5 that
would have the same sum.
2
Fig. 7.23: Joint angles through a wrist singularity.
a Joint coordinates computed by the inverse kinematics
function (ikine6s);

### Images

#### Image 1 (Page 94)
![Image 1 - Page 94](Attachments_Ch7/TEL200_Ch7_p094_x481_y100_w405_h301_01.jpeg)

---

## Page 95

Motion through a Singularity
• From Fig. 7.23b we observe that the
numerical inverse kinematics method
ikine_LM handles the singularity with
far less unnecessary joint motion.
• This is a consequence of the minimum-
norm solution which has returned the
smallest magnitude 𝑞3 and 𝑞5 which
have the correct sum.
2
Fig. 7.23: Joint angles through a wrist singularity.
b
cartesian
trajectory
by
the
numerical
inverse
kinematics function (ikine_LM);

### Images

#### Image 1 (Page 95)
![Image 1 - Page 95](Attachments_Ch7/TEL200_Ch7_p095_x486_y117_w413_h293_01.jpeg)

---

## Page 96

Motion through a Singularity
• The joint-space motion between the
two poses, Fig. 7.23c, is immune to the
singularity problem since it is does NOT
involve inverse kinematics.
• However,
it
will
not
maintain
the
orientation of the tool in the 𝑥-direction
for the whole path…
…only at the two end points.
2
Fig. 7.23: Joint angles through a wrist singularity.
c joint-space motion (jtraj);

### Images

#### Image 1 (Page 96)
![Image 1 - Page 96](Attachments_Ch7/TEL200_Ch7_p096_x493_y121_w379_h286_01.jpeg)

---

## Page 97

Motion through a Singularity
• The dexterity of a robot manipulator, its
ability to move easily in any direction, is
referred to as its manipulability.
• It is a scalar measure, high is good,
and can be computed for each point
along the trajectory
$$
>>> m = puma.manipulability(sol.q);
$$
and is plotted in Fig. 7.23d.
2
Fig. 7.23: Cartesian motion through a wrist singularity.
d manipulability;

### Images

#### Image 1 (Page 97)
![Image 1 - Page 97](Attachments_Ch7/TEL200_Ch7_p097_x37_y309_w319_h44_01.png)

#### Image 2 (Page 97)
![Image 2 - Page 97](Attachments_Ch7/TEL200_Ch7_p097_x506_y149_w354_h265_02.jpeg)

---

## Page 98

Motion through a Singularity
• This
shows
that
manipulability
for
analytic IK (red) was almost zero around
the time of the rapid wrist joint motion.
• The numerical inverse kinematics (blue)
was able to keep manipulability high
throughout
the
trajectory
since
it
implicitly minimizes the joint velocities.
• The manipulability measure (w) and
the
numerical
inverse
kinematics
function are based on the manipulator’s
Jacobian matrix (𝐽) which is the topic of 
the next chapter (Ch. 8).
2
Fig. 7.23: Cartesian motion through a wrist singularity.
d manipulability;
𝑤=
det(𝐽𝑞𝐽𝑇𝑞)

### Images

#### Image 1 (Page 98)
![Image 1 - Page 98](Attachments_Ch7/TEL200_Ch7_p098_x506_y149_w354_h265_01.jpeg)

---

## Page 99

Some further aspects:
• Configuration Change
❑Move from/to different configurations (left- or right-handed, elbow up or down,
wrist flipped/non-flipped)
• Under-Actuated Manipulator
❑An under-actuated manipulator is one that has fewer than 6 (six) joints and is,
therefore, limited in the end-effector poses that it can attain.
• Redudant Manipulator
❑A redundant manipulator is a robot with more than 6 (six) joints.
2

---

## Page 100

Wrapping up…
• We have learned how to determine the forward 
kinematics and the inverse kinematics of a serial-link 
robot manipulator;
• Forward kinematics involves determining the end-
effector pose 𝝃given the joint coordinates 𝒒;
• Inverse kinematics is the problem of determining 
the joint coordinates 𝒒given the end-effector pose 𝝃;
• The joint and link structure can be expressed 
systematically by using four parameters for each 
link through the Denavit-Hartenberg (DH) convention;
Forward kinematics
Backward kinematics
q  
 𝝃

---

## Page 101

Wrapping up…
• For simple robots, or those with 6 (six) joints and a spherical wrist we can 
compute the inverse kinematics using an analytic solution; 
…This inverse is not unique, and the robot may have several joint configurations 
that result in the same end-effector pose; 
• For robots which do NOT have 6 (six) joints and a spherical wrist we can use 
an iterative numerical (optimization) approach to solving the inverse kinematics;
• We also learned about creating paths to move the end-effector smoothly 
between two given points;

---

## Page 102

Wrapping up…
• Joint-space paths are simple to compute but in general do NOT result in
straight line paths in Cartesian space which may be problematic for some
applications (MoveJ/MoveAbsJ in RAPID)
• Straight line paths in Cartesian space can be generated but singularities in
the workspace may lead to very high joint rates (e.g., joint velocities)
(MoveL in RAPID)

---

## Page 103

Watch the QUT Robot Academy videos:
• Robotic arms and forward kinematics
• Inverse kinematics and robot motion
«Homework»

---

## Page 104

Bibliography
[1] Corke, P., Robotics, Vision and Control: Fundamental Algorithms in
Python, Springer International Publishing AG, 3rd Ed., 2023.
[2] Siciliano, B., Sciavicco, Villani, L. & Oriolo, G., Robotics: Modeling, 
Planning and Control, Springer-Verlag London Limited, 2nd Ed., 2009.

---

## Page 105

Thank you for your attention !

---
