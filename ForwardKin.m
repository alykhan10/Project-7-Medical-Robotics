% Forward Kinematics Computation of the Robot.

% Initializing the Variables.

syms dis1 dis3 dis4 dis6 dis7 dis9 dis10
syms theta1 theta2 theta3 theta4 theta5 theta6 theta7 theta8 theta9

% Computing the Transformation Matrix of Spherical Joint 1.

T1 = 
[cos(theta1) 0 -sin(theta1) 0; sin(theta1) 0 cos(theta1) 0; 0 -1 0 dis1; 0 0 0 1];
T2 = 
[cos(theta2) 0 sin(theta2) 0; sin(theta2) 0 -cos(theta2) 0; 0 1 0 0; 0 0 0 1];
T3 = 
[cos(theta3) 0 sin(theta3) 0; sin(theta3) 0 -cos(theta3) 0; 0 1 0 dis3; 0 0 0 1];

TSPH1 = T1*T2*T3;

% Computing the Transformation Matrix of Spherical Joint 2.

T4 = 
[cos(theta4) 0 -sin(theta4) 0; sin(theta4) 0 cos(theta4) 0; 0 -1 0 dis4; 0 0 0 1];
T5 = 
[cos(theta5) 0 sin(theta5) 0; sin(theta5) 0 -cos(theta5) 0; 0 1 0 0; 0 0 0 1];
T6 = 
[cos(theta6) 0 sin(theta6) 0; sin(theta6) 0 -cos(theta6) 0; 0 1 0 dis6; 0 0 0 1];

TSPH2 = T4*T5*T6;

% Computing the Transformation Matrix of Spherical Joint 3.

T7 = 
[cos(theta7) 0 -sin(theta7) 0; sin(theta7) 0 cos(theta7) 0; 0 -1 0 dis7; 0 0 0 1];
T8 = 
[cos(theta8) 0 sin(theta8) 0; sin(theta8) 0 -cos(theta8) 0; 0 1 0 0; 0 0 0 1];
T9 = 
[cos(theta9) -sin(theta9) 0 0; sin(theta9) cos(theta9) 0 0; 0 0 1 dis9; 0 0 0 1];

TSPH3 = T7*T8*T9;

% Computing the Transformation Matrix of the Prismatic Joint.

T10 = [1 0 0 0; 0 1 0 0; 0 0 1 dis10; 0 0 0 1];

TPRSM = T10;

% Computing the Resultant Transformation Matrix of the Robot.

TROBOT = TSPH1*TSPH2*TSPH3*TPRSM;