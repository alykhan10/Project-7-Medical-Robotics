% Jacobian Computation of the Robot.

% Obtaining the Z-Axes Vector from the Resultant Transformation Matrix of the Robot.

ZT = TROBOT(:,3);

% Obtaining the Joint Variables Vector of the Robot.

QVAR = [theta1 theta2 theta3 theta4 theta5 theta6 theta7 theta8 theta9 dis10];

% Computing the Linear Velocity Jacobian of the Robot.

JV = jacobian(ZT,QVAR);

% Computing the Base Transformation Matrices of Spherical Joint 1.

BASET0 = eye(4);
BASET1 = T1;
BASET2 = BASET1*T2;
BASET3 = BASET2*T3;

% Computing the Base Transformation Matrices of Spherical Joint 2.

BASET4 = BASET3*T4;
BASET5 = BASET4*T5;
BASET6 = BASET5*T6;

% Computing the Base Transformation Matrices of Spherical Joint 3.

BASET7 = BASET6*T7;
BASET8 = BASET7*T8;
BASET9 = BASET8*T9;

% Obtaining the Z-Axes Vectors from the Base Transformation Matrices of Spherical Joint 1.

Z0 = BASET0(:,3);
Z1 = BASET1(:,3);
Z2 = BASET2(:,3);
Z3 = BASET3(:,3);

% Obtaining the Z-Axes Vectors from the Base Transformation Matrices of Spherical Joint 2.

Z4 = BASET4(:,3);
Z5 = BASET5(:,3);
Z6 = BASET6(:,3);

% Obtaining the Z-Axes Vectors from the Base Transformation Matrices of Spherical Joint 3.

Z7 = BASET7(:,3);
Z8 = BASET8(:,3);
Z9 = BASET9(:,3);

% Compiling the Z-Axes Vectors into a Matrix.

ZMATRIX = [Z0 Z1 Z2 Z3 Z4 Z5 Z6 Z7 Z8 Z9];


% Obtaining the Coefficients for Spherical Joint 1.

P1 = [1];
P2 = [1];
P3 = [1];

% Obtaining the Coefficients for Spherical Joint 2.

P4 = [1];
P5 = [1];
P6 = [1];

% Obtaining the Coefficients for Spherical Joint 3.

P7 = [1];
P8 = [1];
P9 = [1];

% Obtaining the Coefficients for the Prismatic Joint.

P10 = [0];

% Compiling the Coefficients into a Vector.

PVECTOR = [P1 P2 P3 P4 P5 P6 P7 P8 P9 P10];

% Computing the Angular Velocity Jacobian of the Robot.

JW = PVECTOR.*ZMATRIX;

% Computing the Full Velocity Jacobian of the Robot.

JFULL = [JV;JW]; 