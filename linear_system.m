load('parameters.mat');

g = p.g;
l = p.l;
Ixx = p.Ixx;
Iyy = p.Iyy;
Izz = p.Izz;
kD = p.kD;
m = p.m;
mp = p.mp;
M = m + mp;

A_vv = -kD / m * eye(3);
A_veta = [0, (M * g / m), 0; 
          - (M * g / m), 0, 0; 
          0, 0, 0];
A_valpha = [- (mp * g / m); 0; 0];
alpha_alphav = [- kD / (M * l), 0, 0];
alpha_alphaeta = [0, -g / l, 0];

B_v = [0, 0, 0, 0;
       0, 0, 0, 0;
       1/M, 0, 0, 0];
B_nu = [0, 1/Ixx, 0, 0;
        0, 0, 1/Iyy, 0;
        0, 0, 0, 1/Izz];

% set up A matrix
A = zeros(14,14);
A(1:3,4:6) = eye(3);
A(4:6,4:6) = A_vv;
A(4:6,7:9) = A_veta;
A(4:6,13) = A_valpha;
A(7:9,10:12) = eye(3);
A(13,14) = 1;
A(14,4:6) = alpha_alphav;
A(14,7:9) = alpha_alphaeta;
A(14,13) = g/l;

% set up B matrix
B = zeros(14,4);
B(4:6,:) = B_v;
B(10:12,:) = B_nu;

% set up C matrix
C = eye(14);

% set up D matrix
D = 0;

% state space system
sys = ss(A, B, C, D);