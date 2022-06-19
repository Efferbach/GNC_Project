clc
addpath('./Thrust/');

q_init = axang2quat([0 1 0 -1.572])';
R_eleb1 = axang2rotm([0 0 1 -0.16349]);
R_eleb2 = axang2rotm([0 0 1 0.15331]);
t_init = [0.5;0;0.5];
eta_init = [t_init;q_init];

m = 0.21; %kg
g= -9.82; %m/s²
Ix = 3*10^(-3); %kg*m²
Ixz = -14*10^(-6); %kg*m²
Iy = 6.2*10^(-4); %kg*m²
Iz = 3.5*10^(-3); %kg*m²
Ic = [Ix 0 -Ixz;0 Iy 0;-Ixz 0 Iz];

M = [m * eye(3,3) zeros(3,3);zeros(3,3) Ic];
M_inv = inv(M);

Lambda = deg2rad(19.8); %rad
S = 0.08; %m²
b = 0.5; %m
A_R = b^2/S;
Cl_alpha = (2*pi*cos(Lambda))/(2*cos(Lambda)/A_R + sqrt(1 + (2*cos(Lambda)/A_R)^2));

rho = 1.225; %kg/m³

k0 = 0.87; %Oswalds efficiency factor
C_D0 = 0.02; %Skin friction coefficient

%Thrust
rp = 0.0625;
l = 0.145;
Ith = 1.6*10^(-6);
rpm2rad = 0.104719755;
Omega_max = 1500;

C_D_R = 1.1;
dx = 130; %mm
v_w = 0;

% Pitching moment coefficient given AOA
C_M_alpha_deg = [-150 0.35; -125 0.4; -100 0.35; -75 0.29; -50 0.21; -25 0.11; 0 0; 25 -0.115; 50 -0.21; 75 -0.29; 100 -0.34; 125 -0.39; 150 -0.34];
C_M_alpha = [-2.618 0.35; -2.1817 0.4; -1.7453 0.35; -1.309 0.29; -0.8727 0.21; -0.4363 0.11; 0 0; 0.4363 0.11; 0.8727 0.21; 1.309 0.29; 1.7453 0.35; 2.1817 0.4; 2.618 0.35];

C_M_coeff = polyfit(C_M_alpha(:,1), C_M_alpha(:,2),3);


%%Top left - down right
A1 = 0.0225;
A2 = 0.0235;
A3 = 0.001575;
A4 = 0.01197;
A5 = 0.004725;

A = [A1;A2;A3;A4;A5;A4;A3];

A_C = [0.1264;0;0];

% Span
b1 = 0.500;
b2 = 0.500;
b3 = 0.0225;
b4 = 0.190;
b5 = 0.075;

b = [b1;b2;b3;b4;b5;b4;b3];

% Mean aerodynamical chord MAC
c1 = 0.090;
c2 = 0.047;
c3 = 0.063;

c = [c1;c2;c3;c3;c3;c3;c3];

% Aerodynamic center wrt bodyframe
r1_B = [47.5;0;0];
r2_B = [-31.25;0;0];
r3_B = [-82.75;-238.75;0];
r4_B = [-82.75;-132.5;0];
r5_B = [-82.75;0;0];
r6_B = [-82.75;132.5;0];
r7_B = [-82.75;238.75;0];

r_B = [r1_B r2_B r3_B r4_B r5_B r6_B r7_B];
r_B = r_B./1000; % From mm to m


% Rod segments
% Right side start in geometric frame
pr_start = [46.5 19 -10 0 -10 14.5 177 177 177 177 177 177 177 177 177 177 177 177 172 172 172;
    252	252	252	252	252	252	207.8 181.3 145	108.8 82.2 72.5 82.2 108.7 145 181.3 207.8 217.5 140.5 140.5 157;
    0 -66 -66 25 69 69 36.2 62.8 72.5 62.8 36.2 0 -36.2 -62.8 -72.5 -62.8 -36.3 0 9.5 -9.5 0];
% Right side end in geomtric frame
pr_end = [19 -10 0 -10 14.5	81 177 177 177 177 177 177 177 177 177 177 177 177 175 175 175;
    252	252	252	252	252	252	182.3 146 109.8 83.2 73.5 83.2 109.7 146 182.3 208.8 218.5 208.8 114.5 114.5 209;
    -66	-66 25 69 69 0 62.8 72.5 62.8 36.2 0 -36.2 -62.8 -72.5 -62.8 -36.3 0 36.3 54.6 -54.6 0];
p_d = [7 7 7 7 7 7 3 3 3 3 3 3 3 3 3 3 3 3 3 3 3];

% Left side start in geometric frame
pl_start = pr_start;
pl_start(2,:) = -pl_start(2,:);
% Left side end in geometric frame
pl_end = pr_end;
pl_end(2,:) = -pl_end(2,:);

% Lengths of rods
lr = pr_end-pr_start; % Right side
ll = pl_end-pl_start; % Left side

% Right side start in body frame
pr_start(1,:) = pr_start(1,:)-dx;
% Right side end in body frame
pr_end(1,:) = pr_end(1,:)-dx;

% Left side start in body frame
pl_start(1,:) = pl_start(1,:)-dx;
% Left siden end in body frame
pl_end(1,:) = pl_end(1,:)-dx;

% Right side middle of rods in body frame
prm = pr_start+lr./2;
% Left side middle of rods in body frame
plm = pl_start+ll./2;

% Vertical parts on each side
% Area
Av1 = 0.003120;
Av2 = Av1;

% Span
bv1 = 0.026;
bv2 = bv1;

% Mean aerodynamical chord MAC
cv1 = 0.120;
cv2 = cv1;

% Aerodynamical center wrt bodyframe
rv1_B = [-45;-252;0];
rv2_B = [-45;252;0];

rv_B = [rv1_B rv2_B];
rv_B = rv_B./1000; % From mm to m


% Some of the above is done in mm and is now converted to m
pr_start = pr_start./1000;
pr_end = pr_end./1000;
pl_start = pl_start./1000;
pl_end = pl_end./1000;
lr = lr./1000;
ll = ll./1000;
li = cat(2,lr,ll);
prm = prm./1000;
plm = plm./1000;
pm = cat(2,prm,plm);
p_d = p_d/1000;
d = cat(2,p_d,p_d);



%%Alternative areas - From left to right

At1 = 0.002408;
At2 = 0.004785;
At3 = 0.013051;
At4 = 0.011100;
At5 = 0.017042;

At = [At1;At2;At3;At4;At5;At4;At3;At2;At1];

% Span
bt1 = 0.0225;
bt2 = 0.0383;
bt3 = 0.0884;
bt4 = 0.0633;
bt5 = 0.0750;

bt = [bt1;bt2;bt3;bt4;bt5;bt4;bt3;bt2;bt1];

% Mean aerodynamical chord
ct1 = 0.1100;
ct2 = 0.1250;
ct3 = 0.1482;
ct4 = 0.1759;
ct5 = 0.2314;

ct = [ct1;ct2;ct3;ct4;ct5;ct4;ct3;ct2;ct1];

% Aerodynamic center wrt bodyframe
rt1_B = [-47.5;-239.3;0];
rt2_B = [-36.25;-208;0];
rt3_B = [-18.85;-143.4;0];
rt4_B = [1.9;-68.4;0];
rt5_B = [43;0;0];
rt6_B = [1.9;68.4;0];
rt7_B = [-18.85;143.4;0];
rt8_B = [-36.25;208;0];
rt9_B = [-47.5;239.3;0];

rt_B = [rt1_B rt2_B rt3_B rt4_B rt5_B rt6_B rt7_B rt8_B rt9_B];
rt_B = rt_B./1000;




