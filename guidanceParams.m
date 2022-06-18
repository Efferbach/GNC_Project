wn = 8;
damp = 1;

wpts = [10 25 30 40 45 50];
wpts_qs = [eul2quat([0 0 0], "XYZ")' eul2quat([0.5 0 0], "XYZ")' eul2quat([0.5 0.5 0], "XYZ")' eul2quat([0.5 0.5 0.5], "XYZ")' eul2quat([1 0.5 0.5], "XYZ")' eul2quat([1 1 0.5], "XYZ")'];
wpts = [wpts;zeros(2,6);wpts_qs];


%SLERP Coeff boundary constraints
SLERP_bconstraints = [0;0;1;0];
SLERP_B = [1 0 0 0;
     0 1 0 0;
     1 1 1 1;
     0 1 2 3];
SLERP_a = flip(SLERP_B \ SLERP_bconstraints);

INIT_ETA = [0;0;0;1;0;0;0];