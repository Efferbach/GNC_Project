wn = 8;
damp = 1;

wpts_qs = [axang2quat([0 1 0 -1.572])' axang2quat([0 1 0 -1.572])' axang2quat([0 1 0 -1.572/2])' axang2quat([0 1 0 0])' axang2quat([0 1 0 0])' axang2quat([0 1 0 2*-1.572/2])' axang2quat([0 1 0 2*-1.572/2])'  axang2quat([0 1 0 -1.572])'];
%wpts_qs = [axang2quat([0 1 0 -1.572])' axang2quat([0 1 0 -1.572])' axang2quat([0 1 0 -1.572])' axang2quat([0 1 0 -1.572])' axang2quat([0 1 0 -1.572])' axang2quat([0 1 0 -1.572])' axang2quat([0 1 0 -1.572])'  axang2quat([0 1 0 -1.572])'];
wpts_p = [0.5 0.5 2 4 8 10 12 12;
          0 1 2 3 4 3 2 1;
          0.5 5 6 6 6 8 10 0.5];
%wpts_qs = [axang2quat([0 1 0 -1.572])' axang2quat([0 1 0 -1.572])' axang2quat([0 1 0 -1.572])' axang2quat([0 1 0 -1.572])'];
%wpts_p = [0.5 2 4 6;
%          0 0 0 0;
%          0.5 2 4 6];
wpts = [wpts_p;wpts_qs];


%SLERP Coeff boundary constraints
SLERP_bconstraints = [0;0;1;0];
SLERP_B = [1 0 0 0;
     0 1 0 0;
     1 1 1 1;
     0 1 2 3];
SLERP_a = flip(SLERP_B \ SLERP_bconstraints);

INIT_ETA = [0.5;0;0.5;axang2quat([0 1 0 -1.572])'];

tf_b = [wn^2];
tf_a = [(1/wn) (1+2*damp) (1+2*damp)*wn wn^2];
