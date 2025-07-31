function A = get_A_matrix(L1, L2, lambda1, lambda2, thetagrip, rgrip)

A = [0 0 1 0 1 0 -1 0 0 0 0 0 0 0;
     0 0 0 -1 0 -1 0 1 0 0 0 0 0 0;
     0 0 -L1/2*sind(lambda1) L1/2*cosd(lambda1) L1/2*sind(lambda1) -L1/2*cosd(lambda1) -L1/2*sind(lambda1) L1/2*cosd(lambda1) 0 0 0 0 0 0;
     0 0 0 0 0 0 -1 0 -1 0 1 0 0 0;
     0 0 0 0 0 0 0 1 0 1 0 1 0 0;
     0 0 0 0 0 0 -L2/2*sind(lambda2) -L2/2*cosd(lambda2) -L2/2*sind(lambda2) -L2/2*cosd(lambda2) 0 0 0 0;
     -1 0 0 -1 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 1 0 1;
     0 0 0 0 1 0 0 0 -1 0 0 0 0 0;
     0 0 0 0 0 -1 0 0 0 1 0 0 0 0;
     0 0 0 0 -rgrip*sind(thetagrip) rgrip*cosd(thetagrip) 0 0 rgrip*sind(thetagrip) -rgrip*cosd(thetagrip) 0 0 0 0;
     ];

end