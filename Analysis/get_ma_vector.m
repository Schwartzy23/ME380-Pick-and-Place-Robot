function ma = get_ma_vector(L1, L2, ddy1, ddy2, lambda1, dlambda1, lambda2, dlambda2, ddlambda1, ddlambda2, thetagrip, dthetagrip, rgrip, ddthetagrip, ddr0,theta0) 

% Mass and Moments % 

mL1 = 0.2;
mL2 = mL1;
my1 = 0.2;
my2 = my1;
mgrip = 0.4;

g=9810;

IL1 = (1/3)*mL1*L1^2; %Left Link
IL2 = (1/3)*mL2*L2^2; %Right Link

Igrip = mgrip*rgrip^2; %Consolodate Gripper to a point mass and apply P.A.T.


% Eqns of COM acceleration %

agx_L1 = ddr0*cosd(theta0) - L1/2* (ddlambda1*sind(lambda1) + dlambda1^2*cosd(lambda1));
agy_L1 = ddr0*sind(theta0) + L1/2* (ddlambda1*cosd(lambda1) - dlambda1^2*sind(lambda1));
agx_L2 = ddr0*cosd(theta0) - L1/2* (ddlambda2*sind(lambda2) + dlambda2^2*cosd(lambda2));
agy_L2 = ddr0*sind(theta0) + L1/2* (ddlambda2*cosd(lambda2) - dlambda2^2*sind(lambda2));
%agx_y1 = 0;
agy_y1 = ddy1;
%agx_y2 = 0;
agy_y2 = ddy2;
agx_grip = ddr0*cosd(theta0) - rgrip*(ddthetagrip*sind(thetagrip) + dthetagrip^2*cosd(thetagrip));
agy_grip = ddr0*sind(theta0) + rgrip*(ddthetagrip*cosd(thetagrip) - dthetagrip^2*sind(thetagrip));

% MA Vector %

ma = [mL1*agx_L1;
      mL1*(agy_L1+g);
      IL1*ddlambda1;
      
      mL2*agx_L2;
      mL2*(agy_L2+g);
      IL2*ddlambda2;
      
      %my1*agx_y1; --Zero
      my1*(agy_y1+g);
      
      %my2*agx_y2; --Zero
      my2*(agy_y2+g);
      
      mgrip*agx_grip;
      mgrip*(agy_grip+g);
      Igrip*ddthetagrip;
      ];

end