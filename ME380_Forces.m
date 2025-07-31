%ME380 Kinetic Analysis

clear; clc; close all;

%%%%%% Constant %%%%%%%
b1 = 61;
b2 = 396;

L1 = 405;
L2 = L1;

rgrip = 50;


%%% Important inputs %%%
theta0 = 12.76;

r0 = 1;
dr0 = 0;
ddr0 = 3;

%%%%%%% Variable %%%%%%
%y1, y2, lambda1, lambda2.
thetagrip = 0;
dthetagrip = 0;
ddthetagrip = 0;



FM11_mag = []; FM11_theta = [];
F01_mag = []; F01_theta = [];
F12_mag = []; F12_theta = [];
F25_mag = []; F25_theta = [];
F23_mag = []; F23_theta = [];
F35_mag = []; F35_theta = [];
F34_mag = []; F34_theta = [];
F04_mag = []; F04_theta = [];
F4M2_mag = []; F4M2_theta = [];


travel_dist_mm = 335;

r0_list = []; %defining matrix

lambda1_list = [];

tol = 1;

for r = 1:1:travel_dist_mm

    %%%%% Kinematics %%%%%%

    lambda1 = 360-acosd((r*cosd(theta0)+b1)/L1);
    lambda2 = 360-acosd((r*cosd(theta0)-b2)/L2);
    y1 = r*sind(theta0)-L1*sind(lambda1);
    y2 = r*sind(theta0)-L2*sind(lambda2);

    dr0 = sqrt(dr0^2 +2*ddr0*r);
  
    dlambda1 = (-dr0*cosd(theta0))/(L1*sind(lambda1));
    dlambda2 = (-dr0*cosd(theta0))/(L2*sind(lambda2));
    dy1 = dr0*sind(theta0)-L1*cosd(lambda1)*dlambda1;
    dy2 = dr0*sind(theta0)-L2*cosd(lambda2)*dlambda2;

    ddlambda1 = -(L1*cosd(lambda1)*dlambda1^2 + ddr0*cosd(theta0))/L1/sind(lambda1);
    ddlambda2 = -(L2*cosd(lambda2)*dlambda2^2 + ddr0*cosd(theta0))/L2/sind(lambda2);
    ddy1 = ddr0*sind(theta0) - L1*cosd(lambda1)*ddlambda1 + L1*sind(lambda1)*dlambda1^2;
    ddy2 = ddr0*sind(theta0) - L2*cosd(lambda2)*ddlambda2 + L2*sind(lambda2)*dlambda2^2;

    % *****Add in Gripper Kinematics******** % 


    lambda1_list(r)= lambda1; lambda2_list(r)= lambda2; %y1_list(r) = y1; y2_list(r) = y2;
    %dlambda1_list(r)= dlambda1; dlambda2_list(r)= dlambda2; dy1_list(r) = dy1; dy2_list(r) = dy2;

    
    
    A = get_A_matrix(L1, L2, lambda1, lambda2, thetagrip, rgrip); % call on A matrix

    ma = get_ma_vector(L1, L2, ddy1, ddy2, lambda1, dlambda1, lambda2, dlambda2, ddlambda1, ddlambda2, thetagrip, dthetagrip, rgrip, ddthetagrip, ddr0,theta0); % call on ma vector matrix
   
    x = pinv(A) * ma; % solve for Force matrix
    
    FM11y = x(1); F01x = x(2); F12x = x(3); F12y = x(4); F25x = x(5); F25y = x(6); F23x = x(7); 
    F23y = x(8); F35x = x(9);   F35y = x(10); F34x = x(11); F34y = x(12); F04x = x(13); F4M2y = x(14); 



    FM11_mag = [FM11_mag,FM11y];
    FM11_theta = [FM11_theta, pi/2];
    F01_mag = [F01_mag, F01x]; 
    F01_theta = [F01_theta, 0];
    F12_mag = [F12_mag, hypot(F12x,F12y)]; 
    F12_theta = [F12_theta, atan(F12y/F12x)];
    if r > 1 && abs(F12_theta(r)-F12_theta(r-1)) > tol
        F12_theta(r) = F12_theta(r)-pi;
    end
    F25_mag = [F25_mag, hypot(F25x,F25y)]; 
    F25_theta = [F25_theta, atan(F25y/F25x)];
    if r > 1 && abs(F25_theta(r)-F25_theta(r-1)) > tol
        F25_theta(r) = F25_theta(r)+pi;
    end
    F23_mag = [F23_mag, hypot(F23x,F23y)]; 
    F23_theta = [F23_theta, atan(F23y/F23x)];
    if r > 1 && abs(F23_theta(r)-F23_theta(r-1)) > tol
        F23_theta(r) = F23_theta(r)-pi;
    end
    F35_mag = [F35_mag, hypot(F35x,F35y)];  
    F35_theta = [F35_theta, atan(F35y/F35x)];
    if r > 1 && abs(F35_theta(r)-F35_theta(r-1)) > tol
        F35_theta(r) = F35_theta(r)+pi;
    end
    F34_mag = [F34_mag, hypot(F34x,F34y)];  
    F34_theta = [F34_theta, atan(F34y/F34x)];
    if r > 1 && abs(F34_theta(r)-F34_theta(r-1)) > tol
        F34_theta(r) = F34_theta(r)-pi;
    end
    F04_mag = [F04_mag, F04x];
    F04_theta = [F04_theta,0];
    F4M2_mag = [F4M2_mag, F4M2y]; 
    F4M2_theta = [F4M2_theta, pi/2];


end

%%

save('L2S2_Force.txt', 'F34_mag', '-ascii')
save('L2S2_Angle.txt', 'F34_theta', '-ascii')

save('L1S1_Force.txt', 'F12_mag', '-ascii')
save('L1S1_Angle.txt', 'F12_theta', '-ascii')

save('L1G_Force.txt', 'F25_mag', '-ascii')
save('L1G_Angle.txt', 'F25_theta', '-ascii')

save('GL2_Force.txt', 'F35_mag', '-ascii')
save('GL2_Angle.txt', 'F35_theta', '-ascii')

save('L1L2_Force.txt', 'F23_mag', '-ascii')
save('L1L2_Angle.txt', 'F23_theta', '-ascii')

%%

r = 1:travel_dist_mm;

F_mags = [FM11_mag; F01_mag; F12_mag; F25_mag; F23_mag; F35_mag;  F34_mag;  F04_mag; F4M2_mag];
F_titles = ["LS Motor - LS Slider Force" "LS Slider - Linear Shaft Force" "LS Slider - Link1 Force" "Link 1 - Gripper Force" "Link 1 - Link 2 Force" "Link 2 - Gripper Force" "Link 2 - RS Slider Force" "RS Slider - Linear Shaft Force" "RS Slider - RS Motor Force"];

F_angles = [FM11_theta; F01_theta; F12_theta; F25_theta; F23_theta; F35_theta; F34_theta; F04_theta; F4M2_theta];
F_ang_titles = ["LS Motor - LS Slider Force Angle" "LS Slider - Linear Shaft Force Angle" "LS Slider - Link1 Force Angle" "Link 1 - Gripper Force Angle" "Link 1 - Link 2 Force Angle" "Link 2 - Gripper Force Angle" "Link 2 - RS Slider Force Angle" "RS Slider - Linear Shaft Force Angle" "RS Slider - RS Motor Force Angle"];

for i =1:1:9
    
    figure(i)
    plot(r, F_mags(i,:))
    grid on;
    title(F_titles(i))
end

for i =1:1:9
    
    figure(i+9)
    plot(r, F_angles(i,:))
    grid on;
    title(F_ang_titles(i))
end

