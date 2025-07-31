% ME380 Mechanism Plot with Dynamic Forces
clear;
clc;
close all;

%%%%%%% Initiating Constants %%%%%%%%%
b1 = 61;
b2 = 396;

L1 = 405;
L2 = 405;

beta1 = 0;
beta2 = 0;

gamma1 = 90;
gamma2 = 90;

theta0 = 12.76;
r0 = 1;

fl1s1 = load('L1S1_Force.txt');
al1s1 = load('L1S1_Angle.txt');

fl2s2 = load('L2S2_Force.txt');
al2s2 = load('L2S2_Angle.txt');

fl1g = load('L1G_Force.txt');
al1g = load('L1G_Angle.txt');

fgl2 = load('GL2_Force.txt');
agl2 = load('GL2_Angle.txt');

fl1l2 = load('L1L2_Force.txt');
al1l2 = load('L1L2_Angle.txt');



scalefl1s1 = -.2*fl1s1;
scalefl2s2 = -.2*fl2s2;
scalefl1g = -.5*fl1g;
scalefgl2 = .5*fgl2;
scalefl1l2 = .5*fgl2;


for r = r0:335

    %%%% Setting dependent variables %%%%%
    lambda1(r) = acosd((r*cosd(theta0)+b1)/L1);
    y1(r) = r*sind(theta0)-L1*sind(lambda1(r));

    lambda2(r) = acosd((r*cosd(theta0)-b2)/L2);
    y2(r) = r*sind(theta0)-L2*sind(lambda2(r));


    %%%%% x,y Position of significant points on linkage %%%%%
    Ax(r)=0;
    Ay(r)=0;

    Bx(r)= b1;
    By(r)= 0;

    Cx(r)= Bx(r) + r*cosd(theta0);
    Cy(r)= r*sind(theta0);

    Dx(r)= 0;
    Dy(r)= -y1(r);

    %Second Four Bar Linkage
    
    Fx(r) = b1 + b2;
    Fy(r) = 0;

    Ex(r) = Fx(r);
    Ey(r) = -y2(r);

    C2x(r) = Ex(r) + L2*cosd(lambda2(r));
    C2y(r) = -Ey(r) + L2*sind(lambda2(r));

    F2x(r) = Ex(r) + scalefl2s2(r)*cos(al2s2(r));
    F2y(r) = Ey(r) + scalefl2s2(r)*sin(al2s2(r));

    F1x(r) = Dx(r) + scalefl1s1(r)*cos(al1s1(r));
    F1y(r) = Dy(r) + scalefl1s1(r)*sin(al1s1(r));

    F1Gx(r) = Cx(r) + scalefl1g(r)*cos(al1g(r));
    F1Gy(r) = Cy(r) + scalefl1g(r)*sin(al1g(r));

    FG2x(r) = Cx(r) + scalefgl2(r)*cos(agl2(r));
    FG2y(r) = Cy(r) + scalefgl2(r)*sin(agl2(r));

    FL1L2x(r) = Cx(r) + scalefl1l2(r)*cos(al1l2(r));
    FL1L2y(r) = Cy(r) + scalefl1l2(r)*sin(al1l2(r));

    %%%% Plotting Linkage %%%%%
    plot([Dx(r) Ax(r)], [Dy(r) Ay(r)], "k", [Cx(r) Dx(r)], [Cy(r) Dy(r)], "k", [Ex(r) Fx(r)], [Ey(r) Fy(r)], "k", [Ex(r) C2x(r)], [Ey(r) C2y(r)], "k", [Ex(r) F2x(r)], [Ey(r) F2y(r)], "-b", [Dx(r) F1x(r)], [Dy(r) F1y(r)], "-r", [Cx(r) F1Gx(r)], [Cy(r) F1Gy(r)], "-b", [Cx(r) FG2x(r)], [Cy(r) FG2y(r)], "-r", [Cx(r) FL1L2x(r)], [Cy(r) FL1L2y(r)], "-m", 'Linewidth', 1);
    title('Joint Force Visualization during Linear Pathing')
    xlabel('X [mm]')
    ylabel("Y [mm]")

    hold on;
    grid on;
    axis ([-150 500 -100 600]);
    drawnow;
    hold off;


end

