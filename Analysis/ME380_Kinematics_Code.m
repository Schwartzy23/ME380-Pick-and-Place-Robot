%Matlab code for 380 Kinematics
clear;
clc;

%%%%%%% Initiating variables %%%%%%%%%
%%%%%% Constant %%%%%%%

b1 = 61;
b2 = 396;
L1 = 405;
L2 = 405;
beta1 = 0;
beta2 = 0;
gamma1 = 90;
gamma2 = 90;

%%% Important inputs %%%
theta0 = 0;

r0 = 1;
dr0 = 5;
ddr0 = 0;

%%%%%%% Variable %%%%%%
%y1, y2, lambda1, lambda2.

%%%%%% Initiate list of variables %%%%%%%%%

rlist = [];

y1_list = [];
y2_list = [];
lambda1_list = [];
lambda2_list = [];

dy1_list = [];
dy2_list = [];
dlambda1_list = [];
dlambda2_list = [];

ddy1_list = [];
ddy2_list = [];
ddlambda1_list = [];
ddlambda2_list = [];


for r = r0:r0+335
    rlist(end+1)=r;
    
    %%%%%% lambda1 %%%%%%
    lambda1 = 360-acosd((r*cosd(theta0)+b1)/L1);
    lambda1_list(end+1)=lambda1;
    %%%%%% lambda2 %%%%%%
    lambda2 = 360-acosd((r*cosd(theta0)-b2)/L2);
    lambda2_list(end+1)=lambda2;
    
    %%%%%% y1 %%%%%%
    y1 = r*sind(theta0)-L1*sind(lambda1);
    y1_list(end+1)=y1;
    %%%%%% y2 %%%%%%
    y2 = r*sind(theta0)-L2*sind(lambda2);
    y2_list(end+1)=y2;

    %%%%%%%%%%%%%%% First Order Derivatives %%%%%%%%%%%%%%%%%%%%%%%
    %%%%%% d/dt lambda1 %%%%%%
    dlambda1 = (-dr0*cosd(theta0))/(L1*sind(lambda1));
    dlambda1_list(end+1)=dlambda1;
    %%%%%% d/dt lambda2 %%%%%%
    dlambda2 = (-dr0*cosd(theta0))/(L2*sind(lambda2));
    dlambda2_list(end+1)=dlambda2;
    %%%%%% d/dt y1 %%%%%%
    dy1 = dr0*sind(theta0)-L1*cosd(lambda1)*dlambda1;
    dy1_list(end+1)=dy1;
    %%%%%% d/dt y2 %%%%%%
    dy2 = dr0*sind(theta0)-L2*cosd(lambda2)*dlambda2;
    dy2_list(end+1)=dy2;


    %%%%%%%%%%%%%%% Second Order Derivatives %%%%%%%%%%%%%%%%%%%%%%%
    %%%%%% d2/dt2 lambda1 %%%%%%
    ddlambda1 = -(L1*cosd(lambda1)*dlambda1^2 + ddr0*cosd(theta0))/L1/sind(lambda1);
    ddlambda1_list(end+1)=ddlambda1;
    %%%%%% d2/dt2 lambda2 %%%%%%
    ddlambda2 = -(L2*cosd(lambda2)*dlambda2^2 + ddr0*cosd(theta0))/L2/sind(lambda2);
    ddlambda2_list(end+1)=ddlambda2;
    %%%%%% d2/dt2 y1 %%%%%%
    ddy1 = ddr0*sind(theta0) - L1*cosd(lambda1)*ddlambda1 + L1*sind(lambda1)*dlambda1^2;
    ddy1_list(end+1)=ddy1;
    %%%%%% d2/dt2 y2 %%%%%%
    ddy2 = ddr0*sind(theta0) - L2*cosd(lambda2)*ddlambda2 + L2*sind(lambda2)*dlambda2^2;
    ddy2_list(end+1)=ddy2;

end


%%%%%%%%% Variable to Plot %%%%%%%%%%%%

variables = [lambda1_list; lambda2_list; y1_list; y2_list; dlambda1_list; dlambda2_list; dy1_list; dy2_list; ddlambda1_list; ddlambda2_list; ddy1_list; ddy2_list ];
%var_to_plot = lambda1_list;


for i =1:1:12
    
    
    figure(i)
    plot(rlist, variables(i,:))
    grid on;
    title("Variable #"+i)
end




