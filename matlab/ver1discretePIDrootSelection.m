%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% discrete PID controller

clear all;
close all;
clc;

syms z r1 r2 r3;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% characteristic equation

CE_disc = (z - r1)*(z - r2)*(z - r3);

pretty(collect(CE_disc))


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% CE_disc value test
r1 = 0.1;
r2 = 0.2;
r3 = 0.3;

CE_disc_value = (z - r1)*(z - r2)*(z - r3);

pretty(collect(CE_disc_value))

CE_discCoeff = coeffs(CE_disc_value);

firstTerm = double(CE_discCoeff(1,4));
secondTerm = double(CE_discCoeff(1,3));
thirdTerm = double(CE_discCoeff(1,2));
fourthTerm = double(CE_discCoeff(1,1));

newRoots = [firstTerm secondTerm thirdTerm fourthTerm]

roots(newRoots)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% real characteristic equation

r1 = 0.1;
r2 = 0.2;
r3 = 0.3;

dt = 0.001;

%%% kp kd ki calculation

Kd = -r1*r2*r3/(dt)

Kp = -(r3*(r2 + r1) + r1*r2 + 2*Kd*dt)/(dt)

Ki = -(r3 + r2 + r1 + Kd*dt + Kp*dt)/(dt)

firstRealTerm = 1;
secondRealTerm = Ki*dt + Kp*dt + Kd*dt;
thirdRealTerm = -Kp*dt - 2*Kd*dt;
fourthRealTerm = Kd*dt;

realCEequation = [firstRealTerm secondRealTerm thirdRealTerm fourthRealTerm];

roots(realCEequation)



























