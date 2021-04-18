%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% reading data in matlab

clear all;
close all;
clc;

fileID = fopen('ver1MCK.txt','r');
formatSpec = '%f %f %f %f %f';
sizeA = [5 Inf];
systemResult = fscanf(fileID,formatSpec,sizeA);
systemResult = systemResult';

fclose(fileID);

%% new creation
x1 = systemResult(:,1);
x2 = systemResult(:,2);
xRef = systemResult(:,3);
u = systemResult(:,4);
t = systemResult(:,5);

figure
plot(t,x1)
hold on
plot(t,xRef)
legend("Pos","Ref Pos")
xlabel("Time (sec)")
ylabel("Position (meter)")

figure
plot(t,u)
