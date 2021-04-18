%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% reading data in matlab

clear all;
close all;
clc;

fileID = fopen('ver1MCK.txt','r');
formatSpec = '%f %f %f %f';
sizeA = [4 Inf];
systemResult = fscanf(fileID,formatSpec,sizeA);
systemResult = systemResult';

fclose(fileID);

%% new creation

x = systemResult(:,1);
xRef = systemResult(:,2);
u = systemResult(:,3);
t = systemResult(:,4);

figure
plot(t,x)
hold on
plot(t,xRef)
legend("Pos","Ref Pos")
xlabel("Time (sec)")
ylabel("Position (meter)")

figure
plot(t,u)
