clc;
clear all;
close all;
format short g;
fclose all;
% ----------------------------------------------------------------------- %
[Fx,Fy,M] = calcAero_OpenFOAM();
[U,theta] = structural_responce(M,Fy);