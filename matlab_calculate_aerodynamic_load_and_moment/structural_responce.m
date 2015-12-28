function [U,theta] = structural_responce(M,Fy,AoA)
% Beam data
L = 1.0;
E = 200e9 / 50;
b = 0.01;
h = 0.01;

I = (1/12) * b * h^3;
x = linspace(0,L,100)';

theta = -M * x / (E * I) + (6 * Fy * L * x - 3 * Fy .* x.^2) / (6 * E * I);
U = -M * x.^2 / (2 * E * I) + Fy * x.^2 .* (3 * L - x) / (6 * E * I);
