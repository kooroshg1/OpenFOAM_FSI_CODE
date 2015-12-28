function [Fx,Fy,M,U,theta] = calcFSI(Xm,Ym,AoA,currentFolder)
[Fx,Fy,M] = calcAero_OpenFOAM(Xm,Ym,AoA,currentFolder);
[U,theta] = structural_responce(M,Fy);