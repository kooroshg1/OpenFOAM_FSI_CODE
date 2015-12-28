clc;
clear all;
close all;
format short g;
fclose all;
% ----------------------------------------------------------------------- %
% cd /media/koorosh/koorosh/Matlab/Joukowsky_airfoil/openFOAM/FSI_simulation
currentFolder = pwd;
% ----------------------------------------------------------------------- %
x0 = -0.3;
y0 = 0.1;
Xm = -0.125; % x-coordinate of the mounting point
Ym = 0.0; % y-coordinate of the mounting point
AoA0 = 2.0;
Uinfty = 1;
maxIter = 10;

AoA = zeros(maxIter+1,1); AoA(1) = AoA0;
FX = zeros(maxIter,1);
FY = zeros(maxIter,1);
MZ = zeros(maxIter,1);
DISP = [];
THETA = [];

% Uinfty_DOE = linspace(10,70,10);
% AoA0_DOE = linspace(1,7,10);
% 
% [Uinfty_DOE,AoA0_DOE] = meshgrid(Uinfty_DOE,AoA0_DOE);
% Uinfty_DOE = reshape(Uinfty_DOE,[],1);
% AoA0_DOE = reshape(AoA0_DOE,[],1);

X0_DOE = linspace(-0.01,-0.1,15);
Y0_DOE = linspace(0.25,0.3,15);
[Y0_DOE,X0_DOE] = meshgrid(Y0_DOE,X0_DOE);
X0_DOE = reshape(X0_DOE,[],1);
Y0_DOE = reshape(Y0_DOE,[],1);

try
rmdir('DOEresult','s');
catch
end
mkdir('DOEresult');
% for DOEstep=1:length(X0_DOE)
for DOEstep=1:1
x0 = -0.1;
y0 = 0.1;

% x0 = X0_DOE(DOEstep);
% y0 = Y0_DOE(DOEstep);

% Xm = -0.25;
% Ym = Ym_DOE(DOEstep);

% Initialize storage matrices
AoA = zeros(maxIter+1,1); AoA(1) = AoA0;
FX = zeros(maxIter,1);
FY = zeros(maxIter,1);
MZ = zeros(maxIter,1);
DISP = [];
THETA = [];

disp(['Xm = ' num2str(Xm)])
disp(['          Fx          Fy          Mz               AoA          U'])
tic
for iter = 1:maxIter
    %% Generate OpenFOAM Folder
    [stat, mess, id] = rmdir('OpenFOAM_joukowski_airfoil_simulation','s');
    copyfile('OpenFOAM_joukowski_airfoil_simulation_template',...
             'OpenFOAM_joukowski_airfoil_simulation');

    %% Generate OpenFOAM Mesh
    cd OpenFOAM_blockMeshGenerator/
    openFOAM_joukowsky_airfoil_generator(x0,y0);
    copyfile('blockMeshDict',...
             [currentFolder '/OpenFOAM_joukowski_airfoil_simulation/constant/polyMesh']);
    cd ..

    %% Update Angle of Attack (AoA)
    updateAoA(AoA(iter),Uinfty,currentFolder);
    
    %% Run OpenFOAM Simulation
    cd OpenFOAM_joukowski_airfoil_simulation/
    [~,~] = system('blockMesh');
    [~,~] = system('checkMesh');
    [~,~] = system('simpleFoam');
    [~,~] = system('sample');
%     system('blockMesh');
%     system('checkMesh');
%     system('simpleFoam');
%     system('sample');
    cd ..

    %% Calculate FSI respose
    cd matlab_calculate_aerodynamic_load_and_moment/
    [Fx,Fy,M,U,theta] = calcFSI(Xm,Ym,AoA(iter),currentFolder);
    
%     h2 = figure(2);
%     subplot(2,1,1)
%     plot(linspace(0,1,length(U)),U)
%     title('beam displacement')
%     xlabel('X')
%     ylabel('Disp')
%     axis equal
%     hold on
%     subplot(2,1,2)
%     plot(linspace(0,1,length(theta)),theta)
%     title('beam rotation')
%     xlabel('X')
%     ylabel('\theta')
%     axis equal
%     hold on
    
    AoA(iter+1) = AoA0 + atan(theta(end)) * 180 / pi;
    FX(iter) = Fx;
    FY(iter) = Fy;
    MZ(iter) = M;
    % ------------------------------------------------------------------- %
    disp([Fx Fy M AoA(iter+1) U(end)])
    % ------------------------------------------------------------------- %
    DISP = [DISP,U];
    THETA = [THETA,theta];
%     disp('copy your file, then presse any key to continue!')
%     pause
    if abs(AoA(iter+1)-AoA(iter)) < 1e-3
        disp('convergence reached')
        break;
    end
    cd ..
end
% clf(h2)
cd(currentFolder)
elapsed_time = toc;
dlmwrite('case2.txt',[FX,FY,MZ,AoA(2:end)])
% dlmwrite(['DOEresult/' num2str(DOEstep)],[FX,FY,MZ,AoA(2:end)])
% dlmwrite(['DOEresult/' num2str(DOEstep)],[x0,y0,0,0],'-append')
% dlmwrite(['DOEresult/' num2str(DOEstep)],[elapsed_time,0,0,0],'-append')
end