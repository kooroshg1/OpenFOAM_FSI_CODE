function updateAoA(AoA,Uinfty,currentFolder)

cd OpenFOAM_joukowski_airfoil_simulation/0
copyfile('U','U_');
% Generate input file for OpenFOAM
frID = fopen('U_','r');
fwID = fopen('U','w');

frRead = fgetl(frID);
fprintf(fwID,'%s\n',frRead);
while ~feof(frID)
    frRead = fgetl(frID);
    if strcmp(frRead,'internalField   uniform (10 0 0);')
        fprintf(fwID,'%s\n',['internalField   uniform (' num2str(Uinfty*cosd(AoA)) ' ' num2str(Uinfty*sind(AoA)) ' ' num2str(0) ');']);
        continue;
    end
    if strcmp(frRead,'        value uniform (10 0 0); // CHANGE THIS')
        fprintf(fwID,'%s\n',['        value uniform (' num2str(Uinfty*cosd(AoA)) ' ' num2str(Uinfty*sind(AoA)) ' ' num2str(0) '); // CHANGE THIS']);
        continue;
    end
    fprintf(fwID,'%s\n',frRead);
end
delete('U_')
cd(currentFolder)