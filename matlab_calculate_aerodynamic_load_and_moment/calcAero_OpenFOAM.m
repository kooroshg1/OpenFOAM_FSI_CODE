function [Fx,Fy,M] = calcAero_OpenFOAM(Xm,Ym,AoA,currentFolder)
% clc;
% clear all;
% format short g;
% close all;
% % ----------------------------------------------------------------------- %
% 
% currentFolder = '/home/koorosh/Desktop/FSI_simulation/';
% Xm = -0.25;
% Ym = 0.0;
% AoA = 15.0;
numberOfStepsToAverage = 2;

% Reads surface pressure data from sample directory
files = dir([currentFolder '/OpenFOAM_joukowski_airfoil_simulation/postProcessing/surfaces/']);
fileName = zeros(1,length(files));

for i=3:length(files)
    fileName(i) = str2num(files(i).name);
end

fileName = sort(fileName');
surfacePressureHistory = zeros(180,numberOfStepsToAverage);
X = zeros(180,1);
Y = zeros(180,1);
Z = zeros(180,1);
fileNumber = 1;
for fileNameIndex = (length(fileName) - numberOfStepsToAverage + 1):length(fileName)
    fRead = fopen([currentFolder '/OpenFOAM_joukowski_airfoil_simulation/postProcessing/surfaces/' num2str(fileName(fileNameIndex)) '/p_airfoil.raw'],'r');
    fLine = fgetl(fRead);
    fLine = fgetl(fRead);
    lineNumber = 1;
    while ~feof(fRead)
        fLine = fgetl(fRead);
        fLineNum = strread(fLine);
        X(lineNumber) = fLineNum(1);
        Y(lineNumber) = fLineNum(2);
        Z(lineNumber) = fLineNum(3);
        surfacePressureHistory(lineNumber,fileNumber) = fLineNum(4);
        lineNumber = lineNumber + 1;
        %fprintf(fWrite,'%s\n',fLine);
    end
    fileNumber = fileNumber + 1;
end
surfacePressure = transpose(mean(transpose(surfacePressureHistory)));
%{
% Reorder point on the airfoil so that the sequence is right
% positive = true
% negative = false
vectorCheck = zeros(length(X),2);
for i=1:length(X)
    if X(i) > 0
        vectorCheck(i,1) = true;
    end
    if Y(i) > 0
        vectorCheck(i,2) = true;
    end
end

frontTop = intersect(find(vectorCheck(:,1) == false),...
                     find(vectorCheck(:,2) == true));
backTop = intersect(find(vectorCheck(:,1) == true),...
                     find(vectorCheck(:,2) == true));
frontBottom = intersect(find(vectorCheck(:,1) == false),...
                     find(vectorCheck(:,2) == false));
backBottom = intersect(find(vectorCheck(:,1) == true),...
                     find(vectorCheck(:,2) == false));
                 
frontTopData = [X(frontTop) Y(frontTop) surfacePressure(frontTop)];
frontTopData = sortrows(frontTopData);
frontTopCoordinate = frontTopData(:,[1,2]);
frontTopSurfacePressure = frontTopData(:,3);

backTopData = [X(backTop) Y(backTop) surfacePressure(backTop)];
backTopData = sortrows(backTopData)theta;
backTopCoordinate = backTopData(:,[1,2]);
backTopSurfacePressure = backTopData(:,3);

backBottomData = [X(backBottom) Y(backBottom) surfacePressure(backBottom)];
backBottomData = sortrows(backBottomData);
backBottomData = flipud(backBottomData);
backBottomCoordinate = backBottomData(:,[1,2]);
backBottomSurfacePressure = backBottomData(:,3);

frontBottomData = [X(frontBottom) Y(frontBottom) surfacePressure(frontBottom)];
frontBottomData = sortrows(frontBottomData);
frontBottomData = flipud(frontBottomData);
frontBottomCoordinate = frontBottomData(:,[1,2]);
frontBottomSurfacePressure = frontBottomData(:,3);


X = [frontTopCoordinate(:,1);...
     backTopCoordinate(:,1);...
     backBottomCoordinate(:,1);...
     frontBottomCoordinate(:,1)];

Y = [frontTopCoordinate(:,2);...
     backTopCoordinate(:,2);...
     backBottomCoordinate(:,2);...
     frontBottomCoordinate(:,2)];
 
P = [frontTopSurfacePressure(:,1);...
     backTopSurfacePressure(:,1);...
     backBottomSurfacePressure(:,1);...
     frontBottomSurfacePressure(:,1)];
%}
% ----------------------------------------------------------------------- %
coord = [X,Y,surfacePressure];
[ind05,val05] = find(abs(coord(:,1) - 0.5) < 0.01);

[indN05,valN05] = find(abs(coord(:,1) + 0.5) < 0.01);
% break
if min(ind05) < min(indN05)
    top = sortrows(coord(min(ind05):min(indN05),:),1);
    bottom = sortrows(coord(min(indN05):end,:),1);
else
    top = coord(min(indN05):min(ind05),:);
    bottom = flipud(sortrows(coord(min(ind05):end,:),1));
end
% top = sortrows(top,1);
% bottom = sortrows(bottom,1);
% break
% ----------------------------------------------------------------------- %
X = [top(:,1);bottom(:,1)];
Y = [top(:,2);bottom(:,2)];
P = [top(:,3);bottom(:,3)];

% Calculate normal vector at each node on the airfoil
normVec = zeros(length(X),1);
nodeArea = zeros(length(X),1);
for i=1:length(X)
    if i == 1
        tangVec = [X(i+1) - X(end),Y(i+1) - Y(end)];
    elseif i == length(X)
        tangVec = [X(1) - X(end-1),Y(1) - Y(end-1)];
    else
        tangVec = [X(i+1) - X(i-1),Y(i+1) - Y(i-1)];      
    end
    nodeArea(i) = norm(tangVec);
    tangVec = tangVec / norm(tangVec);
    if isnan(tangVec)
        tangVec(1) = 0;
        tangVec(2) = 0;
    end
    normVec(i,1) = -tangVec(2);
    normVec(i,2) = tangVec(1);
%     cross([normVec(i,:),0],[tangVec,0])
end


% figure(1)
% scatter(X,Y,'o')
% hold on
% for i=1:length(X)
%     plot(X(i),Y(i),'r+')
%     pause(0.1)
% end
% for i=1:length(top)
%     plot(top(i,1),top(i,2),'r+')
%     pause(0.1)
% end
    

% figure,
% plot(X,Y,'o')
% hold on
% quiver(X,Y,normVec(:,1),normVec(:,2))

F = zeros(length(P),2);
for i = 1:length(F) 
    F(i,:) = (-P(i) * nodeArea(i)) * normVec(i,:);
end

Fx = sum(F(:,1));
Fy = sum(F(:,2));

Fx = Fx * cosd(abs(AoA)) + Fy * sind(abs(AoA));
Fy = Fx * sind(abs(AoA)) + Fy * cosd(abs(AoA));

% Xm = 0; % x-coordinate of the mounting point
% Ym = 0; % y-coordinate of the mounting point

nodeVector = [X-Xm,Y-Ym];
Mz = cross([nodeVector,zeros(length(nodeVector),1)],[F,zeros(length(nodeVector),1)]);
M = sum(Mz(:,3));

% figure,
% plot(X,P)
% 
% figure,
% plot(X,F(:,1))