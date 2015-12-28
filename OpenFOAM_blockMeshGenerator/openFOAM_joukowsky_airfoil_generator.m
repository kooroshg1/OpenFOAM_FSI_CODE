% clc;
% close all;
% clear all;
% fclose all;
% 
% x0 = -0.01;
% y0 = 0.0;
% X0 = linspace(-0.1,-0.9,10);
% Y0 = linspace(0.1,0.9,10);
% [X0,Y0] = meshgrid(X0,Y0);
% X0 = reshape(X0,[],1);
% Y0 = reshape(Y0,[],1);
% for DOEstep=1:length(X0)
%     clearvars -except X0 Y0 DOEstep
%     x0 = X0(DOEstep)
%     y0 = Y0(DOEstep)
% ----------------------------------------------------------------------- %
function flag = openFOAM_joukowsky_airfoil_generator(x0,y0)
% ----------------------------------------------------------------------- %
% Joukowski showed that the image of a circle passing through (1.0,0) and
% containing the point (-1,0) is mapped onto a curve shaped like the
% cross section of an airplane wing.

N = 60; % Number of points on airfoil surface
% x0 = -0.1; % Defines maximum thickness. [-1 0]
% y0 = 0.1; % Defines camber. Positive values for up camber [0 1]
b = sqrt(0.9);
R = sqrt((1 - x0)^2 + y0^2); % Calculate R based on (1.0,0) on circle
% Checking the (-1,0) is inside
if (1 + x0)^2 + y0^2 - R^2 >= 0
    disp(['Cannot generate airfoil. Please check your design variables']);
%     return;
end

theta = linspace(0,2*pi,N);

x = x0 + R .* cos(theta);
y = y0 + R .* sin(theta);
z = x + 1i * y;

z = z + b ./ z; % Joukowski transform
X = real(z);
Y = imag(z);

Y = Y / (max(X) - min(X));
X = X / (max(X) - min(X));
X = X + 0.5 - max(X);
[a,b] = max(X);
Y = Y - Y(b);
Y = Y - mean(Y);
[max(X) min(X) max(Y) min(Y)];
X = transpose(X);
Y = transpose(Y);
% ----------------------------------------------------------------------- %

% ----------------------------------------------------------------------- %
coord = [X Y];

removeRow = [];
for i=2:size(coord,1)
    if abs(coord(i,1) - coord(i-1,1)) < 0.0001
        removeRow = [removeRow,i];
    end
end

[ind05,val05] = find(abs(coord(:,1) - 0.5) < 0.01);
[val,indTemp] = max(coord(ind05,1));
ind05 = ind05(indTemp);

[indN05,valN05] = find(abs(coord(:,1) + 0.5) < 0.01);
[val,indTemp] = max(abs(coord(indN05,1)));
indN05 = indN05(indTemp);

if ind05 > indN05
    coord = [coord(ind05:end,:);coord(1:ind05-1,:)];
else
    coord = [coord(indN05:end,:);coord(1:indN05-1,:)];
end

removeRow = [];
for i=2:size(coord,1)
    if abs(coord(i,1) - coord(i-1,1)) < 0.0001
        removeRow = [removeRow,i];
    end
end
coord(removeRow,:) = [];

% ----------------------------------------------------------------------- %
[ind05,val05] = find(abs(coord(:,1) - 0.5) < 0.01);
[val,indTemp] = max(coord(ind05,1));
ind05 = ind05(indTemp);

[indN05,valN05] = find(abs(coord(:,1) + 0.5) < 0.01);
[val,indTemp] = max(abs(coord(indN05,1)));
indN05 = indN05(indTemp);

if ind05 < indN05
    top = coord(1:indN05,:);
    bottom = coord(indN05:end,:);
    bottom = [bottom;coord(ind05,:)];
else
    top = coord(ind05:end,:);
    top = [top;coord(indN05,:)];
    bottom = coord(1:ind05,:);
end
    
top = flipud(top);
[val,indTop05] = min(abs(top(:,1) + 0.25));

[val,indBottom05] = min(abs(bottom(:,1) + 0.25));

front_node = (top(1,:) + bottom(1,:)) / 2;
back_node = (top(end,:) + bottom(end,:)) / 2;
middle_top_node = top(indTop05,:);
middle_bottom_node = bottom(indBottom05,:);

back_top = top(indTop05+1:end-1,:);
back_bottom = bottom(indBottom05+1:end-1,:);

front_top = top(2:indTop05-1,:);
front_bottom = bottom(2:indBottom05-1,:);

mesh_angle_vector = front_node - (middle_top_node + middle_bottom_node) / 2;
mesh_angle = atan(mesh_angle_vector(2) / mesh_angle_vector(1)) * 180 / pi;
% figure(1),
% plot(top(:,1),top(:,2),'ro',...
%      bottom(:,1),bottom(:,2),'ko',...
%      top(indTop05,1),top(indTop05,2),'r+',...
%      bottom(indBottom05,1),bottom(indBottom05,2),'k+',...
%      front_node(:,1),front_node(:,2),'g+',...
%      back_node(:,1),back_node(:,2),'y+')
% axis equal
% pause(0.3)
% clf

% break
%{
[a,Xmax] = max(X);
[a,Xmin] = min(X);
[a,Ymax] = max(Y);
[a,Ymin] = min(Y);
if (coord(Xmin,1) == coord(Xmin+1,1))
    coord(Xmin,2) = 0.5 * ( coord(Xmin,2) + coord(Xmin+1,2) );
    coord(Xmin+1,:) = [];
end
if (coord(Xmax,1) == coord(end,1))
    coord(Xmax,2) = 0.5 * ( coord(Xmax,2) + coord(end,2) );
    coord(end,:) = [];
end

coord_index = sort([Xmax,Xmin,Ymax,Ymin]);

% Group the locations of points on the airfoil
if coord_index(1) ~= 1
    back_node = coord(Xmax,:);
    back_top = [coord(coord_index(4)+1:end,:);coord(1:coord_index(1)-1,:)];
    middle_top_node = coord(Ymax,:);
    front_top = coord(coord_index(1)+1:coord_index(2)-1,:);
    front_node = coord(Xmin,:);
    front_bottom = coord(coord_index(2)+1:coord_index(3)-1,:);
    middle_bottom_node = coord(Ymin,:);
    back_bottom = coord(coord_index(3)+1:coord_index(4)-1,:)  ;
else
    back_node = coord(Xmax,:);
    back_top = coord(coord_index(1)+1:coord_index(2)-1,:);
    middle_top_node = coord(Ymax,:);
    front_top = coord(coord_index(2)+1:coord_index(3)-1,:);
    front_node = coord(Xmin,:);
    front_bottom = coord(coord_index(3)+1:coord_index(4)-1,:);
    middle_bottom_node = coord(Ymin,:);
    back_bottom = coord(coord_index(4)+1:end,:);
end
%}
% Generate the location of the surrounding domain
R = 15; % Radius of the front of the domain
L = 30; % Length behind airfoil
p0 = [front_node(1) front_node(2)] + R * [cosd(180+mesh_angle) sind(180+mesh_angle)];
r_front_top = R + abs(front_node(1)) - abs(middle_top_node(1));
p1 = [middle_top_node(1) r_front_top + front_node(2)];
p0p1 = [p1(1) + r_front_top * cos([linspace(pi/2,pi,10)]') ...
        p0(2) + r_front_top * sin([linspace(pi/2,pi,10)]')];
p2 = [back_node(1) p1(2)];
p3 = [back_node(1) + L p1(2)];
p4 = [p3(1) back_node(2)];

r_front_bottom = R + abs(front_node(1)) - abs(middle_bottom_node(1));
p7 = [middle_bottom_node(1) p0(2) - r_front_bottom];
p0p7 = [p7(1) + r_front_bottom * cos([linspace(pi,3*pi/2,10)]') ...
        p0(2) + r_front_bottom * sin([linspace(pi,3*pi/2,10)]')];
p6 = [back_node(1) p7(2)];
p5 = [p3(1) p6(2)];



% plotThis = [p0;p1;p2;p3;p4;p5;p6;p7];
% figure,
% scatter(plotThis(:,1),plotThis(:,2))
% hold on
% scatter(p0p1(:,1),p0p1(:,2),'+')
% hold on
% scatter(p0p7(:,1),p0p7(:,2),'*')
% hold on
% plot(X,Y,'.')
% axis([-2 2 -2 2])
% break
% front_top = flipud(front_top);
% back_top = flipud(back_top);

% Generate input file for OpenFOAM
frID = fopen('blockMeshDict_template','r');
fwID = fopen('blockMeshDict','w');
frRead = fgetl(frID);
fprintf(fwID,'%s\n',frRead);
while ~feof(frID)
    frRead = fgetl(frID);
    if strcmp(frRead,'vertices')
        fprintf(fwID,'%s\n',frRead);
        frRead = fgetl(frID);
        fprintf(fwID,'%s\n',frRead);
        fprintf(fwID,'%s\n','    // coordinates of the outer domain');
        fprintf(fwID,'%s\n',['    (' num2str(p0(1)) ' ' num2str(p0(2)) ' ' num2str(0) ')']);
        fprintf(fwID,'%s\n',['    (' num2str(p1(1)) ' ' num2str(p1(2)) ' ' num2str(0) ')']);
        fprintf(fwID,'%s\n',['    (' num2str(p2(1)) ' ' num2str(p2(2)) ' ' num2str(0) ')']);
        fprintf(fwID,'%s\n',['    (' num2str(p3(1)) ' ' num2str(p3(2)) ' ' num2str(0) ')']);
        fprintf(fwID,'%s\n',['    (' num2str(p4(1)) ' ' num2str(p4(2)) ' ' num2str(0) ')']);
        fprintf(fwID,'%s\n',['    (' num2str(p5(1)) ' ' num2str(p5(2)) ' ' num2str(0) ')']);
        fprintf(fwID,'%s\n',['    (' num2str(p6(1)) ' ' num2str(p6(2)) ' ' num2str(0) ')']);
        fprintf(fwID,'%s\n',['    (' num2str(p7(1)) ' ' num2str(p7(2)) ' ' num2str(0) ')']);
        fprintf(fwID,'%s\n','    // Airfoil key coordinates');
        fprintf(fwID,'%s\n',['    (' num2str(front_node(1)) ' ' num2str(front_node(2)) ' ' num2str(0) ')']);
        fprintf(fwID,'%s\n',['    (' num2str(middle_top_node(1)) ' ' num2str(middle_top_node(2)) ' ' num2str(0) ')']);
        fprintf(fwID,'%s\n',['    (' num2str(back_node(1)) ' ' num2str(back_node(2)) ' ' num2str(0) ')']);
        fprintf(fwID,'%s\n',['    (' num2str(middle_bottom_node(1)) ' ' num2str(middle_bottom_node(2)) ' ' num2str(0) ')']);
        fprintf(fwID,'%s\n','    // ************************************************ // ');
        fprintf(fwID,'%s\n','    // coordinates of the outer domain');
        fprintf(fwID,'%s\n',['    (' num2str(p0(1)) ' ' num2str(p0(2)) ' ' num2str(1) ')']);
        fprintf(fwID,'%s\n',['    (' num2str(p1(1)) ' ' num2str(p1(2)) ' ' num2str(1) ')']);
        fprintf(fwID,'%s\n',['    (' num2str(p2(1)) ' ' num2str(p2(2)) ' ' num2str(1) ')']);
        fprintf(fwID,'%s\n',['    (' num2str(p3(1)) ' ' num2str(p3(2)) ' ' num2str(1) ')']);
        fprintf(fwID,'%s\n',['    (' num2str(p4(1)) ' ' num2str(p4(2)) ' ' num2str(1) ')']);
        fprintf(fwID,'%s\n',['    (' num2str(p5(1)) ' ' num2str(p5(2)) ' ' num2str(1) ')']);
        fprintf(fwID,'%s\n',['    (' num2str(p6(1)) ' ' num2str(p6(2)) ' ' num2str(1) ')']);
        fprintf(fwID,'%s\n',['    (' num2str(p7(1)) ' ' num2str(p7(2)) ' ' num2str(1) ')']);
        fprintf(fwID,'%s\n','    // Airfoil key coordinates');
        fprintf(fwID,'%s\n',['    (' num2str(front_node(1)) ' ' num2str(front_node(2)) ' ' num2str(1) ')']);
        fprintf(fwID,'%s\n',['    (' num2str(middle_top_node(1)) ' ' num2str(middle_top_node(2)) ' ' num2str(1) ')']);
        fprintf(fwID,'%s\n',['    (' num2str(back_node(1)) ' ' num2str(back_node(2)) ' ' num2str(1) ')']);
        fprintf(fwID,'%s\n',['    (' num2str(middle_bottom_node(1)) ' ' num2str(middle_bottom_node(2)) ' ' num2str(1) ')']);
        continue;
    end
    if strcmp(frRead,'edges')
        fprintf(fwID,'%s\n',frRead);
        frRead = fgetl(frID);
        fprintf(fwID,'%s\n',frRead);
        fprintf(fwID,'%s\n','    // Arc for the front');
        fprintf(fwID,'%s\n',['    arc 0 1 (' num2str(p0p1(5,1)) ' ' num2str(p0p1(5,2)) ' ' num2str(0) ')']);
        fprintf(fwID,'%s\n',['    arc 12 13 (' num2str(p0p1(5,1)) ' ' num2str(p0p1(5,2)) ' ' num2str(1) ')']);
        fprintf(fwID,'%s\n',['    arc 0 7 (' num2str(p0p7(5,1)) ' ' num2str(p0p7(5,2)) ' ' num2str(0) ')']);
        fprintf(fwID,'%s\n',['    arc 12 19 (' num2str(p0p7(5,1)) ' ' num2str(p0p7(5,2)) ' ' num2str(1) ')']);
        % =============================================================== %
        fprintf(fwID,'%s\n','    // Define airfoil top-front surface // ');
        fprintf(fwID,'%s\n','    spline 8 9');
        fprintf(fwID,'%s\n','    (');
        for in=1:length(front_top)
            fprintf(fwID,'%s\n',['        (' num2str(front_top(in,1)) ' ' num2str(front_top(in,2)) ' ' num2str(0) ')']);
        end
        fprintf(fwID,'%s\n','    )');
        % ---------------------------------------------------------------
        fprintf(fwID,'%s\n','    spline 20 21');
        fprintf(fwID,'%s\n','    (');
        for in=1:length(front_top)
            fprintf(fwID,'%s\n',['        (' num2str(front_top(in,1)) ' ' num2str(front_top(in,2)) ' ' num2str(1) ')']);
        end
        fprintf(fwID,'%s\n','    )');
        % =============================================================== %
        fprintf(fwID,'%s\n','    // Define airfoil top-back surface // ');
        fprintf(fwID,'%s\n','    spline 9 10');
        fprintf(fwID,'%s\n','    (');
        for in=1:length(back_top)
            fprintf(fwID,'%s\n',['        (' num2str(back_top(in,1)) ' ' num2str(back_top(in,2)) ' ' num2str(0) ')']);
        end
        fprintf(fwID,'%s\n','    )');
        % ---------------------------------------------------------------
        fprintf(fwID,'%s\n','    spline 21 22');
        fprintf(fwID,'%s\n','    (');
        for in=1:length(back_top)
            fprintf(fwID,'%s\n',['        (' num2str(back_top(in,1)) ' ' num2str(back_top(in,2)) ' ' num2str(1) ')']);
        end
        fprintf(fwID,'%s\n','    )');
        % =============================================================== %
        fprintf(fwID,'%s\n','    // Define airfoil bottom-front surface // ');
        fprintf(fwID,'%s\n','    spline 8 11');
        fprintf(fwID,'%s\n','    (');
        for in=1:length(front_bottom)
            fprintf(fwID,'%s\n',['        (' num2str(front_bottom(in,1)) ' ' num2str(front_bottom(in,2)) ' ' num2str(0) ')']);
        end
        fprintf(fwID,'%s\n','    )');
        % ---------------------------------------------------------------
        fprintf(fwID,'%s\n','    spline 20 23');
        fprintf(fwID,'%s\n','    (');
        for in=1:length(front_bottom)
            fprintf(fwID,'%s\n',['        (' num2str(front_bottom(in,1)) ' ' num2str(front_bottom(in,2)) ' ' num2str(1) ')']);
        end
        fprintf(fwID,'%s\n','    )');
        % =============================================================== %
        fprintf(fwID,'%s\n','    // Define airfoil bottom-back surface // ');
        fprintf(fwID,'%s\n','    spline 11 10');
        fprintf(fwID,'%s\n','    (');
        for in=1:length(back_bottom)
            fprintf(fwID,'%s\n',['        (' num2str(back_bottom(in,1)) ' ' num2str(back_bottom(in,2)) ' ' num2str(0) ')']);
        end
        fprintf(fwID,'%s\n','    )');
        % ---------------------------------------------------------------
        fprintf(fwID,'%s\n','    spline 23 22');
        fprintf(fwID,'%s\n','    (');
        for in=1:length(back_bottom)
            fprintf(fwID,'%s\n',['        (' num2str(back_bottom(in,1)) ' ' num2str(back_bottom(in,2)) ' ' num2str(1) ')']);
        end
        fprintf(fwID,'%s\n','    )');
        continue;
    end
    fprintf(fwID,'%s\n',frRead);
end
fclose all;

% Generate a sampeling file for OpenFOAM
sampleTop = [front_top;middle_top_node;back_top];
sampleBottom = [front_bottom;middle_bottom_node;back_bottom];
frID = fopen('sampleDict_template','r');
fwID = fopen('sampleDict','w');
frRead = fgetl(frID);
fprintf(fwID,'%s\n',frRead);
while ~feof(frID)
    frRead = fgetl(frID);
    if strcmp(frRead,'        // Top surface')
        fprintf(fwID,'%s\n',frRead);
        for in=1:length(sampleTop)
            fprintf(fwID,'%s\n',['            (' num2str(sampleTop(in,1)) ' ' num2str(sampleTop(in,2)) ' ' num2str(0) ')']);
        end
        continue;
    end
    if strcmp(frRead,'        // Bottom surface')
        fprintf(fwID,'%s\n',frRead);
        for in=1:length(sampleBottom)
            fprintf(fwID,'%s\n',['            (' num2str(sampleBottom(in,1)) ' ' num2str(sampleBottom(in,2)) ' ' num2str(0) ')']);
        end
        continue;
    end
    fprintf(fwID,'%s\n',frRead);
end
flag = true;
% end
