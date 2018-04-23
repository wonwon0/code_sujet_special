figure
close all
clear
%import de la main fermée
[v5, f5, n, c, stltitle] = stlread('M5_bin.STL', 0);
[v5, f5]=patchslim(v5, f5);
v5=v5/1000;
v5=v5*[1 0 0; 0 cos(40*pi/180) -sin(40*pi/180); 0 sin(40*pi/180) cos(40*pi/180)];
v5=[v5(:,1)-0.01 v5(:,2)-0.215 v5(:,3)-0.311];

h2=patch('Faces',f5,'Vertices',v5,'FaceVertexCData',c);
set(h2, 'facec', [0 0.6 1]);
set(h2, 'EdgeColor','b');


line([0 0],[0.05 -0.2],[0 0],'color','r','LineWidth',2);
line([-0.05 0.05],[0 0],[0 0],'color','r','LineWidth',2);
xlabel('x')
ylabel('y')
zlabel('z')
view(180,0);