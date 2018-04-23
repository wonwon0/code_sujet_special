close all
clear
[X Y] = meshgrid(linspace(-5,5,20),linspace(-5,5,20));
Z = 5*ones(length(X),length(Y));
figure
s=surf([X Y Z -Y -Z -X],[Y Z X -Z -X -Y],[Z X Y -X -Y -Z]);
figure
p=patch(surf2patch(s));
shading faceted;
view(3)
axis([-10,10,-10,10,-10,10]);