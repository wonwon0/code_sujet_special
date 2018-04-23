global state;
if state==0
    close all
    
    %addpath('U:\matlab\rrr version phil\p_poly_dist_v1')
    %addpath('U:\matlab\rrr version phil\joystick soft 2')
    addpath('U:\matlab\rrr version phil\Remesher')
    filename = 'test_col2.gif';
elseif state==1
        %%

    %définition d'un trajectoire semi-circulaire sur le plan x-y et sinusoidale
    %lorsque projetée sur l'axe z.
    pose_init=[0.15,0.15,0.4];

    %Définition des paramètres DH du robot
    L=[0.2755 0.290 0.390 0.0098];

    %Position initiale du robo
    theta_init=cin_inv(pose_init,[0 -pi/2 -pi/2],L);
    theta_act=theta_init;

    %vitesse maxiamle du robot
    vit=0.1;
    j=1;

    %fréquence de l'asservissement
    dt=0.01;

    %nombre de points conservés dans le graphique
    buf_size = 1;
    buf = zeros(buf_size,3)+0.3;

    %calcul du positionnement de chacun des liens du robot

    P1=[0 0 L(1)];
    P2=[L(2)*cos(theta_init(2)).*cos(theta_init(1)) L(2)*cos(theta_init(2)).*sin(theta_init(1)) L(2)*sin(theta_init(2))]+P1;
    P3=cin_dir(theta_init, [0.2755 0.290 0 0.0098]);
    P4=cin_dir(theta_init, [0.2755 0.290 0.2 0.0098]);
    P5=cin_dir(theta_init, L);
%     %%
%     %import et initialisation des solides
% 
%     Q1=[cos(theta_init(1)) -sin(theta_init(1))*cos(0) sin(0)*sin(theta_init(1)); 
%         sin(theta_init(1)) cos(0)*cos(theta_init(1)) -sin(0)*cos(theta_init(1)); 
%         0 sin(0) cos(0)];
%     Q2=[1 0 0; 
%         0 cos(theta_init(2)+pi/2) -sin(theta_init(2)+pi/2); 
%         0 sin(theta_init(2)+pi/2) cos(theta_init(2)+pi/2)];
%     Q3=[1 0 0; 
%         0 cos(theta_init(3)) -sin(theta_init(3)); 
%         0 sin(theta_init(3)) cos(theta_init(3))];
% 
%     %import de la base
%     [v1, f1, n1, c1, stltitle] = stlread('M0_bin.STL', 0);
%     [v1, f1]=patchslim(v1, f1);
%     v1=[v1(:,1)-0.052 v1(:,2) v1(:,3)-0.05];
%     v1=[1 0 0; 0 cos(pi/2) -sin(pi/2); 0 sin(pi/2) cos(pi/2)]*[cos(pi) -sin(pi) 0; sin(pi) cos(pi) 0; 0 0 1]*v1';
%     v1=v1';
%     v1=[v1(:,1) v1(:,2) v1(:,3)+0.15656];
% 
% 
%     %import du joint 1
%     [v2, f2, n, c, stltitle] = stlread('M1_bin.STL', 0);
%     [v2, f2]=patchslim(v2, f2);
%     v2=[v2(:,1)-0.18737 v2(:,2)-0.04276 v2(:,3)-0.04181];
%     v2=[cos(pi/2) 0 sin(pi/2); 0 1 0; -sin(pi/2) 0 -cos(pi/2)]*v2';
%     v2=[cos(pi/2) -sin(pi/2) 0; sin(pi/2) cos(pi/2) 0; 0 0 1]*v2;
%     v2=v2';
% 
%     v2_1=(Q1*v2')';
%     v2_1=[v2_1(:,1) v2_1(:,2) v2_1(:,3)+0.15656];
% 
%     %import et posiitonnement de la membrure 1
%     [v3, f3, n, c, stltitle] = stlread('M2_bin.STL', 0);
%     [v3, f3]=patchslim(v3, f3);
%     v3=[v3(:,1)-0.04120 v3(:,2)-0.04120 v3(:,3)-0.11257];
%     v3=[cos(-pi/2) 0 sin(-pi/2); 0 1 0; -sin(-pi/2) 0 -cos(-pi/2)]*v3';
%     v3=[1 0 0; 0 cos(pi/2) -sin(pi/2); 0 sin(pi/2) cos(pi/2)]*v3;
%     v3=[cos(pi) -sin(pi) 0; sin(pi) cos(pi) 0; 0 0 1]*v3;
%     v3=v3';
% 
%     v3_1=(Q2*v3');
%     v3_1=(Q1*v3_1)';
%     v3_1=[v3_1(:,1) v3_1(:,2) v3_1(:,3)+0.275];
% 
% 
%     %import de la membrure 2--
%     [v4, f4, n, c, stltitle] = stlread('M4_bin.STL', 0);
%     [v4, f4]=patchslim(v4, f4);
%     v4=v4/1000;
%     v4=[v4(:,1)-0.0098 v4(:,2)-0.173 v4(:,3)-0.6471];
%     v4=v4*[1 0 0; 0 cos(40*pi/180) -sin(40*pi/180); 0 sin(40*pi/180) cos(40*pi/180)];
%     v4=[1 0 0; 0 cos(-pi) -sin(-pi); 0 sin(-pi) cos(-pi)]*v4';
%     v4=v4';
%     v4_1=(Q3*v4');
%     v4_1=(Q2*v4_1);
%     v4_1=(Q1*v4_1)';
% 
%     v4_1=[v4_1(:,1)+P3(1) v4_1(:,2)+P3(2) v4_1(:,3)+P3(3)];
% 
% 
%     %import de la main fermée
%     [v5, f5, n, c, stltitle] = stlread('M5_bin.STL', 0);
%     [v5, f5]=patchslim(v5, f5);
%     v5=v5/1000;
%     v5=v5*[1 0 0; 0 cos(40*pi/180) -sin(40*pi/180); 0 sin(40*pi/180) cos(40*pi/180)];
%     v5=[v5(:,1)-0.01 v5(:,2)-0.215 v5(:,3)-0.311];
%     v5=[1 0 0; 0 cos(-pi) -sin(-pi); 0 sin(-pi) cos(-pi)]*v5';
%     v5=v5';
%     v5_1=(Q3*v5');
%     v5_1=(Q2*v5_1);
%     v5_1=(Q1*v5_1)';
%     v5_1=[v5_1(:,1)+P4(1) v5_1(:,2)+P4(2) v5_1(:,3)+P4(3)];
% 
% 
% 
%     %affichage des membrures positionnées.
%     fig=figure(1);
%     set(fig, 'renderer', 'OpenGL');
%     set(fig, 'Position', [-960 40 960 960])
%     p1=patch('Faces',f1,'Vertices',v1,'FaceVertexCData',c);
%     set(p1, 'facec', 'flat');  
%     set(p1, 'facec', [0 0.6 1]);
%     set(p1, 'EdgeColor','None');
% 
%     p2=patch('Faces',f2,'Vertices',v2_1,'FaceVertexCData',c);
%     set(p2, 'facec', 'flat');  
%     set(p2, 'facec', [0 0.6 1]);
%     set(p2, 'EdgeColor','None');
%     hold on
% 
%     p3=patch('Faces',f3,'Vertices',v3_1,'FaceVertexCData',c);
%     set(p3, 'facec', 'flat');  
%     set(p3, 'facec', [0 0.6 1]);
%     set(p3, 'EdgeColor','None');
%     hold on
% 
%     p4=patch('Faces',f4,'Vertices',v4_1,'FaceVertexCData',c);
%     set(p4, 'facec', 'flat');  
%     set(p4, 'facec', [0 0.6 1]);
%     set(p4, 'EdgeColor','None');
%     hold on
% 
%     p5=patch('Faces',f5,'Vertices',v5_1,'FaceVertexCData',c);
%     set(p5,'facec','flat');
%     set(p5, 'facec', [0 0.6 1]);
%     set(p5, 'EdgeColor','None');
%     hold on

    camlight;
    axis vis3d equal;
    view(3);

    xlabel('x')
    ylabel('y')
    zlabel('z')
    axis([-1.25 1.25 -1.25 1.25 -0.5 1.5]);
    c1=-1;
    c2=-1;
    c3=-1;
    %%
    k=0;
    limit=StructureLimites();
    [d_min, pose_prox]=verif_dist_rtlab(limit.limite(1).surfaces.surface1,limit.limite(1).surfaces.surface2,limit.limite(1).centroide,limit.limite(1).rayonProxy,pose_init);
    afficherLimites(limit);
    hold on
    for i=1:length(pose_prox(:,1))
        h8(i)=line([pose_init(1) pose_prox(i,1)],[pose_init(2) pose_prox(i,2)],[pose_init(3) pose_prox(i,3)],'LineWidth',2,'color','r');
    end
else
    canard=0;
    %communication avec le joystick
    theta_act=mod(theta,2*pi);
    
    canard=1;
    a=move;
%     [pos, but] = mat_joy(0);
%     a = pos;
    %PGD du robot pour obtenir la position de l'effecteur après avoir
    %réaliser la commande de la dernière boucle.
    pose_act=cin_dir(theta_act, L);
    canard=2;
    %Saturation de la commande de vitesse (s'assurer que la norme soit
    %moins grande que 1)
    if (a(1)==0 && a(2)==0 && a(3)==0)
        dir=[0 0 0];
    elseif sqrt((double(a(1)))^2+(double(a(2)))^2+(double(a(3)))^2)<1
        dir=[c1*double(a(1)),c2*double(a(2)),c3*double(a(3))];
    else
        dir=[c1*double(a(1)),c2*double(a(2)),c3*double(a(3))]/sqrt((double(a(1)))^2+(double(a(2)))^2+(double(a(3)))^2);
    end
    canard=3;
    %On cherche si un objet entre en collision avec le robot
    [d_min,pose_prox]=verifDistance(limit,pose_act);
    canard=4;
    %on garde en mémoir l'input de l'utilisateur
    v_prem=dir;
    %on test si l'input de l'utilisateur n'entre pas en conflit avec une
    %limite
    v_input=verifVitesseAlex(v_prem, d_min, pose_prox, pose_act);
    canard=5;
    %on peut m'intenant faire bouger le robot sans craindre d'enfrindre une
    %limite
    % calcul du déplacement effectué lors d'une boucle.
%     delta_dep=v_input*vit*dt;
%     pose_next_act=pose_act+delta_dep;
%     theta_next=cin_inv(pose_next_act,theta_act, L);
    canard=6;
    jacob=jacobian_loc(theta_act, L);
    vit_ang=inv(jacob)*v_input'*vit/dt;
    theta_next=theta_act+(vit_ang')*dt;
    
    %sauvegarde du prochain angle. Il est nécessaire pour calculer la
    %position des points requis pour l'affichage du graphique.
    theta_next_save=theta_next;
    theta_next=[theta_next(1)+pi/2 theta_next(2)+pi/2 theta_next(3)];
    
    %Définition des points pour afficher le robot sur le simulateur.
%     P1=[0 0 L(1)];
%     P2=[L(2)*cos(theta_next(2)).*cos(theta_next(1)) L(2)*cos(theta_next(2)).*sin(theta_next(1)) L(2)*sin(theta_next(2))]+P1;
%     P3=cin_dir(theta_next_save, [0.2755 0.290 0 0.0098]);
%     P4=cin_dir(theta_next_save, [0.2755 0.290 0.2 0.0098]);
%     P5=cin_dir(theta_next_save, L);
%     
%     %rotation de la diférence de distance avec la derniere itération
%     
%     Q1=[cos(theta_next_save(1)) -sin(theta_next_save(1))*cos(0) sin(0)*sin(theta_next_save(1)); 
%     sin(theta_next_save(1)) cos(0)*cos(theta_next_save(1)) -sin(0)*cos(theta_next_save(1)); 
%     0 sin(0) cos(0)];
%     Q2=[1 0 0; 
%         0 cos(theta_next_save(2)+pi/2) -sin(theta_next_save(2)+pi/2); 
%         0 sin(theta_next_save(2)+pi/2) cos(theta_next_save(2)+pi/2)];
%     Q3=[1 0 0; 
%         0 cos(theta_next_save(3)) -sin(theta_next_save(3)); 
%         0 sin(theta_next_save(3)) cos(theta_next_save(3))];
% 
%     v2_1=(Q1*v2')';
%     v2_1=[v2_1(:,1) v2_1(:,2) v2_1(:,3)+0.15656];
%     v3_1=(Q2*v3');
%     v3_1=(Q1*v3_1)';
%     v3_1=[v3_1(:,1) v3_1(:,2) v3_1(:,3)+0.275];
%     v4_1=(Q3*v4');
%     v4_1=(Q2*v4_1);
%     v4_1=(Q1*v4_1)';
%     v4_1=[v4_1(:,1)+P3(1) v4_1(:,2)+P3(2) v4_1(:,3)+P3(3)];
%     v5_1=(Q3*v5');
%     v5_1=(Q2*v5_1);
%     v5_1=(Q1*v5_1)';
%     v5_1=[v5_1(:,1)+P4(1) v5_1(:,2)+P4(2) v5_1(:,3)+P4(3)];
%     
%     
%     
    %Delete les dernières lignes sur le tableau avant de réafficher les
    %nouvelles.
    
%     if exist('h1','var')
%         delete(h1)
%     end
%     if exist('h2','var')
%         delete(h2)
%     end
%     if exist('h3','var')
%         delete(h3)
%     end
%     if exist('h4','var')
%         delete(h4)
%     end
%     if exist('h5','var')
%         delete(h5)
%     end
%     if exist('h6','var')
%         delete(h6)
%     end
%     if exist('h7','var')
%         delete(h7)
%     end
%     if exist('h8','var')
%         delete(h8)
%     end
%     
%     if exist('p2','var')
%         delete(p2)
%     end
%     if exist('p3','var')
%         delete(p3)
%     end
%     if exist('p4','var')
%         delete(p4)
%     end
%     if exist('p5','var')
%         delete(p5)
%     end
%     
%     %affichage des membrures positionnées.
% 
%     p2=patch('Faces',f2,'Vertices',v2_1,'FaceVertexCData',c);
%     set(p2, 'facec', [0 0.6 1]);
%     set(p2, 'EdgeColor','None');
%     hold on
% 
%     p3=patch('Faces',f3,'Vertices',v3_1,'FaceVertexCData',c);
%     set(p3, 'facec', [0 0.6 1]);
%     set(p3, 'EdgeColor','None');
%     hold on
% 
%     p4=patch('Faces',f4,'Vertices',v4_1,'FaceVertexCData',c);
%     set(p4, 'facec', [0 0.6 1]);
%     set(p4, 'EdgeColor','None');
%     hold on
%     p5=patch('Faces',f5,'Vertices',v5_1,'FaceVertexCData',c);
%     set(p5,'facec','flat');
%     set(p5, 'facec', [0 0.6 1]);
%     set(p5, 'EdgeColor','None');
%     hold on
% 
%     %Affichage du graphique
%     buf = [P5(1),P5(2),P5(3);buf(1:end-1,:)];
%     h1=plot3(buf(:,1),buf(:,2),buf(:,3),'X','Color','b');
%     hold on
%     h2=line([0 P1(1)],[0 P1(2)],[0 P1(3)]);
%     h3=line([P1(1) P2(1)],[P1(2) P2(2)],[P1(3) P2(3)]);
%     h4=line([P2(1) P3(1)],[P2(2) P3(2)],[P2(3) P3(3)]);
%     h5=line([P3(1) P5(1)],[P3(2) P5(2)],[P3(3) P5(3)]);
%     hold on
%     [h6(1), h7(1)]=vectarrow(P5,P5+dir/5,'r');
%     hold on
%     [h6(2), h7(2)]=vectarrow(P5,P5+v_input/5,'b');
%     hold on
    
%     for i=1:length(pose_prox(:,1))
%     h8(i)=line([pose_act(1) pose_prox(i,1)],[pose_act(2) pose_prox(i,2)],[pose_act(3) pose_prox(i,3)],'LineWidth',2,'color','r');
%     end
%     drawnow

    theta1_next=theta_next_save(1)*180/pi;
    theta2_next=theta_next_save(2)*180/pi;
    theta3_next=theta_next_save(3)*180/pi;
    theta_next_save=mod(theta_next_save,2*pi);
    theta_act=mod(theta_act,2*pi);
    diff=theta_next_save-theta_act;
    delta_theta = signedDelta_theta(theta_next_save,theta_act)*180/pi;
    theta1_delt=vit_ang(1)*180/pi;
    theta2_delt=vit_ang(2)*180/pi;
    theta3_delt=vit_ang(3)*180/pi;
%     theta1_delt=delta_theta(1)*180/pi;
%     theta2_delt=delta_theta(2)*180/pi;
%     theta3_delt=delta_theta(3)*180/pi;
    canard=7;
    
%     %enregistrement du .gif
%     k=k+1;
%     frame = getframe(1);
%     im = frame2im(frame);
%     [imind,cm] = rgb2ind(im,256);
%     if k == 1;
%         imwrite(imind,cm,filename,'gif', 'Loopcount',inf);
%     else
%         imwrite(imind,cm,filename,'gif','WriteMode','append');
%     end
end

















