%fichier test pour premiere animation basique.
close all
clear
L=[0.2755 0.290 0.2803 0.0098];
%définition d'un trajectoire semi-circulaire sur le plan x-y et sinusoidale
%lorsque projetée sur l'axe z.
x=cos(pi/2:-pi/100:-pi)/2.2;y=sin(pi/2:-pi/100:-pi)/2.2;z=sin((-pi/2:pi/100:pi)/4)/4+0.3;
pose_init=[x(1) y(1) z(1)];
theta_init=cin_inv(pose_init,[0 -pi/2 -pi/2]);
theta_act=theta_init;
err=1;
vit=1;
j=1;
dt=0.0001;
for i=2:length(x)
    while err>0.004
    pose_act=cin_dir(theta_act);
    pose_next=[x(i) y(i) z(i)];
    dir=(pose_next-pose_act)/norm(pose_next-pose_act);
    delta_dep=dir*vit*dt;
    pose_next_act=pose_act+delta_dep;
    theta_next=cin_inv(pose_next_act,theta_act);
    theta1_next(j)=theta_next(1);
    theta2_next(j)=theta_next(2);
    theta3_next(j)=theta_next(3);
    posex(j)=pose_next_act(1);
    posey(j)=pose_next_act(2);
    posez(j)=pose_next_act(3);
    pose_next_act=cin_dir(theta_next);
    err=sum(abs(pose_next-pose_next_act));
    theta_act=[theta1_next(j) theta2_next(j) theta3_next(j)];
    j=j+1;
    err_stock(j)=err;
    end
    err=1;
end
theta1_next=theta1_next'+pi/2;
theta2_next=theta2_next'+pi/2;
theta3_next=theta3_next';
%%

P1=[zeros(1,length(posex)) ; zeros(1,length(posex)) ; zeros(1,length(posex))+L(1)]';
P2=[L(2)*cos(theta2_next).*cos(theta1_next) L(2)*cos(theta2_next).*sin(theta1_next) L(2)*sin(theta2_next)]+P1;
P3=[posex' posey' posez'];
figure(1);
for i=1:length(posex)
    
    if(mod(i,10)==0)
    if exist('h2','var')
        delete(h2)
    end
    if exist('h3','var')
        delete(h3)
    end
    if exist('h4','var')
        delete(h4)
    end
    h1=plot3(P3(i,1),P3(i,2),P3(i,3),'X');
    hold on
    h2=line([0 P1(i,1)],[0 P1(i,2)],[0 P1(i,3)]);
    h3=line([P1(i,1) P2(i,1)],[P1(i,2) P2(i,2)],[P1(i,3) P2(i,3)]);
    h4=line([P2(i,1) P3(i,1)],[P2(i,2) P3(i,2)],[P2(i,3) P3(i,3)]);
    drawnow
    end
    
end
%%
subplot(1,3,1)
plot(theta1_next)
subplot(1,3,2)
plot(theta2_next)
subplot(1,3,3)
plot(theta3_next)