
function theta   = cin_inv_realdh( pose_act, theta_act, L)
%cin_inv Summary of this function goes here
%   donne les coordonnées articulaires du robot en fonction de la postion
%   de l'effecteur demandé.
theta_dh=[-pi/2;pi/2;0];
% theta_dh=[0;0;0];
x_d=pose_act(1);
y_d=pose_act(2);
z_d=pose_act(3);

%----------------------CALCUL THETA1-------------------
%verfi simgulartie
if (sqrt((x_d^2+y_d^2))<(L(4)*1.1))
    theta_11=theta_act(1);
    theta_12=theta_act(1);
else
    theta_11 = 2*atan2(-x_d+sqrt(x_d^2+y_d^2-L(4)^2),y_d+L(4));
    theta_12 = -2*atan2(x_d+sqrt(x_d^2+y_d^2-L(4)^2),y_d+L(4));
    
end

%----------------------CALCUL THETA3-------------------
%verif singularite avec theta_11
t1=x_d*cos(theta_11)+y_d*sin(theta_11);
t2=z_d-L(1);
t3=(t1^2+t2^2-L(2)^2-L(3)^2)/(2*L(2)*L(3));
if t3^2>0.9999
    theta_31=theta_act(3);
    theta_32=theta_act(3);
else
    theta_31=atan2(sqrt(1-t3^2),t3);
    theta_32=atan2(-sqrt(1-t3^2),t3);
end
%verif singularite avec theta_12
t1=x_d*cos(theta_12)+y_d*sin(theta_12);
t2=z_d-L(1);
t3=(t1^2+t2^2-L(2)^2-L(3)^2)/(2*L(2)*L(3));
if t3^2>0.9999
    theta_33=theta_act(3);
    theta_34=theta_act(3);
else
    theta_33=atan2(sqrt(1-t3^2),t3);
    theta_34=atan2(-sqrt(1-t3^2),t3);
end
%----------------------CALCUL THETA2-------------------
%verif singularite avec theta_11 et theta_31
t1=x_d*cos(theta_11)+y_d*sin(theta_11);
t2=z_d+L(1);
t3=-sin(theta_31)*L(3);
t4=L(2)+L(3)*cos(theta_31);
%calcul theta_21 avec theta_11 et theta_31
if sqrt((L(2)+L(3)*cos(theta_31))^2+(L(3)*sin(theta_31))^2)<0.00001
    theta_21=theta_act(2);
else
    theta_21=atan2(t2,t1)-atan2(t3,t4);
end
%verif singularite avec theta_11 et theta_32
t1=x_d*cos(theta_11)+y_d*sin(theta_11);
t2=z_d+L(1);
t3=-sin(theta_32)*L(3);
t4=L(2)+L(3)*cos(theta_32);
%calcul theta_22 avec theta_11 et theta_32
if sqrt((L(2)+L(3)*cos(theta_32))^2+(L(3)*sin(theta_32))^2)<0.00001
    theta_22=theta_act(2);
else
    theta_22=atan2(t2,t1)-atan2(t3,t4);
end
%verif singularite avec theta_12 et theta_33
t1=x_d*cos(theta_12)+y_d*sin(theta_12);
t2=z_d+L(1);
t3=-sin(theta_33)*L(3);
t4=L(2)+L(3)*cos(theta_33);
%calcul theta_23 avec theta_12 et theta_33
if sqrt((L(2)+L(3)*cos(theta_33))^2+(L(3)*sin(theta_33))^2)<0.00001
    theta_23=theta_act(3);
else
    theta_23=atan2(t2,t1)-atan2(t3,t4);
end
%verif singularite avec theta_12 et theta_34
t1=x_d*cos(theta_12)+y_d*sin(theta_12);
t2=z_d+L(1);
t3=-sin(theta_34)*L(3);
t4=L(2)+L(3)*cos(theta_34);
%calcul theta_24 avec theta_12 et theta_34
if sqrt((L(2)+L(3)*cos(theta_34))^2+(L(3)*sin(theta_34))^2)<0.00001
    theta_24=theta_act(2);
else
    theta_24=atan2(t2,t1)-atan2(t3,t4);
end
%on enlève les angles du a la config du robot quand on a calculer les param
%dh
theta1=[theta_11 theta_11 theta_12 theta_12]'-theta_dh(1);
theta2=[theta_21 theta_22 theta_23 theta_24]'-theta_dh(2);
theta3=[theta_31 theta_32 theta_33 theta_34]'-theta_dh(3);
theta_temp=[theta1 theta2 theta3];
a=0;

%on met les angles entre -pi et pi
for i=1:4
    if (theta_temp(i,1) > pi)||(theta_temp(i,1)<-pi)
        theta_temp(i,1)=theta_temp(i,1)+2*pi;
    elseif (theta_temp(i,2) > pi)||(theta_temp(i,2)<-pi)
        theta_temp(i,2)=theta_temp(i,2)+2*pi;
    end
end
%sélection des solution avec le coude vers le haut
iter=1;
theta_coude_up=[theta_act;theta_act];
for i=1:4
    if theta_temp(i,3)<0 && theta_temp(i,3)>-pi
        for j=1:3
            theta_coude_up(iter,j)=theta_temp(i,j);
        end
        iter=iter+1;
    end 
end

Solution1=[theta_coude_up(1,1) theta_coude_up(1,2) theta_coude_up(1,3)];
Solution2=[theta_coude_up(2,1) theta_coude_up(2,2) theta_coude_up(2,3)];
% Find the error for joint 1 between different solutions
        Theta1_Err1 = Solution1(1) - theta_act(1);
        Theta1_Err2 = Solution2(1) - theta_act(1);

        % Find the solution nearest to the actual position and refactor between 0 and 360
        if (Theta1_Err1 > 0)
            Theta1_Err1_360 = Theta1_Err1 - floor(abs(Theta1_Err1)/(2*pi))*(2*pi);
        else
            Theta1_Err1_360 = Theta1_Err1 + floor(abs(Theta1_Err1)/(2*pi))*(2*pi);
        end
        if (Theta1_Err1_360 < -pi)
            Theta1_Err1_360 = Theta1_Err1_360 + (2*pi);
        elseif (Theta1_Err1_360 > pi)
            Theta1_Err1_360 = Theta1_Err1_360 - (2*pi);
        end

        if (Theta1_Err2 > 0)
            Theta1_Err2_360 = Theta1_Err2 - floor(abs(Theta1_Err2)/(2*pi))*(2*pi);
        else
            Theta1_Err2_360 = Theta1_Err2 + floor(abs(Theta1_Err2)/(2*pi))*(2*pi);
        end
        if (Theta1_Err2_360 < -pi)
            Theta1_Err2_360 = Theta1_Err2_360 + (2*pi);
        elseif (Theta1_Err2_360 > pi)
            Theta1_Err2_360 = Theta1_Err2_360 - (2*pi);
        end
        
        % Find the closest solution to the actual position
        if (abs(Theta1_Err1_360) < abs(Theta1_Err2_360))
            SolutionF = Solution1;
        else
            SolutionF = Solution2;
        end
        theta=SolutionF;
        for i=1:3
            theta(i)=mod(theta(i),2*pi);
        end
        