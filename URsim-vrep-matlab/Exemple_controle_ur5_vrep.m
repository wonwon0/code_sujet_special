    
clear all
% Ceci est un exemple permettant de contrôler en translation le robot UR5
% dans URsim et de visualiser le robot dans VRep.
% Cet exemple requiert un Joystick.



% Connect to robot
Robot_IP = '132.203.102.123'; % adresse ip de la machine virtuelle ou du vrai robot
Socket_conn = tcpip(Robot_IP,30000,'NetworkRole','server'); % création du socket de communication TCP
fclose(Socket_conn); % fermeture des communications déjà exitante s'il y a lieu.
disp('Press Play on Robot...')
fopen(Socket_conn); % connection au robot d'URsim après avoir appuyé sur play dans URsim
disp('Connected!');
dh=dh_UR5(); % paramètre dh du robot
disp('connection VREP...');
vrep=remApi('remoteApi'); % initialisation de la "classe" point d'entrée de l'API de VRep
vrep.simxFinish(-1); % fermeture des communications déjà exitante s'il y a lieu.
id = vrep.simxStart('127.0.0.1', 19997, true, true, 2000, 5); % connection à VRep. "id" est le socket de communication pour VRep
if (id>-1)
    disp('Connected to VREP remote API server');
end
h = struct('id', id);
armJoints = [-1,-1,-1,-1,-1,-1];
theta_dot=[0;0;0;0;0;0];
for i = 1:6
    %acquisition de la nomenclature des joints du robot dans VRep
  [res armJoints(i)] = vrep.simxGetObjectHandle(id, sprintf('UR5_joint%d',i), vrep.simx_opmode_oneshot_wait); vrchk(vrep, res);
end
h.armJoints=armJoints;
Robot_Pose_j = readrobotpose_j(Socket_conn); % lecture de la position angulaire des joint dans URsim.
for i = 1:6
    %assignation des position anguilaire du robot dans VRep pour qu'il
    %aille la même configuration que le robot d'URsim
    res = vrep.simxSetJointPosition(id, h.armJoints(i),...
                                         Robot_Pose_j(i),...
                                 vrep.simx_opmode_oneshot);
end
while 1
    tic;
    Robot_Pose_j = readrobotpose_j(Socket_conn); % lecture de la position angulaire des joint dans URsim.
    Robot_Pose_j=Robot_Pose_j';
    Robot_Pose=cin_dir_6ddl(Robot_Pose_j,dh);
    Robot_Pose_vrep=Robot_Pose_j-dh.theta;
    for i = 1:6
    %assignation des position anguilaire du robot dans VRep pour qu'il
    %aille la même configuration que le robot d'URsim
    res = vrep.simxSetJointPosition(id, h.armJoints(i),...
                                         Robot_Pose_vrep(i),...
                                    vrep.simx_opmode_oneshot);
    end
    %communication avec le joystick
    [pos, but] = mat_joy(1);
    a = pos;
    if but(2)==1
        ligne=0;
    end
    if but(3)==1
        ligne=1;
    end
    %Saturation de la commande de vitesse (s'assurer que la norme soit
    %moins grande que 1)
    if (a(1)==0 && a(2)==0 && a(3)==0)
        dir=[0 0 0];
    elseif sqrt((double(a(1)))^2+(double(a(2)))^2+(double(a(3)))^2)<1
        dir=[double(a(2)),double(a(1)),-double(a(3))];
    else
        dir=[double(a(2)),double(a(1)),-double(a(3))]/sqrt((double(a(1)))^2+(double(a(2)))^2+(double(a(3)))^2);
    end
    next_pose=Robot_Pose+[0 0 0 dir(1);0 0 0 dir(2);0 0 0 dir(3); 0 0 0 0]*1;
    next_ang=cin_inv_6ddl(next_pose,dh,Robot_Pose_j);
    for i =1:6
        if (next_ang(i) - Robot_Pose_j(i))>pi
            theta_dot(i)=-next_ang(i) - Robot_Pose_j(i)-2*pi;
        elseif (next_ang(i) - Robot_Pose_j(i))<-pi
            theta_dot(i)=next_ang(i) - Robot_Pose_j(i)+2*pi;
        else
            theta_dot(i)=next_ang(i) - Robot_Pose_j(i);
        end
    end
    time=toc;
    
    theta_dot=theta_dot/time;
    speedrobot(Socket_conn,theta_dot'); % envoi de la commande de vitesse articulaire à URsim

end





















