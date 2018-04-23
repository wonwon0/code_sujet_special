    

% Note: 
% Copy the Polyscope folder to UR5 Controller using a USB drive
% Set UR5 IP
% Set PC IP in Polyscope program
% First run Matlab code; then run the Polyscope program
clear
% rosshutdown
% rosinit
addpath('../rrr version phil v2')
addpath('../rrr version phil/geom3d/geom3d')
addpath('../rrr version phil/geom3d/meshes3d')
addpath('../rrr version phil/p_poly_dist_v1')
addpath('../rrr version phil/joystick soft 2')
addpath('../rrr version phil/Remesher')
[robot_joint_subscriber, joint_cmd_publisher, joint_cmd_message, dh] = ur5_ros_controller_init();
contact_subscriber = rossubscriber('/forces');
contacts = receive(contact_subscriber,10);
dh_eff = dh;
% for i = 1:100
%     pose_data = receive(robot_joint_subscriber,10);
%     pose_data.Actual.Positions
% end
% rosshutdown
% % Mouse3D('start');

my_joystick = vrjoystick(1);
last_cond = 0;

% load limit bodies
limit=StructureLimites_v3();

% trouver l'état actuel du robot
pose_data = receive(robot_joint_subscriber,10);
Robot_Pose_j = pose_data.Position(2:7)';
Robot_Pose_j_history = [0 Robot_Pose_j 0 0 0];
pose_init = cin_dir_6ddl(Robot_Pose_j,dh_eff);
pose_init = [pose_init(1,4), pose_init(2,4), pose_init(3,4)];
% on examine l'état des collisions au départ de l'algo

jacob_eff = jacob_UR5(Robot_Pose_j ,pose_init, dh(1));
[d_min, pose_prox, pose_prox_pt_act] = verifDistance_v5(limit, pose_init, jacob_eff, jacob_eff, Robot_Pose_j);
%afficherLimites(limit);
theta_dot_threshold = 0.0001;
while 1
    tic;
    contacts = receive(contact_subscriber,10);
    pose_data = receive(robot_joint_subscriber,10);
    Robot_Pose_j = pose_data.Position(2:7)';
    Robot_Poses = cin_dir_6ddl(wrapToPi(Robot_Pose_j), dh);
    Robot_Poses = Robot_Poses(1:3,4)';
    
    [ dir, rot ] = read_joystick_inputs( my_joystick );
    
    % On cherche si un objet entre en collision avec le robot
    pose_prox=[];d_min=[];
    jacob_eff = jacob_UR5(Robot_Pose_j ,Robot_Poses(1, :), dh(1));
    t=1;
    for i=1:size(Robot_Poses,1)
        jacob_local = jacob_UR5(Robot_Pose_j ,Robot_Poses(1, :), dh(i));
        %[d_min_t,pose_prox_t,pose_prox_pt_act]=verifDistance_v5(limit,Robot_Poses(i, :), jacob_local,jacob_eff,Robot_Pose_j);
        for j=1:length(contacts.Contacts)
            if strcmp(contacts.Contacts(j).Collision2, 'ur_on_table::ur_6_hand_limb::ur_6_hand_limb_collision')
                
                contacts.Contacts(j).Normal
                pose_prox(t,:) = Robot_Poses(1, 1:3) + contacts.Contacts(j).Normal';
                pose_prox_pt_act = pose_prox;
                d_min_t(t) = 0.01;
            else
                d_min_t(t) = 10;
                pose_prox(t,:) = Robot_Poses(1, 1:3);
                pose_prox_pt_act = pose_prox;
            end
            t = t+1;
        end
        d_min=[d_min d_min_t];
        pose_prox=[pose_prox; pose_prox_pt_act];
        poses_prox_pt_act.pose(i).poses=pose_prox_pt_act;
    end
    %on garde en m�moire l'input de l'utilisateur
    %on test si l'input de l'utilisateur n'entre pas en conflit avec une
    %limite
    normale_effecteur=[];
    d_min_relevant=[];
    d_min
    pose_prox
    for i = 1:2:length(dh)
        jacob_pt=jacob_UR5(Robot_Pose_j ,Robot_Poses(i, :), dh(i));
        m=size(limit.limite,2);
        min_gap = 5;
        for j=1:m
            if d_min((i-1)*m+j)<min_gap && ((all(limit.limite(j).type=='tube'))||(all(limit.limite(j).type=='sphe')))
                if (all(limit.limite(j).type=='tube')) && ddl(i)==2
                    ;
                else
                    normale_effecteur_loc=PointToEffector(pose_prox((i-1)*m+j,:), Robot_Poses(i,:), jacob_pt, jacob_eff );
                    normale_effecteur=[normale_effecteur;normale_effecteur_loc];
                    d_min_relevant=[d_min_relevant;d_min((i-1)*m+j)];
                end
            elseif d_min((i-1)*m+j)<min_gap
                normale_effecteur_loc=PointToEffector(pose_prox((i-1)*m+j,:), Robot_Poses(i,:), jacob_pt, jacob_eff );
                normale_effecteur=[normale_effecteur;normale_effecteur_loc];
                d_min_relevant=[d_min_relevant;d_min((i-1)*m+j)];
            end
        end
    end
    %%%%%%%%%%
    normale_effecteur
    v_prem = dir;
    normale_effecteur = check_redund_v2(Robot_Poses(1,:), normale_effecteur,d_min_relevant,pose_prox);
    normale_effecteur
    v_input=verifVitesse_v7(v_prem,normale_effecteur,d_min_relevant, min_gap * 1);
    normale_effecteur;
%     if slide==0
%         if ~all(v_prem==v_input)
%             v_input=[0 0 0];
%         end
%     end
    [ next_ang, theta_dot, next_pose ] = move_ur5_robot( v_input, rot, last_cond, robot_joint_subscriber, dh );
    
    theta_dot = SpeedLimiter(theta_dot, 1);
    
    [ Robot_Pose_j_history ] = ros_ur_controller_manager( theta_dot, next_ang, theta_dot_threshold, pose_data, joint_cmd_publisher, joint_cmd_message, Robot_Pose_j_history);
end





















