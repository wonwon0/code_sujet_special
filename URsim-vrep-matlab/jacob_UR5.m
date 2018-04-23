function [ jac ] = jacob_UR5( theta_act, Pose, dh, membrure)
    %theta_act: position angulaire des joint du robot
    %pose: position et orientation de l'effecteur
    %dh: structure contenant les param. dh du robot
    if nargin < 4
        membrure = 6;
    end
    theta=theta_act;
    x=Pose(1);y=Pose(2);z=Pose(3);
    jac=[ 0 sin(theta(1)) sin(theta(1)) sin(theta(1)) cos(theta(1))*sin(theta(2)+theta(3)+theta(4)) -cos(theta(1))*cos(theta(2)+theta(3)+theta(4))*sin(theta(5))+sin(theta(1))*cos(theta(5));
        0 -cos(theta(1)) -cos(theta(1)) -cos(theta(1)) sin(theta(1))*sin(theta(2)+theta(3)+theta(4)) -sin(theta(1))*cos(theta(2)+theta(3)+theta(4))*sin(theta(5))-cos(theta(1))*cos(theta(5));
        1 0 0 0 -cos(theta(2)+theta(3)+theta(4)) -sin(theta(2)+theta(3)+theta(4))*sin(theta(5));
        -y -cos(theta(1))*(z-dh.b(1)) -cos(theta(1))*(z-dh.b(1)-dh.a(2)*sin(theta(2))) cos(theta(1))*(dh.b(5)*cos(theta(2)+theta(3)+theta(4))+dh.b(6)*sin(theta(2)+theta(3)+theta(4))*sin(theta(5))) -dh.b(6)*(sin(theta(1))*sin(theta(5))+cos(theta(1))*cos(theta(2)+theta(3)+theta(4))*cos(theta(5))) 0;
        x -sin(theta(1))*(z-dh.b(1)) -sin(theta(1))*(z-dh.b(1)-dh.a(2)*sin(theta(2))) sin(theta(1))*(dh.b(5)*cos(theta(2)+theta(3)+theta(4))+dh.b(6)*sin(theta(2)+theta(3)+theta(4))*sin(theta(5))) dh.b(6)*(cos(theta(1))*sin(theta(5))-sin(theta(1))*cos(theta(2)+theta(3)+theta(4))*cos(theta(5))) 0;
        0 cos(theta(1))*x+sin(theta(1))*y dh.a(3)*cos(theta(2)+theta(3))+dh.b(5)*sin(theta(2)+theta(3)+theta(4))-dh.b(6)*cos(theta(2)+theta(3)+theta(4))*sin(theta(5)) dh.b(5)*sin(theta(2)+theta(3)+theta(4))-dh.b(6)*cos(theta(2)+theta(3)+theta(4))*sin(theta(5)) -dh.b(6)*sin(theta(2)+theta(3)+theta(4))*cos(theta(5)) 0];
    if membrure ~= 6
        jac(:,membrure + 1:end) = 0;
    end
end

