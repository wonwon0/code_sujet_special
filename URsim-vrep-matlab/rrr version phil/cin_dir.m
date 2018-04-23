function [ pose_act ] = cin_dir( theta_act, L )
%cin_dir résoud le problème géométrique directe du robot RRR
theta1=theta_act(1)+pi/2;theta2=theta_act(2)+pi/2;theta3=theta_act(3);

x=L(2)*cos(theta1)*cos(theta2)+L(3)*cos(theta1)*cos(theta2)*cos(theta3)-...
    L(3)*cos(theta1)*sin(theta2)*sin(theta3)-L(4)*sin(theta1);
y=L(2)*sin(theta1)*cos(theta2)+L(3)*sin(theta1)*cos(theta2)*cos(theta3)-...
    L(3)*sin(theta1)*sin(theta2)*sin(theta3)+L(4)*cos(theta1);
z=L(1)+L(2)*sin(theta2)+L(3)*sin(theta2)*cos(theta3)+L(3)*cos(theta2)*sin(theta3);
pose_act=[x y z];

end

