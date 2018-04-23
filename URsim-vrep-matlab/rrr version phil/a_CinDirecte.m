function [T,T1,T2,T3] = a_CinDirecte(theta) 

% theta=[0;90;0]*pi/180;

    [DHa,DHb,DHal,DHth] = a_paramDH();% Get DH parameters
    
    thetadh(1)=theta(1) + DHth(1); % Transfer into the DH theta frame 
    thetadh(2)=theta(2) + DHth(2);
    thetadh(3)=theta(3) + DHth(3);
    
    % Homog. Transform 1
    T1=[cos(thetadh(1)) -cos(DHal(1))*sin(thetadh(1)) sin(DHal(1))*sin(thetadh(1)) DHa(1)*cos(thetadh(1));
        sin(thetadh(1)) cos(DHal(1))*cos(thetadh(1)) -sin(DHal(1))*cos(thetadh(1)) DHa(1)*sin(thetadh(1));
        0 sin(DHal(1)) cos(DHal(1)) DHb(1);
        0 0 0 1];
    
    % Homog. Transform 2
    T2=[cos(thetadh(2)) -cos(DHal(2))*sin(thetadh(2)) sin(DHal(2))*sin(thetadh(2)) DHa(2)*cos(thetadh(2));
        sin(thetadh(2)) cos(DHal(2))*cos(thetadh(2)) -sin(DHal(2))*cos(thetadh(2)) DHa(2)*sin(thetadh(2));
        0 sin(DHal(2)) cos(DHal(2)) DHb(2);
        0 0 0 1];

    % Homog. Transform 3
    T3=[cos(thetadh(3)) -cos(DHal(3))*sin(thetadh(3)) sin(DHal(3))*sin(thetadh(3)) DHa(3)*cos(thetadh(3));
        sin(thetadh(3)) cos(DHal(3))*cos(thetadh(3)) -sin(DHal(3))*cos(thetadh(3)) DHa(3)*sin(thetadh(3));
        0 sin(DHal(3)) cos(DHal(3)) DHb(3);
        0 0 0 1];

    % Homog. Transform Total
    T=T1*T2*T3;