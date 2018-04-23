function [DHa,DHb,DHal,DHth] = a_paramDH()

L1 = 0.2755;
L2 = 0.29;
L3 = 0.2803;
e = 0.0098;

    DHa = [0;L2;L3];
    DHb = [L1;0;-e];
    DHal = [pi/2;0;0];
    DHth = [pi/2;pi/2;0];

end