function [SentAngle,RequestAngle,e_0,DoneJoints] = SpeedLimiterNController(Encoders,Target,PIDKs,Ts,VMax,min_max_doigt)
%#codegen
persistent mSentAngle Integral e_1;
if isempty(mSentAngle)
    mSentAngle = Encoders;
    Integral=[0;0;0;0;0;0];
    e_1=[0;0;0;0;0;0];
end
DoneJoints=0;
%Offset fingers to match value found at initialisation
Target(7) = Target(7)+min_max_doigt(1);
Target(8) = Target(8)+min_max_doigt(3);
Target(9) = Target(9)+min_max_doigt(5);

% ===================== JOINT VELOCITY STATURATION =====================
    JointVelocities=min(VMax,40)*[1 1 1 2 2 2]';
    dJ=Target(1:6)-mSentAngle(1:6);
    Div=max((abs(dJ./(JointVelocities*Ts))));
    if Div > 1
        dJ=dJ/Div;
        DoneJoints=0;
    else
        DoneJoints=1;
    end   
    % FINGERS DONE SEPARATELY (This way Broken fingers dont interfere)
        JointVelocitiesF=VMax*[2 2 2]';
        dJF=Target(7:9)-mSentAngle(7:9);
        DivF=max((abs(dJF./(JointVelocitiesF*Ts))));
        if DivF > 1
            dJF=dJF/DivF;
        end
    mSentAngle=mSentAngle+[dJ;dJF];
% ========================= Joint Compensation =========================
    ON=PIDKs(25);Kp=PIDKs(1:6);Ki=PIDKs(7:12);Kd=PIDKs(13:18);uMax=PIDKs(19:24);
    e_0=mSentAngle(1:6)-Encoders(1:6);
	MaxRes=[0.1 0.1  0.1 0.15 0.15 0.15]';
	e_0=(sign(e_0).*(abs(e_0)-MaxRes)).*((abs(e_0))>MaxRes); %Encoder Resolution Clipping
    if ON
        Integral   = Integral+ e_0*Ts;  
        Derivative = (e_0-e_1)/Ts;
        u_c=Kp.*e_0 + Ki.*Integral +Kd.*Derivative;    
    else
    	Integral=[0;0;0;0;0;0];
        u_c=[0;0;0;0;0;0];    
    end
    e_1=e_0;
    % MAX ALLOWED COMPENSATION
        Veriff=abs(u_c)  > uMax;
        u_c=sign(u_c).*uMax.*Veriff+~Veriff.*u_c;
 %REARRENGE
    RequestAngle=mSentAngle;
    SentAngle=[mSentAngle(1:6)+u_c;mSentAngle(7:9)];
end% Init Variables ---------------------
