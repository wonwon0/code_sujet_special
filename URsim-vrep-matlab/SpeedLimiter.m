function output_velocity = SpeedLimiter(input_velocity,VMax)

% ===================== JOINT VELOCITY STATURATION =====================
    JointVelocities=min(VMax,1)*[1 1 1 1 1 1]';
    output_velocity=input_velocity;
    Div=max((abs(output_velocity./(JointVelocities))));
    if Div > 1
        output_velocity=output_velocity/Div;
    end 
