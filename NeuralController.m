%
% Simple neural controller
%
% Author: Jamie Brown
%
% Created: 18/02/19
%
% Changes
%               
%
%
%

function [LeftVoltage, RightVoltage] = NeuralController(LeftS, RightS)

    %----------------------------------------------%
    %Set up weights and thresholds
    T1 = 0.015;
    T2 = 0.015;
    w1 = -1.4;  %left input to left motor
    w2 = 1.2;   %left input to right motor
    w3 = 1.25;  %right input to left motor
    w4 = -1;    %right input to right motor
    %----------------------------------------------%

    %----------------------------------------------%
    if (LeftS == 1 && RightS == 1)
        LeftVoltage = 0.75;
        RightVoltage = 0.75;
    else

        LeftNeuVal = (LeftS*w1) + (RightS*w3);  %calculate left neuron value before biasing
        if (LeftNeuVal > T1)                    %apply threshold bias
            LeftVoltage = 0.75;
        else
            LeftVoltage = -0.5;
        end

        RightNeuVal = (LeftS*w2) + (RightS*w4);  % calculate right neuron value before biasing
        if (RightNeuVal > T2)
            RightVoltage = 0.75;
        else
            RightVoltage = -0.5;
        end
    end
    %----------------------------------------------%
