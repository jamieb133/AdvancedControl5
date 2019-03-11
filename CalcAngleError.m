%
% Converts heading to counterclockwise bearing
%
% Author: Jamie Brown
%
% Created: 25/02/19
%
% Changes
%               
%
%
%

function angleError = CalcAngleError(headingAngle, refAngle)

    %convert from heading to counterclockwise bearing 
    if refAngle < 0 
        refAngleNew = (2*pi) + refAngle;
        %disp("ref convert");
    else
        refAngleNew = refAngle;
    end;
    if headingAngle < 0
        %disp("heading convert");
        headingAngleNew = (2*pi) + headingAngle;
    else
        headingAngleNew = headingAngle;
    end;

    refAngleNew = refAngleNew - pi;
    headingAngleNew = headingAngleNew - pi;

    if refAngleNew > headingAngleNew
        angleError = refAngleNew - headingAngleNew;
    else
        angleError = headingAngleNew - refAngleNew;
    end;