%
% Function to calculate required heading
%
% Author: 
%
% Created: 25/02/19
%
% Changes
%               
%
%
%

function [at_waypoint, desired_psi] = los_auto(cur_x,cur_y,desired_pt)

    % set waypoint
    waypoint = desired_pt;
    %disp(cur_x)
    %disp(cur_y)

    % check to see if in acceptance radius
    cur_radius = sqrt(((cur_x-waypoint(1))^2)+((cur_y-waypoint(2))^2));

    if (cur_radius < 0.1),
        at_waypoint = 1;
    else
        at_waypoint = 0;
    end;

    % Calculate heading
    psi_cal=atan2((waypoint(2)-cur_y),(waypoint(1)-cur_x));

    if isnan(psi_cal),
        if (waypoint(2)-cur_y)>=0,
            desired_psi = pi/2;
        else
            desired_psi = -pi/2;
        end;
    else
        desired_psi = psi_cal;
    end;