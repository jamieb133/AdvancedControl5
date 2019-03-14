
%----------------------------------------------%
% Workspace Clear up
close all;
clear all;
clc;
%----------------------------------------------%

%----------------------------------------------%
% Setup Simulation
Vl = -6;
Vr = 6;
sim_time = 30;
dT = 0.05;
xi = zeros(1,24); % intial state for x
LeftS = 0;
RightS = 0;
%----------------------------------------------%

%----------------------------------------------%
% Create Environment

max_x = 10;
max_y = 10;

Obs_Matrix = zeros(max_x/0.01,max_y/0.01);

wall = WallGeneration(-1, 1,1.2,1.2,'h');
wall2 = WallGeneration(-3, -3, -2, 2,'v');
wall3 = WallGeneration(2, 2, -3, 1,'v');
wall4 = WallGeneration(-3, -1, 4, 4,'h');

for x=1:length(wall)
    
    xpos = (wall(x,1)/0.01)+((max_x/2)/0.01);
    ypos = (wall(x,2)/0.01)+((max_y/2)/0.01);
    
    Obs_Matrix(ypos,xpos) = 1;
end

for x=1:length(wall2)
    
    xpos = (wall2(x,1)/0.01)+((max_x/2)/0.01);
    ypos = (wall2(x,2)/0.01)+((max_y/2)/0.01);
    
    Obs_Matrix(ypos,xpos) = 1;
end

for x=1:length(wall3)
    
    
    xpos = int16( (wall3(x,1)/0.01)+((max_x/2)/0.01) );
    ypos = int16( (wall3(x,2)/0.01)+((max_y/2)/0.01) );
    %disp(Obs_Matrix(1, 1))
    ypos
    xpos
    length(wall3)
    Obs_Matrix(ypos,xpos) = 1;
    

    
    %disp(Obs_Matrix(ypos, xpos))
    if ( (isreal(x) && rem(x,1)==0) == false) || (x < 0)
        return
    end;
    if xpos < 0
        xpos;
    elseif ypos< 0
        xpos;
    end;
end

for x=1:length(wall4)
    
    xpos = int16( (wall4(x,1)/0.01)+((max_x/2)/0.01) );
    ypos = int16( (wall4(x,2)/0.01)+((max_y/2)/0.01) );
    
    Obs_Matrix(ypos,xpos) = 1;
end

%----------------------------------------------%

%----------------------------------------------%
ObjectAvoider = readfis('ObjectAvoider.fis');
HeadingController = readfis('HeadingsToTurnCmd.fis');
MotorController = readfis('TurnCommand.fis');

targetX = -1
targetY = -1
targetWaypoint = [targetX, targetY];
simpleGain = 10/pi;
Vd = 1; %drive voltage
motorGain = 7;
Kd = 0.0;
Kp = 10/pi;
Ki = 10/pi;
K = [Kp, Ki, Kd];

prevError = 0;

for outer_loop = 1:(sim_time/dT)

    %----------------------------------------------%
    % Run Model

    sensorOut = ObsSensor1(xi(19), xi(20), [0.2 0], xi(24), Obs_Matrix);
    [atWaypoint, refAngle] = los_auto(xi(19), xi(20), targetWaypoint);

    %calculate current radius from target waypoint
    deltaX = xi(19) - targetX;
    deltaY = xi(20) - targetY;
    radius = sqrt(deltaX^2 + deltaY^2)

    headingAngle = xi(24)
    refAngle

    %disp([xi(19), xi(20), xi(24), refAngle])

    angleError = CalcAngleError(headingAngle, refAngle);
    angleError = refAngle - headingAngle;

    if atWaypoint
        Vl = 0
        Vr = 0
    else
        %this controller provides object avoidance 
        %determines a desired turn command based
        %   solely on the proximity to an obstacle
        [wallSlope, wallProx] = evalfis([sensorOut(:,1) sensorOut(:,2)], ObjectAvoider);

        %this controller determines a desired turn command 
        %   based solely on reference and heading angle fuzzy input sets
        headingCmd = evalfis([refAngle, headingAngle], HeadingController);

        %this controller takes turn commands from the object avoider
        %   & heading controller to determine the output motor voltages
        fuzzyOut = evalfis([headingCmd, radius], MotorController)

        %apply individual voltages calculated from fuzzy controller
        if radius > 1
            Vl = Vd + (motorGain * fuzzyOut(:,1));
            Vr = Vd + (motorGain * fuzzyOut(:,2));
        else 
            %when close to waypoint, reduce drive voltage proportionally
            Vd * radius
            Vl = (Vd * radius) + (motorGain * fuzzyOut(:,1) )
            Vr = (Vd * radius) + (motorGain * fuzzyOut(:,2) )
        end;
        
        %limit the outputs to max voltage range (+- 7.4V)
        if Vl > 7.4
            Vl = 7.4;
        elseif Vl < -7.4
            Vl = -7.4;
        end;

        if Vr > 7.4
            Vr = 7.4;
        elseif Vl < -7.4
            Vl = -7.4;
        end;
        %Vl = -6;
        %Vr = 6;

    end;

    prevError = angleError;





    %apply calculated output voltages to motors
    Va = [Vl; Vl; Vr; Vr];
    [xdot, xi] = full_mdl_motors(Va,xi,0,0,0,0,dT);   
    xi = xi + (xdot*dT); % Euler intergration
    
    % Store varibles
    xdo(outer_loop,:) = xdot;
    xio(outer_loop,:) = xi;
    VlResults(outer_loop,:) = Vl;
    VrResults(outer_loop) = Vr;
    %time(outer_loop) = outer_loop * dT;
    %disp(time(outer_loop));
    %disp(outer_loop*dT);
    %----------------------------------------------%
    
    

    %----------------------------------------------%
    figure(1);
    clf; hold on; grid on; axis([-5,5,-5,5]);
    drawrobot(0.2,xi(20),xi(19),xi(24),'b');
    xlabel('y, m'); ylabel('x, m');
    plot(wall(:,1),wall(:,2),'k-');
    plot(wall2(:,1),wall2(:,2),'k-');

    plot(wall3(:,1),wall3(:,2),'k-');
    plot(wall4(:,1),wall4(:,2),'k-');
    pause(0.001);
    %----------------------------------------------%
    
end
%disp(xio)
%----------------------------------------------%

%----------------------------------------------%
%Plot Variables
%figure(2); plot(xio(:,20),xio(:,19));
%figure(3); plot(xio(:,19));
%figure(4); plot(xio(:,24));

figure(2); 
plot(xio(:,19));
title('Y Distance Travelled');
xlabel('Timesteps');
ylabel('Distance (m)');

figure(3); 
plot(xio(:,20));
title('X Distance Travelled');
xlabel('Timesteps');
ylabel('Distance (m)');

figure(4); 
plot(xio(:,24));
title('PSI Angle');
xlabel('Angle (rads)');
ylabel('Time (s)');

figure(5);
plot(xio(:,20),xio(:,19));
title('X Distance vs Y Distance Travelled');
xlabel('Horizontal Distance (m)');
ylabel('Vertical Distance (m)');

figure(6);
plot(VlResults(:,1));
title('Right Motor Voltage');
xlabel('Horizontal Distance (m)');
ylabel('Vertical Distance (m)');


%----------------------------------------------%