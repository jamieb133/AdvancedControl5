%
% Main simulation with control system
%
% Author: Jamie Brown
% File: run_model.m
%
% Created: 25/02/19
%
% Changes
%               
%
%
%
%----------------------------------------------%
close all;
clear all;
clc;
%----------------------------------------------%

%----------------------------------------------%
%simulation config
sim_time = 25;
fs = 20; %sampling rate
fn = fs / 2; %nyquist 
dT = 1 / fs;
xi = zeros(1,24); % intial state for x
xi(19) = -2; %starting x coordinate
xi(20) = -1; %starting y coordinate
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
    xpos = int16(wall(x,1)/0.01)+((max_x/2)/0.01);
    ypos = int16(wall(x,2)/0.01)+((max_y/2)/0.01);   
    Obs_Matrix(ypos,xpos) = 1;
end

for x=1:length(wall2)
    xpos = int16(wall2(x,1)/0.01)+((max_x/2)/0.01);
    ypos = int16(wall2(x,2)/0.01)+((max_y/2)/0.01);  
    Obs_Matrix(ypos,xpos) = 1;
end

for x=1:length(wall3)
    xpos = int16( (wall3(x,1)/0.01)+((max_x/2)/0.01) );
    ypos = int16( (wall3(x,2)/0.01)+((max_y/2)/0.01) );
    Obs_Matrix(ypos,xpos) = 1;
end

for x=1:length(wall4)
    xpos = int16( (wall4(x,1)/0.01)+((max_x/2)/0.01) );
    ypos = int16( (wall4(x,2)/0.01)+((max_y/2)/0.01) );
    Obs_Matrix(ypos,xpos) = 1;
end

%----------------------------------------------%

%----------------------------------------------%
%setup filters
n = 2;
fCut = fn/1.5; %filter cutoff
wn = fCut / (fs / 2) %normalise cutoff frequency to nyquist
filtType = 'low';
firCoeffs = fir1(n, wn, filtType);
leftFilter = FIRFilter(firCoeffs); %filter for right motor
rightFilter = FIRFilter(firCoeffs); %filter for left motor

sensorDelay = zeros(1, fs*2); %simple moving average buffer for wall proximity
%----------------------------------------------%

%----------------------------------------------%
ObjectAvoider = readfis('ObjectAvoider.fis');
HeadingController = readfis('HeadingsToTurnCmd.fis');
MotorController = readfis('TurnCommand.fis');

targetX = 3.5;
targetY = 2.5

%change these for different scenarios
xi(19) = 0
xi(20) = 1;
xi(24) = pi/2;
targetX = -0.5;
targetY = 3.5;


targetWaypoint = [targetX, targetY];
simpleGain = 10/pi;
Vd = 2.5; %drive voltage
motorGain = 15;
%----------------------------------------------%


%----------------------------------------------%
% MAIN SIMULATION LOOP

for outer_loop = 1:(sim_time/dT)

    %----------------------------------------------%

    %obtain current reference and heading angles 
    [atWaypoint, refAngle] = los_auto(xi(19), xi(20), targetWaypoint);
    headingAngle = xi(24);

    %calculate radius to target waypoint
    deltaX = xi(19) - targetX;
    deltaY = xi(20) - targetY;
    radius = sqrt(deltaX^2 + deltaY^2);

    if radius < 0.05
        %we are within tolerance of 5cm so stop
        Vl = 0;
        Vr = 0;
    else
        %obtain current distance to obstacle
        sensorOut = ObsSensor1(xi(19), xi(20), [0.2 0], xi(24), Obs_Matrix)

        %calculate wall angle and proximity
        wallAngle = atan( (sensorOut(:,2) - sensorOut(:,1)) / 0.2);
        if sensorOut(:,1) < sensorOut(:,2)
            wallProximity = sensorOut(:,1);
        else 
            wallProximity = sensorOut(:,2);
        end;

        %this controller determines a desired turn command (headingCmd)
        %   based solely on reference and heading angle fuzzy input sets
        headingCmd = evalfis([refAngle, headingAngle], HeadingController);

        %take moving average value of wall proximity 
        %   (allows the fuzzy motor controller to estimate whether
        %   or not it is parallel to a wall while the robot "snakes" alongside it)
        sensorDelay = circshift(sensorDelay, 1);
        sensorDelay(1) = wallProximity;
        wallProximityFiltered = mean(sensorDelay)
        %wallProximityFiltered = 1;

        %this controller takes a turn command from the heading controller
        % and determines the output motor voltages depending on whether or 
        %   not a wall is detected or assumed to be parallel
        fuzzyOut = evalfis([headingCmd, radius, wallAngle, wallProximity, wallProximityFiltered], MotorController);

        %generate coefficients for new filter cutoff frequency
        newCoeffs = fir1(n, fuzzyOut(:,3), 'low');
        leftFilter.coeffs = newCoeffs;
        rightFilter.coeffs = newCoeffs;

        %apply lowpass filter to fuzzy motor gains to smoothen
        gainLeft = leftFilter.filter(fuzzyOut(:,1));
        gainRight = rightFilter.filter(fuzzyOut(:,2));
       
        %apply individual voltages calculated from fuzzy controller
        if radius > 1
            %apply an additional constant drive voltage when far from waypoint 
            %   and not in viscinity of a wall
            Vl = Vd + (motorGain * gainLeft);
            Vr = Vd + (motorGain * gainRight);
        else 
            if wallProximity < 1
                %while in viscinity of wall, reduce drive voltage proprtionally
                Vl = (Vd * wallProximity) + (motorGain * gainLeft);
                Vr = (Vd * wallProximity) + (motorGain * gainRight);
            else
                %when close to waypoint, reduce drive voltage proportionally
                Vd * radius;
                Vl = (Vd * radius ) + (motorGain * gainLeft);
                Vr = (Vd * radius ) + (motorGain * gainRight);
            end;
        end;
        
        %limit the outputs to max voltage range (+- 7.4V)
        if Vl > 14.8
            Vl = 14.8;
        elseif Vl < -14.8
            Vl = -14.8;
        end;

        if Vr > 14.8
            Vr = 14.8;
        elseif Vl < -14.8
            Vl = -14.8;
        end;

    end;
    
    %apply calculated output voltages to motors
    Va = [Vl/2; Vl/2; Vr/2; Vr/2];
    [xdot, xi] = full_mdl_motors(Va,xi,0,0,0,0,dT);  
    
    %euler integration
    xi = xi + (xdot*dT); 
    
    %store variables
    xdo(outer_loop,:) = xdot;
    xio(outer_loop,:) = xi;
    VlResults(outer_loop,:) = Vl;
    VrResults(outer_loop) = Vr;
    %----------------------------------------------%
    
    
    %----------------------------------------------%


    %----------------------------------------------%
    %draw robot on graph for each timestep
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
%----------------------------------------------%

%----------------------------------------------%
%PLOTS

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
xlabel('Time (s)');
ylabel('Voltage (V)');

%----------------------------------------------%