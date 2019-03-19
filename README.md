# Advanced Control 5 Lab
A waypoint following and obstacle avoidance system for the simulation of an autonomous robot in MATLAB using fuzzy logic and DSP. 

## Prerequisites
- Matlab Fuzzy Logic Toolbox 
- Matlab Signal Processing Toolbox

## Overview
To run the simulation type:
- `run_model`

- run_model.m: main simulation file
- HeadingsToTurnCommand.fis: fuzzy trajectory controller
- TurnCommand.fis: fuzzy motor controller
- full_mdl_motors.m: model of robot
- ObsSensor1.m: model of distance sensors
- los_auto.m: calculates the reference heading angle
- WallGeneration.m: self explanatory
- FIRFilter.m: sample by sample filter 
- NeuralController.m: basic obstacle avoidance controller for lab 1 (not used in final simulation)


