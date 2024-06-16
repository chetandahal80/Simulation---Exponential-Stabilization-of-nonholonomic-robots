# Simulation for exponential stabilization of non-holonomic mobile robots

## Introduction 

This repository contains a Python script for generating simulation results based on the findings of paper "Exponential Stabilization of Mobile Robots with Nonholonomic Constraints" (1991) by C. Canudas de Wit and 0. J. Sordalen. The paper presents an exponentially stable controller for a 2 D.O.F non-holonomic robots. 

The simulation shows a two-wheeled robot converging to the origin with zero orientation from any initial state. This is accomplished by allowing the mobility of the robot to align with one of the circles centered on the y-axis and passing through the origin. The coordinate transformation strategy and the controller designed in the paper has been implemented in the code for generating the simulation. 

## Code Description

The "main_script.py" file contains the whole code. The " pygame " library is used to display the simulation results of the two-wheeled robot converging to the origin. When the code runs, it first asks the user to input the initial state of the robot (x-position, y-position and orientation). It then displays the trajectory followed by the robot to converge to the origin with zero orientation. After the simulation, it displays the curve showing the change of x-position, y-position and orientation of the robot with respect to the time. 


