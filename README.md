# Aerial manipulator with a fixed end-effector position: MATLAB simulation
## Overview
<p align="center"><img src="aerial_manipulator_2.PNG" width="400" height="338.5">
  
This repository contains the MATLAB simulation for the plug-pulling task using an aerial manipulator. The aerial manipulator which is configured with a quad-rotor and a 2-DOF robotic manipulator with a gripper try to pull the plug out of the socket by tilting its body. The results of the simulation can be visualized with various plots and a simple 3D animation video.

## How to run
### Installation
1. Clone this repository into your MATLAB workspace.  
```sh
git clone github.com/JH-Byun/AM_plug_task_matlab.git
```
2. Checkout to the "ICRA2022" branch
```sh
git checkout ICRA2022
```
### Execution
1. After referring to the detailed descriptions below, click on the desired main file.
2. Execute the **1st, 3rd, 4th, 5th, 6th** block of the main file. All the simulation results will be produced. (**Caution**: Do not run this file all in one step (ex. pressing F5 button))
3. Execute the **7th (result plot settings)** block to set the parameters for plotting.
4. After the 7th box, there will be an explanation for each visualizing plot. Please read them carefully and execute the desired block to collect the result you want.
5. Also, if you execute the 2nd lastest block in the main file, you can watch the simple 3D animation video which visualizes the whole scenario of the aerial plug-pulling task.
  
## Detailed Descriptions for each file
**1. main_proposed.m**: The aerial plug-pulling simulation with the proposed pq initialization strategy
  
**2. main_pq_zero_init.m**: The aerial plug-pulling simulation with the pq initialization strategy which sets p and q value to zero vectors when the mode switching occurs. 
  
**3. parameters.m**: Parameters utilized for constructing dynamics of the aerial manipulator.
  
**4. utils**
  <ol>
    
  **1) Rx.m**: Rotation matrix with respect to x-axis.
    
  **2) Ry.m**: Rotation matrix with respect to the y-axis.
    
  **3) Rz.m**: Rotation matrix with respect to z-axis.
    
  **4) hat.m**: The function making a se(3) matrix. 
    
  **5) invhat.m**: Inverse process of the hat.m function.
    
  **6) getQ.m**: Calculates a Q matrix from the Euler angles.
    
  **7) getQdot.m**: Calculates a Qdot matrix from the Euler rates.
    
  **8) constraint.m**: Saturates the input value with the given minimum and maximum values.
    
  </ol>
  
**5. dynamics**
  <ol>
    
  **1) FF_dynamics_gen.m**: By computing symbolic operations, it obtains the symbolic matrices of M, C, G, J_tau, A, and Adot which configure the Euler-Lagrange equation of the aerial manipulator system. Successively, it can also produce the following MATLAB functions which will be treated afterward.
    
  **2) getM.m**: Calculates the M matrix from state values and parameters.
    
  **3) getC.m**: Calculates the C matrix from state values, their time-derivative values, and parameters.
    
  **4) getG.m**: Calculates the G matrix from state values and parameters.
    
  **5) getJ_tau.m**: Calculates the J_tau matrix from state values and parameters.
    
  **6) getA.m**: Calculates the A matrix from state values and parameters.
    
  **7) getAdot.m**: Calculates the Adot matrix from state values, their time-derivative values, and parameters.
    
  **8) dynamics.m**: From the previous state values and control inputs (Total thrust and body torque), it calculates the next step's state values using ode45.m function of MATLAB.
 
  **9) robotic_manipulator_control.m**: In the actual experiment, the Dynamixel servo motor which is controlled by desired position angles will be utilized. This function makes the robotic manipulator's servo motors satisfactorily follow the desired angles.
  </ol>
  
**6. result_plots**: contains the result plots from the simulation.
  
**7. controller**
